"""Expose the local Panthera SDK connection and its health to HTHost."""

import json
import math
import multiprocessing
import signal
import sys
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock


SDK = Path(__file__).resolve().parents[1] / "Panthera-HT_SDK" / "panthera_python"
ENDPOINT = "/api/panthera/connection"
POSITION_ENDPOINT = "/api/panthera/position"
STARTUP_TIMEOUT = 15
FEEDBACK_TIMEOUT = 2
CONTROL_INTERVAL = 0.04
CONTROL_LEASE = 5
ARM_JOINTS = 6
MOTOR_SLOTS = ARM_JOINTS + 1
GRIPPER_MAX_SPEED = 0.3
GRIPPER_MAX_TORQUE = 0.15
RETURN_SPEED = 0.2
RETURN_SETTLE_SAMPLES = 3
RETURN_SETTLE_SPEED = 0.03


class RobotSession:
    def __init__(self, context):
        self.id = uuid.uuid4().hex
        self.stop = context.Event()
        self.returned = context.Event()
        self.feedback = context.Array("d", 1 + 4 * MOTOR_SLOTS)
        self.limits = context.Array("d", 2 * ARM_JOINTS + 3)
        self.command = context.Array("d", [0, 0, 0, math.nan, 0, *([0] * ARM_JOINTS)])
        self.last_status_request = context.Value("d", 0, lock=False)


def monitor_robot(session, errors):
    stop = session.stop
    try:
        if not any(port.is_char_device() for port in Path("/dev").glob("ttyACM*")):
            raise RuntimeError("未找到机械臂 USB 控制板")

        sys.path.insert(0, str(SDK / "scripts"))
        from Panthera_lib.Panthera import Panthera

        robot = Panthera(str(SDK / "robot_param" / "Follower.yaml"))
        motors = robot.Motors
        expected_count = ARM_JOINTS + int(robot.gripper_installed)
        if len(motors) != expected_count:
            raise RuntimeError("电机数量与从臂配置不匹配")
        motor_ids = [motor.get_motor_id() for motor in motors]
        gripper_limits = [math.nan, math.nan]
        if robot.has_gripper and robot.gripper_limits:
            lower = max(0, robot.gripper_limits["lower"])
            upper = min(2, robot.gripper_limits["upper"])
            if lower < upper:
                gripper_limits = [lower, upper]
        session.limits[:] = [
            *robot.joint_limits["lower"],
            *robot.joint_limits["upper"],
            min(robot.velocity_limits),
            *gripper_limits,
        ]

        pending = {motor.get_motor_id(): motor for motor in motors}
        timestamps = {
            motor_id: motor.get_current_motor_state().time
            for motor_id, motor in pending.items()
        }
        readings = {}
        applied_targets = [math.nan] * MOTOR_SLOTS
        command_id = 0
        target = None
        speeds = None
        gripper_target = math.nan
        gripper_speed = 0
        return_deadline = 0
        settled_samples = 0
        position_tolerance = float(robot.config.get("control", {}).get("position_tolerance", 0.01))
        last_complete = time.monotonic()
        parent = multiprocessing.parent_process()
        while not stop.is_set() and parent.is_alive():
            robot.send_get_motor_state_cmd()
            robot.motor_send_cmd()
            if stop.wait(CONTROL_INTERVAL):
                break

            for motor_id, motor in list(pending.items()):
                state = motor.get_current_motor_state()
                timestamp = state.time
                values = (state.position, state.velocity, state.torque)
                if (
                    state.ID == motor_id
                    and timestamp > 0
                    and timestamp != timestamps[motor_id]
                    and all(math.isfinite(value) and abs(value) < 998 for value in values)
                ):
                    timestamps[motor_id] = timestamp
                    readings[motor_id] = values
                    del pending[motor_id]

            now = time.monotonic()
            if return_deadline and now >= return_deadline:
                raise RuntimeError("等待到位超时")
            if not pending:
                last_complete = now
                measured = [readings[motor_id] for motor_id in motor_ids]
                if not robot.gripper_installed:
                    measured.append((math.nan, math.nan, math.nan))
                positions, velocities, torques = zip(*measured)
                session.feedback[:] = [now, *positions, *velocities, *torques, *applied_targets]
                pending = {motor.get_motor_id(): motor for motor in motors}
                next_id, speed, gripper_speed, gripper_target, deadline, *next_target = session.command[:]
                if next_id != command_id:
                    command_id = next_id
                    target = next_target if speed else None
                    speeds = [speed] * ARM_JOINTS
                    return_deadline = deadline
                    settled_samples = 0
                has_gripper_target = math.isfinite(gripper_target)
                if (target is not None or has_gripper_target) and not stop.is_set():
                    if now - session.last_status_request.value > CONTROL_LEASE:
                        raise RuntimeError("控制页面已失联，位置控制已结束")
                    if return_deadline:
                        count = MOTOR_SLOTS if has_gripper_target else ARM_JOINTS
                        at_zero = all(
                            abs(position) <= position_tolerance and abs(velocity) <= RETURN_SETTLE_SPEED
                            for position, velocity in zip(positions[:count], velocities[:count])
                        )
                        settled_samples = settled_samples + 1 if at_zero else 0
                        if settled_samples >= RETURN_SETTLE_SAMPLES:
                            session.returned.set()
                            break
                    if target is not None:
                        if not robot.Joint_Pos_Vel(target, speeds, iswait=False):
                            raise RuntimeError("SDK 拒绝了关节目标")
                        applied_targets[:ARM_JOINTS] = target
                    if has_gripper_target and not stop.is_set():
                        if not robot.gripper_control(gripper_target, gripper_speed, GRIPPER_MAX_TORQUE):
                            raise RuntimeError("SDK 拒绝了夹爪目标")
                        applied_targets[-1] = gripper_target
                elif stop.wait(0.06):
                    break
            elif now - last_complete > FEEDBACK_TIMEOUT:
                missing = "、".join(motor.get_motor_name() for motor in pending.values())
                raise RuntimeError(f"未收到电机反馈（{missing}）")
    except KeyboardInterrupt:
        pass
    except (Exception, SystemExit) as exc:
        print(f"Panthera: {exc}", file=sys.stderr, flush=True)
        message = str(exc) if isinstance(exc, RuntimeError) else "Panthera SDK 初始化或读取失败"
        errors.send(message)
    finally:
        errors.close()


class RobotConnection:
    def __init__(self):
        self.context = multiprocessing.get_context("spawn")
        self.lock = Lock()
        self.worker = None
        self.message = ""
        self.return_deadline = 0

    def connect(self):
        with self.lock:
            self._status()
            if self.worker is None:
                self.message = ""
                self.session = RobotSession(self.context)
                self.errors, child_errors = self.context.Pipe(duplex=False)
                self.worker = self.context.Process(
                    target=monitor_robot, args=(self.session, child_errors)
                )
                self.started = time.monotonic()
                try:
                    self.worker.start()
                except OSError:
                    self.worker = None
                    self.errors.close()
                    self.message = "无法启动 Panthera SDK 进程"
                finally:
                    child_errors.close()
            return self._status()

    def disconnect(self):
        with self.lock:
            self._stop_worker()
            self.message = ""
            return self._status()

    def status(self):
        with self.lock:
            status = self._status()
            if status["state"] in ("connected", "homing"):
                self.session.last_status_request.value = time.monotonic()
            return status

    def set_position(self, request):
        with self.lock:
            status = self._status()
            if status["state"] != "connected" or request.get("session") != status["session"]:
                return 409, {"message": "真机连接已失效，请重新连接"}

            has_arm = "positions" in request
            has_gripper = "gripper" in request
            if not has_arm and not has_gripper:
                return 400, {"message": "缺少关节或夹爪目标"}
            speed = request.get("speed")
            if not is_finite_number(speed) or not 0 < speed <= status["maxSpeed"]:
                return 400, {"message": "速度超出真机允许范围"}

            command = self.session.command
            command_id, arm_speed, gripper_speed, gripper, _, *positions = command[:]
            if has_arm:
                positions = request["positions"]
                if (
                    not isinstance(positions, list)
                    or len(positions) != ARM_JOINTS
                    or not all(is_finite_number(value) for value in positions)
                ):
                    return 400, {"message": "需要六个有效的关节角度"}
                for index, (position, (lower, upper)) in enumerate(
                    zip(positions, status["limits"]), start=1
                ):
                    if not lower <= position <= upper:
                        return 400, {"message": f"J{index} 目标超出真机关节限位"}
                arm_speed = speed

            if has_gripper:
                limits = status["gripperLimits"]
                if limits is None:
                    return 409, {"message": "夹爪未连接"}
                gripper = request["gripper"]
                if not is_finite_number(gripper) or not limits[0] <= gripper <= limits[1]:
                    return 400, {"message": "目标超出夹爪限位"}
                gripper_speed = min(speed, GRIPPER_MAX_SPEED)

            self.session.last_status_request.value = time.monotonic()
            command[:] = [command_id + 1, arm_speed, gripper_speed, gripper, 0, *positions]
            return 202, {"message": "位置已发送"}

    def return_and_disconnect(self, request):
        with self.lock:
            status = self._status()
            if request.get("session") != status.get("session"):
                return 409, {"message": "真机连接已失效，请重新连接"}
            if status["state"] == "homing":
                return 202, status
            if status["state"] != "connected":
                return 409, {"message": "真机当前无法回零"}

            limits = status["limits"]
            gripper_limits = status["gripperLimits"]
            if any(not lower <= 0 <= upper for lower, upper in limits) or (
                gripper_limits and not gripper_limits[0] <= 0 <= gripper_limits[1]
            ):
                return 409, {"message": "零位超出真机限位"}

            speed = min(RETURN_SPEED, status["maxSpeed"])
            gripper_target = 0 if gripper_limits else math.nan
            gripper_speed = min(speed, GRIPPER_MAX_SPEED) if gripper_limits else 0
            travel_time = max(abs(position) for position in status["positions"]) / speed
            if gripper_limits:
                travel_time = max(travel_time, abs(status["gripper"]) / gripper_speed)
            now = time.monotonic()
            self.return_deadline = now + max(8, travel_time + 5)
            command = self.session.command
            self.session.last_status_request.value = now
            command[:] = [
                command[0] + 1, speed, gripper_speed, gripper_target,
                self.return_deadline, *([0] * ARM_JOINTS),
            ]
            status["state"] = "homing"
            return 202, status

    def _status(self):
        if self.worker is not None:
            if self.session.returned.is_set():
                self._stop_worker()
                return {"state": "disconnected"}
            if self.errors.poll():
                try:
                    self.message = self.errors.recv()
                except EOFError:
                    self.message = "Panthera SDK 进程已退出"
            elif not self.worker.is_alive():
                self.message = "Panthera SDK 进程已退出"

            now = time.monotonic()
            last_feedback, *feedback = self.session.feedback[:]
            if not self.message:
                if self.return_deadline and now >= self.return_deadline:
                    self.message = "等待到位超时"
                elif last_feedback and now - last_feedback > FEEDBACK_TIMEOUT + 1:
                    self.message = "机械臂反馈超时"
                elif not last_feedback and now - self.started > STARTUP_TIMEOUT:
                    self.message = "机械臂连接超时"
                elif not last_feedback:
                    return {"state": "connecting"}
                else:
                    limits = self.session.limits[:2 * ARM_JOINTS]
                    max_speed, gripper_min, gripper_max = self.session.limits[2 * ARM_JOINTS:]
                    positions, velocities, torques, targets = (
                        feedback[start:start + MOTOR_SLOTS]
                        for start in range(0, len(feedback), MOTOR_SLOTS)
                    )
                    motors = [
                        {
                            "position": position,
                            "velocity": velocity,
                            "torque": torque,
                            "target": target if math.isfinite(target) else None,
                        }
                        for position, velocity, torque, target in zip(positions, velocities, torques, targets)
                        if math.isfinite(position)
                    ]
                    return {
                        "state": "homing" if self.return_deadline else "connected",
                        "session": self.session.id,
                        "positions": positions[:ARM_JOINTS],
                        "gripper": motors[-1]["position"] if len(motors) == MOTOR_SLOTS else None,
                        "sampleTime": last_feedback - self.started,
                        "motors": motors,
                        "limits": list(zip(limits[:ARM_JOINTS], limits[ARM_JOINTS:])),
                        "maxSpeed": max_speed,
                        "gripperLimits": [gripper_min, gripper_max] if math.isfinite(gripper_min) else None,
                    }
            if self.return_deadline:
                self.message = f"回零未完成：{self.message}"
            self._stop_worker()

        if self.message:
            return {"state": "error", "message": self.message}
        return {"state": "disconnected"}

    def _stop_worker(self):
        if self.worker is None:
            return
        self.session.stop.set()
        self.worker.join(timeout=1)
        if self.worker.is_alive():
            self.worker.terminate()
            self.worker.join(timeout=1)
        if self.worker.is_alive():
            self.worker.kill()
            self.worker.join()
        self.worker.close()
        self.errors.close()
        self.worker = None
        self.return_deadline = 0


def is_finite_number(value):
    return type(value) in (int, float) and -math.inf < value < math.inf


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.respond(self.server.robot.status)

    def do_POST(self):
        if self.path == POSITION_ENDPOINT:
            self.respond_with_request(self.server.robot.set_position)
        else:
            self.respond(self.server.robot.connect, mutation=True)

    def do_DELETE(self):
        if self.path != ENDPOINT:
            self.send_error(404)
            return
        self.respond_with_request(self.server.robot.return_and_disconnect)

    def respond(self, action, mutation=False):
        if self.path != ENDPOINT:
            self.send_error(404)
            return
        if mutation and self.headers.get("Content-Type") != "application/json":
            self.send_error(415)
            return
        self.send_json(200, action())

    def respond_with_request(self, action):
        if self.headers.get("Content-Type") != "application/json":
            self.send_error(415)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096:
                raise ValueError
            self.connection.settimeout(2)
            request = json.loads(self.rfile.read(length))
            if not isinstance(request, dict):
                raise ValueError
        except (ValueError, OSError):
            self.send_json(400, {"message": "无效的真机控制请求"})
            return
        self.send_json(*action(request))

    def send_json(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_request(self, code="-", size="-"):
        if self.command != "GET" or code != 200:
            super().log_request(code, size)


def main():
    def shutdown(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, shutdown)
    with ThreadingHTTPServer(("127.0.0.1", 5174), Handler) as server:
        server.robot = RobotConnection()
        print("Panthera 连接服务已启动，监听 127.0.0.1:5174", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.robot.disconnect()


if __name__ == "__main__":
    main()
