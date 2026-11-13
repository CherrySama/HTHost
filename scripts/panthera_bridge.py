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
from threading import Lock, Thread

if __package__:
    from .sdk_actions import ActionRun, validate_plan
else:
    from sdk_actions import ActionRun, validate_plan


SDK = Path(__file__).resolve().parents[1] / "Panthera-HT_SDK" / "panthera_python"
ENDPOINT = "/api/panthera/connection"
POSITION_ENDPOINT = "/api/panthera/position"
PAUSE_POSITION_ENDPOINT = "/api/panthera/position/pause"
MODE_ENDPOINT = "/api/panthera/mode"
ACTION_ENDPOINT = "/api/panthera/actions"
STOP_ACTION_ENDPOINT = "/api/panthera/actions/stop"
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
POSITION, GRAVITY, GRAVITY_FRICTION, IMPEDANCE = range(4)
CONTROL_MODES = ("position", "gravity", "gravity_friction", "impedance")
# Position commands can move, stop in place, or hold when explicitly selecting the mode.
MOVE_POSITION, PAUSE_POSITION, STREAM_POSITION = range(len(CONTROL_MODES), len(CONTROL_MODES) + 3)
POSITION_ACTIONS = (MOVE_POSITION, PAUSE_POSITION)
POSITION_TOLERANCE = 0.01
POSITION_SETTLE_SPEED = 0.03
POSITION_SETTLE_SECONDS = 0.15
TORQUE_INTERVAL = 0.005
TORQUE_FEEDBACK_TIMEOUT = 0.15
CONTROL_WATCHDOG_MS = 250
MODE_TRANSITION_SECONDS = 0.25
IMPEDANCE_TRANSITION_SECONDS = 0.5
IMPEDANCE_SETTLE_SPEED = 0.06
IMPEDANCE_SETTLE_SECONDS = 0.15
IMPEDANCE_WAIT_SECONDS = 3
MOTOR_POSITION_MODE = 10
POSITION_RESPONSE_SECONDS = 0.6
TORQUE_LIMITS = (10.0, 10.0, 10.0, 10.0, 10.0, 3.7)
FRICTION_COULOMB = (0.10, 0.12, 0.12, 0.08, 0.03, 0.02)
FRICTION_VISCOUS = (0.04, 0.06, 0.06, 0.04, 0.02, 0.02)
IMPEDANCE_STIFFNESS = (4.0, 10.0, 10.0, 2.0, 2.0, 1.0)
IMPEDANCE_DAMPING = (0.5, 0.8, 0.8, 0.2, 0.2, 0.1)


class RobotSession:
    def __init__(self, context):
        self.id = uuid.uuid4().hex
        self.stop = context.Event()
        self.returned = context.Event()
        self.feedback = context.Array("d", 6 + 6 * MOTOR_SLOTS)
        self.limits = context.Array("d", 2 * ARM_JOINTS + 3)
        self.command = context.Array("d", [0, POSITION, 0, 0, math.nan, 0, *([0] * ARM_JOINTS)])
        self.last_status_request = context.Value("d", 0, lock=False)


class TorqueControl:
    def __init__(self, robot, mode, positions, now, previous=None):
        self.robot = robot
        self.mode = mode
        self.started = now
        self.target = positions[:ARM_JOINTS] if mode == IMPEDANCE else None
        self.previous_torque = previous.torque if previous else None
        self.torque = None
        self.zero = [0.0] * ARM_JOINTS
        self.limits = [min(limit, rated) for limit, rated in zip(TORQUE_LIMITS, robot.max_torque)]
        self.gripper_position = positions[-1]
        self.gripper_gain = 0.0
        self.previous_gripper_gain = previous.gripper_gain if previous else 0.0
        if previous and previous.mode == IMPEDANCE and mode != IMPEDANCE:
            self.gripper_position = previous.gripper_position

    def send(self, positions, velocities, now):
        arm = positions[:ARM_JOINTS]
        velocity = velocities[:ARM_JOINTS]
        gravity = list(self.robot.get_Gravity(arm))
        if len(gravity) != ARM_JOINTS or not all(math.isfinite(value) for value in gravity):
            raise RuntimeError("动力学计算返回了无效重力力矩")
        torque = gravity
        if self.mode == GRAVITY_FRICTION:
            friction = self.robot.get_friction_compensation(
                velocity, FRICTION_COULOMB, FRICTION_VISCOUS, 0.02
            )
            torque = [g + f for g, f in zip(gravity, friction)]
        elif self.mode == IMPEDANCE:
            torque = [
                g + k * (target - position) - b * speed
                for g, k, b, target, position, speed in zip(
                    gravity, IMPEDANCE_STIFFNESS, IMPEDANCE_DAMPING, self.target, arm, velocity
                )
            ]

        if len(torque) != ARM_JOINTS or not all(math.isfinite(value) for value in torque):
            raise RuntimeError("动力学计算返回了无效力矩")
        duration = IMPEDANCE_TRANSITION_SECONDS if self.mode == IMPEDANCE else MODE_TRANSITION_SECONDS
        progress = min(1.0, (now - self.started) / duration)
        blend = progress * progress * (3 - 2 * progress)
        previous = self.previous_torque if self.previous_torque is not None else gravity
        self.torque = [
            max(-limit, min(limit, start + blend * (value - start)))
            for start, value, limit in zip(previous, torque, self.limits)
        ]

        if self.robot.has_gripper:
            gain = self.previous_gripper_gain + blend * (
                float(self.mode == IMPEDANCE) - self.previous_gripper_gain
            )
            self.gripper_gain = gain
            gripper_position = self.gripper_position if gain else 0.0
            if not self.robot.gripper_control_MIT(
                gripper_position, 0.0, 0.0, 0.65 * gain, 0.06 * gain, send=False
            ):
                raise RuntimeError("SDK 拒绝了夹爪力矩指令")
        if not self.robot.pos_vel_tqe_kp_kd(self.zero, self.zero, self.torque, self.zero, self.zero):
            raise RuntimeError("SDK 拒绝了关节力矩指令")
        return progress >= 1.0


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
        applied_command_id = 0
        applied_at = 0
        command_id = 0
        tracking_position = False
        position_settled_since = None
        completed_position_id = 0
        rejected_mode_id = 0
        waiting_impedance_id = 0
        impedance_wait_started = 0
        settled_since = None
        target = None
        speeds = None
        gripper_target = math.nan
        gripper_speed = 0
        return_deadline = 0
        settled_samples = 0
        torque_control = None
        confirmed_mode = POSITION
        watchdog_ready = False
        handoff_until = 0
        handoff_positions = None
        handoff_gripper = math.nan
        position_tolerance = float(robot.config.get("control", {}).get("position_tolerance", 0.01))
        last_complete = time.monotonic()
        parent = multiprocessing.parent_process()
        while not stop.is_set() and parent.is_alive():
            robot.send_get_motor_state_cmd()
            robot.motor_send_cmd()
            if stop.wait(TORQUE_INTERVAL if torque_control else CONTROL_INTERVAL):
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
                    readings[motor_id] = (*values, state.mode, state.fault)
                    del pending[motor_id]

            now = time.monotonic()
            if return_deadline and now >= return_deadline:
                raise RuntimeError("等待到位超时")
            if not pending:
                last_complete = now
                measured = [readings[motor_id] for motor_id in motor_ids]
                if not robot.gripper_installed:
                    measured.append((math.nan, math.nan, math.nan, 0, 0))
                positions, velocities, torques, motor_modes, motor_faults = zip(*measured)
                session.feedback[:] = [
                    now, confirmed_mode, rejected_mode_id, applied_command_id, applied_at, completed_position_id,
                    *positions, *velocities, *torques, *applied_targets, *motor_modes, *motor_faults,
                ]
                pending = {motor.get_motor_id(): motor for motor in motors}
                next_id, mode, speed, next_gripper_speed, next_gripper, deadline, *next_target = session.command[:]
                accept_command = next_id != command_id
                if accept_command and mode == IMPEDANCE:
                    if waiting_impedance_id != next_id:
                        waiting_impedance_id = next_id
                        impedance_wait_started = now
                        settled_since = None
                    if all(abs(velocity) <= IMPEDANCE_SETTLE_SPEED for velocity in velocities[:len(motors)]):
                        if settled_since is None:
                            settled_since = now
                    else:
                        settled_since = None
                    accept_command = settled_since is not None and now - settled_since >= IMPEDANCE_SETTLE_SECONDS
                    if not accept_command and now - impedance_wait_started >= IMPEDANCE_WAIT_SECONDS:
                        rejected_mode_id = command_id = next_id

                if accept_command:
                    command_id = next_id
                    return_deadline = deadline
                    settled_samples = 0
                    tracking_position = mode in POSITION_ACTIONS
                    position_settled_since = None
                    if mode == PAUSE_POSITION:
                        target = [
                            max(lower, min(upper, position))
                            for position, lower, upper in zip(
                                positions, robot.joint_limits["lower"], robot.joint_limits["upper"]
                            )
                        ]
                        hold_speed = min(RETURN_SPEED, min(robot.velocity_limits))
                        speeds = [hold_speed] * ARM_JOINTS
                        gripper_target = (
                            max(gripper_limits[0], min(gripper_limits[1], positions[-1]))
                            if math.isfinite(gripper_limits[0]) else math.nan
                        )
                        gripper_speed = min(hold_speed, GRIPPER_MAX_SPEED)
                    elif mode not in (POSITION, MOVE_POSITION, STREAM_POSITION):
                        torque_control = TorqueControl(robot, mode, positions, now, torque_control)
                        target = None
                        gripper_target = math.nan
                    elif torque_control:
                        handoff_positions = positions[:ARM_JOINTS]
                        if any(
                            not lower <= position <= upper
                            for position, lower, upper in zip(
                                handoff_positions, robot.joint_limits["lower"], robot.joint_limits["upper"]
                            )
                        ):
                            raise RuntimeError("实测姿态超出关节限位，无法切回位置控制")
                        handoff_gripper = math.nan
                        if robot.has_gripper and robot.gripper_limits:
                            if robot.gripper_limits["lower"] <= positions[-1] <= robot.gripper_limits["upper"]:
                                handoff_gripper = positions[-1]
                        hold_speed = min(RETURN_SPEED, min(robot.velocity_limits))
                        target = next_target if deadline else handoff_positions
                        speeds = [speed if deadline else hold_speed] * ARM_JOINTS
                        gripper_target = next_gripper if deadline else handoff_gripper
                        gripper_speed = next_gripper_speed if deadline else min(hold_speed, GRIPPER_MAX_SPEED)
                        handoff_until = now + MODE_TRANSITION_SECONDS if deadline else 0
                        torque_control = None
                    else:
                        if speed:
                            target = next_target
                            speeds = [speed] * ARM_JOINTS
                        if next_gripper_speed:
                            gripper_target = next_gripper
                            gripper_speed = next_gripper_speed

                if (
                    tracking_position
                    and applied_command_id == command_id
                    and completed_position_id != command_id
                ):
                    stopped = all(abs(value) <= POSITION_SETTLE_SPEED for value in velocities[:len(motors)])
                    reached = all(
                        not math.isfinite(goal) or abs(position - goal) <= POSITION_TOLERANCE
                        for position, goal in zip(positions, applied_targets)
                    )
                    if not stopped or not reached:
                        position_settled_since = None
                    elif position_settled_since is None:
                        position_settled_since = now
                    elif now - position_settled_since >= POSITION_SETTLE_SECONDS:
                        completed_position_id = command_id

                handoff = now < handoff_until
                arm_command = handoff_positions if handoff else target
                gripper_command = handoff_gripper if handoff else gripper_target
                has_gripper_target = math.isfinite(gripper_command)
                if (torque_control or arm_command is not None or has_gripper_target) and not stop.is_set():
                    fault_message = describe_motor_faults(motor_faults)
                    if fault_message:
                        raise RuntimeError(fault_message)
                    if not watchdog_ready:
                        timeout = robot.motor_timeout_ms
                        robot.set_timeout(min(timeout, CONTROL_WATCHDOG_MS) if timeout > 0 else CONTROL_WATCHDOG_MS)
                        watchdog_ready = True

                if torque_control and not stop.is_set():
                    if now - session.last_status_request.value > CONTROL_LEASE:
                        raise RuntimeError("控制页面已失联，力矩控制已结束")
                    if torque_control.send(positions, velocities, now):
                        confirmed_mode = torque_control.mode
                    applied_targets[:ARM_JOINTS] = torque_control.target or [math.nan] * ARM_JOINTS
                    applied_targets[-1] = (
                        torque_control.gripper_position
                        if torque_control.mode == IMPEDANCE and robot.has_gripper else math.nan
                    )
                    continue

                if (arm_command is not None or has_gripper_target) and not stop.is_set():
                    if now - session.last_status_request.value > CONTROL_LEASE:
                        raise RuntimeError("控制页面已失联，位置控制已结束")
                    if return_deadline and not handoff:
                        count = MOTOR_SLOTS if has_gripper_target else ARM_JOINTS
                        at_zero = all(
                            abs(position) <= position_tolerance and abs(velocity) <= RETURN_SETTLE_SPEED
                            for position, velocity in zip(positions[:count], velocities[:count])
                        )
                        settled_samples = settled_samples + 1 if at_zero else 0
                        if settled_samples >= RETURN_SETTLE_SAMPLES:
                            session.returned.set()
                            break
                    # Serialize dispatch with the pause request, so a replaced target cannot be sent later.
                    with session.command.get_lock():
                        if session.command[0] != next_id:
                            continue
                        if arm_command is not None:
                            if not robot.Joint_Pos_Vel(arm_command, speeds, iswait=False):
                                raise RuntimeError("SDK 拒绝了关节目标")
                            applied_targets[:ARM_JOINTS] = arm_command
                        if has_gripper_target and not stop.is_set():
                            if not robot.gripper_control(gripper_command, gripper_speed, GRIPPER_MAX_TORQUE):
                                raise RuntimeError("SDK 拒绝了夹爪目标")
                    applied_targets[-1] = gripper_command
                    confirmed_mode = POSITION
                    if applied_command_id != command_id and command_id != rejected_mode_id:
                        applied_command_id = command_id
                        applied_at = now
                elif stop.wait(0.06):
                    break
            elif now - last_complete > (TORQUE_FEEDBACK_TIMEOUT if torque_control else FEEDBACK_TIMEOUT):
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
        self.action = None

    def connect(self):
        with self.lock:
            self._status()
            if self.worker is None:
                self.message = ""
                self.action = None
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
            if self.action and self.action.active:
                return 409, {"message": "请先停止 SDK 动作"}
            if status["positionPhase"]:
                return 409, {"message": "当前运动尚未结束"}
            if status["mode"] != "position" or status["pendingMode"]:
                return 409, {"message": "请切回位置控制后再发送目标"}

            has_arm = "positions" in request
            has_gripper = "gripper" in request
            if not has_arm and not has_gripper:
                return 400, {"message": "缺少关节或夹爪目标"}
            speed = request.get("speed")
            if not is_finite_number(speed) or not 0 < speed <= status["maxSpeed"]:
                return 400, {"message": "速度超出真机允许范围"}

            command = self.session.command
            command_id, _, arm_speed, gripper_speed, gripper, _, *positions = command[:]
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
            command_id += 1
            command[:] = [command_id, MOVE_POSITION, arm_speed, gripper_speed, gripper, 0, *positions]
            return 202, {"message": "位置目标已提交，等待电机反馈", "commandId": int(command_id)}

    def pause_position(self, request):
        with self.lock:
            status = self._status()
            if status["state"] != "connected" or request.get("session") != status["session"]:
                return 409, {"message": "真机当前无法暂停"}
            if self.action and self.action.active:
                return 409, {"message": "请在 SDK 示例面板停止动作"}
            if status["positionPhase"] != "moving" or request.get("commandId") != status["positionStatus"]["commandId"]:
                return 409, {"message": "该位置指令已结束或正在停稳"}
            self.session.last_status_request.value = time.monotonic()
            self._queue_action_pose(PAUSE_POSITION)
            return 202, {"message": "已请求暂停，正在停稳"}

    def set_mode(self, request):
        with self.lock:
            status = self._status()
            if status["state"] != "connected" or request.get("session") != status["session"]:
                return 409, {"message": "真机当前无法切换控制模式"}
            if self.action and self.action.active:
                return 409, {"message": "请先停止 SDK 动作"}
            mode = request.get("mode")
            if mode not in CONTROL_MODES:
                return 400, {"message": "未知的控制模式"}
            if status["positionPhase"]:
                return 409, {"message": "请先暂停或等待当前运动结束"}
            if status["pendingMode"]:
                return 409, {"message": "正在切换控制模式"}
            if mode == status["mode"]:
                return 200, {"mode": mode}
            if any(
                not lower <= position <= upper
                for position, (lower, upper) in zip(status["positions"], status["limits"])
            ):
                return 409, {"message": "实测姿态超出关节限位，无法切换控制模式"}

            command = self.session.command
            self.session.last_status_request.value = time.monotonic()
            command[:] = [
                command[0] + 1, CONTROL_MODES.index(mode), 0, 0, math.nan, 0,
                *([0] * ARM_JOINTS),
            ]
            return 202, {"mode": mode}

    def return_and_disconnect(self, request):
        with self.lock:
            status = self._status()
            if request.get("session") != status.get("session"):
                return 409, {"message": "真机连接已失效，请重新连接"}
            if status["state"] == "homing":
                return 202, status
            if status["state"] != "connected":
                return 409, {"message": "真机当前无法回零"}
            if self.action and self.action.active:
                return 409, {"message": "请先停止 SDK 动作"}

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
                command[0] + 1, POSITION, speed, gripper_speed, gripper_target,
                self.return_deadline, *([0] * ARM_JOINTS),
            ]
            status["state"] = "homing"
            return 202, status

    def _queue_action_pose(self, mode, speed=0, pose=None):
        positions, gripper = pose if pose is not None else ([0] * ARM_JOINTS, None)
        command = self.session.command
        command_id = int(command[0]) + 1
        command[:] = [
            command_id, mode, speed, min(speed, GRIPPER_MAX_SPEED) if gripper is not None else 0,
            gripper if gripper is not None else math.nan, 0, *positions,
        ]
        return command_id

    def start_action(self, request):
        with self.lock:
            status = self._status()
            if status["state"] != "connected" or request.get("session") != status["session"]:
                return 409, {"message": "真机连接已失效"}
            if status["mode"] != "position" or status["pendingMode"] or status["positionPhase"]:
                return 409, {"message": "请等待当前运动结束，并选择位置控制"}
            if self.action and self.action.active:
                return 409, {"message": "已有 SDK 动作正在运行"}
            try:
                run = ActionRun(*validate_plan(request, status, GRIPPER_MAX_SPEED))
            except ValueError as exc:
                return 400, {"message": str(exc)}
            self.action = run
            self.session.last_status_request.value = time.monotonic()
            self._approach_action(run, status, time.monotonic())
            Thread(target=self._run_action, args=(run,), daemon=True).start()
            return 202, run.status()

    def stop_action(self, request):
        with self.lock:
            status = self._status()
            run = self.action
            if (status["state"] != "connected" or request.get("session") != status["session"]
                    or not run or request.get("id") != run.id):
                return 409, {"message": "该 SDK 动作已失效"}
            if run.active and run.phase != "stopping":
                self._stop_action(run, time.monotonic())
            return 202, run.status()

    def _approach_action(self, run, status, now):
        pose = run.segment[3][0]
        positions, gripper = pose
        travel = max(abs(a - b) / run.speed for a, b in zip(positions, status["positions"]))
        if gripper is not None and status["gripper"] is not None:
            travel = max(travel, abs(gripper - status["gripper"]) / min(run.speed, GRIPPER_MAX_SPEED))
        run.phase = "approach"
        run.deadline = now + max(8, travel + 5)
        run.command_id = self._queue_action_pose(MOVE_POSITION, run.speed, pose)

    def _stop_action(self, run, now, message=""):
        run.command_id = self._queue_action_pose(PAUSE_POSITION)
        run.phase = "stopping"
        run.deadline = now + 8
        run.message = message

    def _run_action(self, run):
        while True:
            with self.lock:
                if self.action is not run or not run.active:
                    return
                status = self._status()
                if status["state"] != "connected":
                    run.phase = "error"
                    run.message = status.get("message", "真机连接已结束")
                    return
                now = time.monotonic()
                self._tick_action(run, status, now)
            time.sleep(CONTROL_INTERVAL)

    def _tick_action(self, run, status, now):
        _, dwell, times, poses = run.segment
        motion = status["positionStatus"]
        settled = motion and motion.get("completed") and motion["commandId"] == run.command_id
        if run.phase == "stopping":
            if settled:
                run.phase = "error" if run.message else "stopped"
            elif now >= run.deadline and not run.message:
                run.message = "未能确认已停稳，继续保持停止目标"
            return
        if now - self.session.last_status_request.value > CONTROL_LEASE - 0.5:
            self._stop_action(run, now, "网页失联，动作已取消")
        elif run.phase in ("approach", "finishing"):
            if settled:
                run.phase = "playing" if run.phase == "approach" and len(times) > 1 else "dwell"
                run.since = now
            elif now >= run.deadline:
                self._stop_action(run, now, "未能到达动作目标")
        elif run.phase == "playing":
            elapsed = now - run.since
            if elapsed >= times[-1]:
                run.phase = "finishing"
                run.deadline = now + 8
                run.command_id = self._queue_action_pose(MOVE_POSITION, run.speed, poses[-1])
            else:
                self._queue_action_pose(STREAM_POSITION, run.speed, run.sample(elapsed))
        elif now - run.since >= dwell and run.advance():
            self._approach_action(run, status, now)

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
            last_feedback, confirmed_mode, rejected_id, applied_id, applied_at, completed_id, *feedback = self.session.feedback[:]
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
                    mode = CONTROL_MODES[int(confirmed_mode)]
                    request_id, requested_code = self.session.command[:2]
                    position_phase = None
                    if requested_code in (*POSITION_ACTIONS, STREAM_POSITION):
                        requested_mode = "position"
                        if requested_code in POSITION_ACTIONS and request_id != completed_id:
                            position_phase = "settling" if requested_code == PAUSE_POSITION else "moving"
                    else:
                        requested_mode = CONTROL_MODES[int(requested_code)]
                    rejected = request_id != 0 and request_id == rejected_id
                    limits = self.session.limits[:2 * ARM_JOINTS]
                    max_speed, gripper_min, gripper_max = self.session.limits[2 * ARM_JOINTS:]
                    positions, velocities, torques, targets, motor_modes, motor_faults = (
                        feedback[start:start + MOTOR_SLOTS]
                        for start in range(0, len(feedback), MOTOR_SLOTS)
                    )
                    motors = [
                        {
                            "position": position,
                            "velocity": velocity,
                            "torque": torque,
                            "target": target if math.isfinite(target) else None,
                            "driveMode": int(drive_mode),
                            "fault": int(fault),
                        }
                        for position, velocity, torque, target, drive_mode, fault in zip(
                            positions, velocities, torques, targets, motor_modes, motor_faults
                        )
                        if math.isfinite(position)
                    ]
                    position_status = None
                    if position_phase == "moving":
                        position_status = (
                            describe_position_status(motors, applied_id, last_feedback - applied_at)
                            if applied_id == request_id else
                            {"commandId": int(request_id), "message": "位置目标已提交，等待电机反馈"}
                        )
                    elif requested_code in POSITION_ACTIONS and request_id == completed_id:
                        position_status = {
                            "commandId": int(request_id),
                            "completed": True,
                            "message": "已暂停，保持当前位置" if requested_code == PAUSE_POSITION
                            else "已到达目标，保持当前位置",
                        }
                    pending_mode = requested_mode if requested_mode != mode and not rejected else None
                    return {
                        "state": "homing" if self.return_deadline else "connected",
                        "session": self.session.id,
                        "mode": mode,
                        "pendingMode": pending_mode,
                        "modeMessage": "关节速度未稳定，已保留原控制模式" if rejected else None,
                        "positionPhase": position_phase,
                        "positionStatus": position_status,
                        "action": self.action.status() if self.action else None,
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
        if self.action and self.action.active:
            self.action.phase = "error"
            self.action.message = self.message or "真机连接已结束"


def is_finite_number(value):
    return type(value) in (int, float) and -math.inf < value < math.inf


def motor_name(index):
    return f"J{index + 1}" if index < ARM_JOINTS else "夹爪"


def describe_motor_faults(faults):
    faults = [f"{motor_name(i)}（故障码 {int(code)}）" for i, code in enumerate(faults) if code]
    return f"电机报告故障：{'、'.join(faults)}" if faults else None


def describe_position_status(motors, command_id, elapsed):
    unfinished = [
        (index, motor)
        for index, motor in enumerate(motors)
        if motor["target"] is not None
        and (abs(motor["target"] - motor["position"]) > POSITION_TOLERANCE
             or abs(motor["velocity"]) > POSITION_SETTLE_SPEED)
    ]
    inactive = [
        f"{motor_name(index)}（模式 {motor['driveMode']}）"
        for index, motor in unfinished
        if motor["driveMode"] != MOTOR_POSITION_MODE and abs(motor["velocity"]) <= POSITION_SETTLE_SPEED
    ]
    if not unfinished:
        message = "已到达位置目标，正在确认停稳"
    elif inactive and elapsed >= POSITION_RESPONSE_SECONDS:
        message = f"电机未进入位置控制：{'、'.join(inactive)}"
    elif inactive:
        message = "位置目标已提交，等待电机反馈"
    else:
        message = "电机已响应，尚未到位"
    return {"commandId": int(command_id), "message": message}


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.respond(self.server.robot.status)

    def do_POST(self):
        if self.path == POSITION_ENDPOINT:
            self.respond_with_request(self.server.robot.set_position)
        elif self.path == PAUSE_POSITION_ENDPOINT:
            self.respond_with_request(self.server.robot.pause_position)
        elif self.path == MODE_ENDPOINT:
            self.respond_with_request(self.server.robot.set_mode)
        elif self.path == ACTION_ENDPOINT:
            self.respond_with_request(self.server.robot.start_action, max_length=1_500_000)
        elif self.path == STOP_ACTION_ENDPOINT:
            self.respond_with_request(self.server.robot.stop_action)
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

    def respond_with_request(self, action, max_length=4096):
        if self.headers.get("Content-Type") != "application/json":
            self.send_error(415)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= max_length:
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
