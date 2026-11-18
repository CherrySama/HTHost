"""Expose the local Panthera SDK connection and its health to HTHost."""

import json
import math
import multiprocessing
import signal
import time
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Lock, Thread

if __package__:
    from .panthera_control import (
        ARM_JOINTS, ControlCommand, CONTROL_INTERVAL, CONTROL_LEASE, CONTROL_MODES, FEEDBACK_TIMEOUT,
        GRIPPER_MAX_SPEED, MOTOR_POSITION_MODE, MOTOR_SLOTS, MOVE_POSITION, NATIVE_MODES,
        PAUSE_COMMANDS, PAUSE_POSITION, POINT_PAUSE, POINT_POSITION, POSITION_ACTIONS,
        POSITION_RESPONSE_SECONDS, POSITION_SETTLE_SPEED, POSITION_TOLERANCE, RETURN_SPEED, RobotSession,
        SDK, SDK_REPLAY, SDK_SINE, STARTUP_TIMEOUT, STREAM_POSITION,
        SharedStateTimeout, WRITE_ZERO, ZERO_QUEUED, ZERO_STILL_SECONDS, ZERO_TOLERANCE,
        ZERO_UNVERIFIED, ZERO_VERIFYING, ZERO_WRITE_TIMEOUT, ZERO_WRITING, motor_name,
        run_sdk_worker
    )
    from .sdk_actions import (ActionRun, TrajectoryRecording,
                              prepare_native_trajectory, validate_plan)
else:
    from panthera_control import (
        ARM_JOINTS, ControlCommand, CONTROL_INTERVAL, CONTROL_LEASE, CONTROL_MODES, FEEDBACK_TIMEOUT,
        GRIPPER_MAX_SPEED, MOTOR_POSITION_MODE, MOTOR_SLOTS, MOVE_POSITION, NATIVE_MODES,
        PAUSE_COMMANDS, PAUSE_POSITION, POINT_PAUSE, POINT_POSITION, POSITION_ACTIONS,
        POSITION_RESPONSE_SECONDS, POSITION_SETTLE_SPEED, POSITION_TOLERANCE, RETURN_SPEED, RobotSession,
        SDK, SDK_REPLAY, SDK_SINE, STARTUP_TIMEOUT, STREAM_POSITION,
        SharedStateTimeout, WRITE_ZERO, ZERO_QUEUED, ZERO_STILL_SECONDS, ZERO_TOLERANCE,
        ZERO_UNVERIFIED, ZERO_VERIFYING, ZERO_WRITE_TIMEOUT, ZERO_WRITING, motor_name,
        run_sdk_worker
    )
    from sdk_actions import (ActionRun, TrajectoryRecording,
                             prepare_native_trajectory, validate_plan)


ENDPOINT = "/api/panthera/connection"
PAGE_ENDPOINT = "/api/panthera/connection/page"
PAGE_CLOSE_DELAY = 3
POSITION_ENDPOINT = "/api/panthera/position"
PAUSE_POSITION_ENDPOINT = "/api/panthera/position/pause"
MODE_ENDPOINT = "/api/panthera/mode"
CALIBRATION_ENDPOINT = "/api/panthera/calibration/zero"
ACTION_ENDPOINT = "/api/panthera/actions"
STOP_ACTION_ENDPOINT = "/api/panthera/actions/stop"
ESTIMATE_ACTION_ENDPOINT = "/api/panthera/actions/estimate"
START_RECORDING_ENDPOINT = "/api/panthera/actions/record/start"
STOP_RECORDING_ENDPOINT = "/api/panthera/actions/record/stop"


class RobotConnection:
    def __init__(self):
        self.context = multiprocessing.get_context("spawn")
        # 仅记录本桥接进程的配置历史，不代表设备超时寄存器已读回或已清除。
        self.timed_control_started = self.context.RawValue("b", False)
        self.lock = Lock()
        self.worker = None
        self.session = None
        self.message = ""
        self.return_deadline = 0
        self.zero_deadline = 0
        self.zero_failure = None
        self.action = None
        self.recording = None
        self.pages = {}

    def connect(self):
        with self.lock:
            status = self._status()
            if self.worker is not None:
                return status
            self.message = ""
            self.zero_failure = None
            self.action = None
            self.recording = None
            self.pages.clear()
            self.session = RobotSession(self.context, self.timed_control_started)
            self.errors, child_errors = self.context.Pipe(duplex=False)
            self.worker = self.context.Process(
                target=run_sdk_worker, args=(self.session, child_errors)
            )
            self.started = time.monotonic()
            try:
                self.worker.start()
            except OSError:
                self.worker = None
                self.session = None
                self.errors.close()
                self.message = "无法启动 Panthera SDK 进程"
            finally:
                child_errors.close()
            return self._status()

    def disconnect(self):
        with self.lock:
            if not self._stop_worker():
                self.message = "Panthera SDK 进程未能结束"
                return self._connection_error()
            self.message = ""
            self.zero_failure = None
            return self._status()

    def status(self):
        with self.lock:
            status = self._status()
            if status["state"] in ("connected", "homing", "calibrating"):
                self.session.last_status_request.value = time.monotonic()
            return status

    def write_zero(self, request):
        with self.lock, self._recover_shared_state():
            status = self._status()
            if status["state"] != "connected" or request.get("session") != status["session"]:
                return 409, {"message": "真机当前无法写入零位"}
            if not status["calibration"]["ready"]:
                return 409, {"message": "请等待电机静止、反馈正常且其他控制结束"}
            expected = request.get("positions")
            motors = status["motors"]
            if (not isinstance(expected, list) or len(expected) != len(motors)
                    or any(not is_finite_number(value) or abs(value - motor["position"]) > ZERO_TOLERANCE
                           for value, motor in zip(expected, motors))):
                return 409, {"message": "电机位置已变化，请重新核对校准姿态"}
            session = self.session
            now = time.monotonic()
            self.zero_deadline = now + ZERO_WRITE_TIMEOUT
            session.zero[:] = [ZERO_QUEUED, 0, *([math.nan] * MOTOR_SLOTS)]
            session.last_status_request.value = now
            session.command[:] = ControlCommand(
                session.command[0] + 1, WRITE_ZERO,
                gripper=expected[-1] if len(expected) == MOTOR_SLOTS else math.nan,
                targets=expected[:ARM_JOINTS],
            ).pack()
            Thread(target=self._watch_zero, args=(session,), daemon=True).start()
            return 202, {"phase": "queued"}

    def _watch_zero(self, session):
        while True:
            with self.lock:
                if self.worker is None or self.session is not session or not self.zero_deadline:
                    return
                self._status()
            time.sleep(0.2)

    def set_position(self, request):
        with self.lock, self._recover_shared_state():
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
            command_id = command[0]
            positions, arm_speed = [0] * ARM_JOINTS, 0
            gripper, gripper_speed = math.nan, 0
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
            command[:] = ControlCommand(
                command_id, POINT_POSITION, speed=arm_speed, gripper_speed=gripper_speed,
                gripper=gripper, targets=positions,
            ).pack()
            return 202, {"message": "位置目标已提交，等待电机反馈", "commandId": int(command_id)}

    def pause_position(self, request):
        with self.lock, self._recover_shared_state():
            status = self._status()
            if status["state"] != "connected" or request.get("session") != status["session"]:
                return 409, {"message": "真机当前无法暂停"}
            if self.action and self.action.active:
                return 409, {"message": "请在 SDK 示例面板停止动作"}
            if status["positionPhase"] != "moving" or request.get("commandId") != status["positionStatus"]["commandId"]:
                return 409, {"message": "该位置指令已结束或正在停稳"}
            self.session.last_status_request.value = time.monotonic()
            self._queue_action_pose(POINT_PAUSE)
            return 202, {"message": "已请求暂停，正在停稳"}

    def set_mode(self, request):
        with self.lock, self._recover_shared_state():
            status = self._status()
            if status["state"] != "connected" or request.get("session") != status["session"]:
                return 409, {"message": "真机当前无法切换控制模式"}
            if self.action and self.action.active:
                return 409, {"message": "请先停止 SDK 动作"}
            if self.recording and self.recording.active:
                return 409, {"message": "请先停止轨迹录制"}
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
            command[:] = ControlCommand(command[0] + 1, CONTROL_MODES.index(mode)).pack()
            return 202, {"mode": mode}

    def return_and_disconnect(self, request):
        with self.lock, self._recover_shared_state():
            return self._return_and_disconnect(request)

    def _return_and_disconnect(self, request):
        status = self._status()
        if request.get("session") != status.get("session"):
            return 409, {"message": "真机连接已失效，请重新连接"}
        if status["state"] == "homing":
            return 202, status
        if status["state"] != "connected":
            return 409, {"message": "真机当前无法回零"}
        if self.action and self.action.active:
            return 409, {"message": "请先停止 SDK 动作"}
        if self.recording and self.recording.active:
            return 409, {"message": "请先停止轨迹录制"}

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
        command[:] = ControlCommand(
            command[0] + 1, POINT_POSITION, speed=speed, gripper_speed=gripper_speed,
            gripper=gripper_target, return_deadline=self.return_deadline,
        ).pack()
        Thread(target=self._watch_return, args=(self.session,), daemon=True).start()
        status["state"] = "homing"
        return 202, status

    def _watch_return(self, session):
        while True:
            with self.lock:
                if self.session is not session or not self.return_deadline:
                    return
                self._status()
            time.sleep(0.2)

    def page_lifecycle(self, request):
        with self.lock:
            if self.session is None or request.get("session") != self.session.id:
                return 409, {"message": "真机连接已失效"}
            page, stamp, leaving = request.get("page"), request.get("stamp"), request.get("leaving")
            if (not isinstance(page, str) or not 0 < len(page) <= 100
                    or not is_finite_number(stamp) or type(leaving) is not bool):
                return 400, {"message": "无效的页面状态"}
            previous = self.pages.get(page)
            if previous is not None and stamp <= previous[0]:
                return 200, {"message": "已忽略旧页面通知"}
            self.pages[page] = (stamp, leaving)
            if leaving:
                Thread(target=self._return_after_page_close,
                       args=(self.session, page, stamp), daemon=True).start()
            return 202, {"message": "页面状态已更新"}

    def _return_after_page_close(self, session, page, stamp):
        time.sleep(PAGE_CLOSE_DELAY)
        # Reuse the manual disconnect entry while holding the same connection lock.
        with self.lock, self._recover_shared_state():
            if (self.session is not session or self.pages.get(page) != (stamp, True)
                    or any(not leaving for _, leaving in self.pages.values())):
                return
            code, result = self._return_and_disconnect({"session": session.id})
            if code != 202:
                print(f"网页关闭后未回零：{result.get('message', '')}", flush=True)

    def start_recording(self, request):
        with self.lock:
            status = self._status()
            if status["state"] != "connected" or request.get("session") != status["session"]:
                return 409, {"message": "真机连接已失效"}
            if self.recording and self.recording.active:
                return 409, {"message": "真机已在录制"}
            if (status["mode"] not in ("gravity", "gravity_friction") or status["pendingMode"]
                    or status["positionPhase"] or (self.action and self.action.active)):
                return 409, {"message": "请先切换到重力补偿模式"}
            recording = self.recording = TrajectoryRecording(status["session"])
            recording.sample(status)
            Thread(target=self._record_trajectory, args=(recording,), daemon=True).start()
            return 202, recording.status()

    def _record_trajectory(self, recording):
        while True:
            with self.lock:
                if self.recording is not recording or not recording.active:
                    return
                status = self._status()
                if status["state"] != "connected" or status["session"] != recording.session:
                    recording.active = False
                    return
                recording.sample(status)
            time.sleep(0.01)

    def stop_recording(self, request):
        with self.lock:
            recording = self.recording
            if (not recording or request.get("session") != recording.session
                    or request.get("id") != recording.id):
                return 409, {"message": "该轨迹录制已失效"}
            recording.active = False
            return 200, {"frames": recording.frames}

    def _queue_action_pose(self, mode, speed=0, pose=None):
        positions, gripper = pose if pose is not None else ([0] * ARM_JOINTS, None)
        command = self.session.command
        command_id = int(command[0]) + 1
        command[:] = ControlCommand(
            command_id, mode, speed=speed,
            gripper_speed=min(speed, GRIPPER_MAX_SPEED) if gripper is not None else 0,
            gripper=gripper if gripper is not None else math.nan, targets=positions,
        ).pack()
        return command_id

    def _queue_native_trajectory(self, trajectory):
        command = self.session.command
        mode = SDK_SINE if trajectory.kind == "joint-sine" else SDK_REPLAY
        data = [value for row in trajectory.rows for value in row]
        with command.locked():
            self.session.trajectory[:len(data)] = data
            command_id = int(command[0]) + 1
            command[:] = ControlCommand(
                command_id, mode, speed=trajectory.duration,
                targets=(len(trajectory.rows), *([0] * (ARM_JOINTS - 1))),
            ).pack()
        return command_id

    @staticmethod
    def _prepare_native(kind, points, speed, gripper_speed, status):
        script_path = str(SDK / "scripts")
        if script_path not in sys.path:
            sys.path.insert(0, script_path)
        from Panthera_lib import TrajectoryRecorder

        return prepare_native_trajectory(kind, points, speed, gripper_speed, status, TrajectoryRecorder)

    def start_action(self, request):
        with self.lock, self._recover_shared_state():
            status = self._status()
            if status["state"] != "connected" or request.get("session") != status["session"]:
                return 409, {"message": "真机连接已失效"}
            if self.recording and self.recording.active:
                return 409, {"message": "请先停止轨迹录制"}
            if status["mode"] != "position" or status["pendingMode"] or status["positionPhase"]:
                return 409, {"message": "请等待当前运动结束，并选择位置控制"}
            if self.action and self.action.active:
                return 409, {"message": "已有 SDK 动作正在运行"}
            try:
                run = ActionRun(*validate_plan(request, status, GRIPPER_MAX_SPEED, self._prepare_native))
            except ValueError as exc:
                return 400, {"message": str(exc)}
            self.action = run
            self.session.last_status_request.value = time.monotonic()
            self._approach_action(run, status, time.monotonic())
            Thread(target=self._run_action, args=(run,), daemon=True).start()
            return 202, run.status()

    def estimate_action(self, request):
        with self.lock:
            status = self._status()
            if status["state"] != "connected" or request.get("session") != status["session"]:
                return 409, {"message": "真机连接已失效"}
        try:
            _, loops, segments = validate_plan(request, status, GRIPPER_MAX_SPEED, self._prepare_native)
        except ValueError as exc:
            return 400, {"message": str(exc)}
        return 200, {"seconds": loops * sum(times[-1] + dwell for _, dwell, times, _, _ in segments)}

    def stop_action(self, request):
        with self.lock, self._recover_shared_state():
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
                try:
                    self._tick_action(run, status, now)
                except SharedStateTimeout as exc:
                    self._fail_worker(str(exc))
                    return
            time.sleep(CONTROL_INTERVAL)

    def _tick_action(self, run, status, now):
        _, dwell, times, poses, native = run.segment
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
                if native and run.phase == "playing":
                    run.command_id = self._queue_native_trajectory(native)
                    run.deadline = now + 3
            elif now >= run.deadline:
                self._stop_action(run, now, "未能到达动作目标")
        elif run.phase == "playing":
            if native:
                if status["appliedCommand"] != run.command_id:
                    if now >= run.deadline:
                        self._stop_action(run, now, "SDK 轨迹未开始")
                    return
                elapsed = now - status["appliedAt"]
            else:
                elapsed = now - run.since
            if elapsed >= times[-1]:
                run.phase = "finishing"
                run.deadline = now + 8
                run.command_id = self._queue_action_pose(MOVE_POSITION, run.speed, poses[-1])
            elif not native:
                self._queue_action_pose(STREAM_POSITION, run.speed, run.sample(elapsed))
        elif now - run.since >= dwell and run.advance():
            self._approach_action(run, status, now)

    def _status(self):
        try:
            return self._read_status()
        except SharedStateTimeout as exc:
            return self._fail_worker(str(exc))

    @contextmanager
    def _recover_shared_state(self):
        try:
            yield
        except SharedStateTimeout as exc:
            status = self._fail_worker(str(exc))
            raise SharedStateTimeout(status["message"]) from exc

    def _poll_worker_exit(self):
        """处理工作进程退出；调用方持有连接锁，返回终态或 None。"""
        if self.message:
            self._stop_worker()
            return self._connection_error()
        if self.session.returned.value:
            if self._stop_worker():
                return {"state": "disconnected"}
            self.message = "已回零，但 Panthera SDK 进程未能结束"
            return self._connection_error()
        if self.errors.poll():
            try:
                message = self.errors.recv()
            except (EOFError, OSError):
                message = "Panthera SDK 进程已退出"
            return self._fail_worker(message)
        elif not self.worker.is_alive():
            return self._fail_worker("Panthera SDK 进程已退出")
        return None

    def _read_status(self):
        if self.worker is not None:
            terminal = self._poll_worker_exit()
            if terminal is not None:
                return terminal

            now = time.monotonic()
            zero_phase, still_since, *zero_positions = self.session.zero[:]
            zero_active = zero_phase in (ZERO_QUEUED, ZERO_WRITING, ZERO_VERIFYING)
            if zero_active and now >= self.zero_deadline:
                self.message = "零位写入或反馈验证超时，请核对电机零位后再控制"
            elif zero_phase == ZERO_UNVERIFIED:
                self.message = "零位反馈未通过验证，请核对各电机零位后再控制"
            elif not zero_active:
                self.zero_deadline = 0
            last_feedback, confirmed_mode, rejected_id, applied_id, applied_at, completed_id, *feedback = self.session.feedback[:]
            if not self.message:
                if self.return_deadline and now >= self.return_deadline:
                    self.message = "等待到位超时"
                elif not zero_active and last_feedback and now - last_feedback > FEEDBACK_TIMEOUT + 1:
                    self.message = "机械臂反馈超时"
                elif not last_feedback and now - self.started > STARTUP_TIMEOUT:
                    self.message = "机械臂连接超时"
                elif not last_feedback:
                    return {"state": "connecting"}
                else:
                    mode = CONTROL_MODES[int(confirmed_mode)]
                    request_id, requested_code, arm_speed, gripper_speed = self.session.command[:4]
                    position_phase = None
                    if requested_code in (*POSITION_ACTIONS, STREAM_POSITION, WRITE_ZERO, *NATIVE_MODES):
                        requested_mode = "position"
                        if requested_code in POSITION_ACTIONS and request_id != completed_id:
                            position_phase = "settling" if requested_code in PAUSE_COMMANDS else "moving"
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
                        command_motors = motors
                        if requested_code == POINT_POSITION:
                            command_motors = [
                                motor if (arm_speed if index < ARM_JOINTS else gripper_speed)
                                else {**motor, "target": None}
                                for index, motor in enumerate(motors)
                            ]
                        position_status = (
                            describe_position_status(command_motors, applied_id, last_feedback - applied_at)
                            if applied_id == request_id else
                            {"commandId": int(request_id), "message": "位置目标已提交，等待电机反馈"}
                        )
                    elif requested_code in POSITION_ACTIONS and request_id == completed_id:
                        position_status = {
                            "commandId": int(request_id),
                            "completed": True,
                            "message": "已暂停，保持当前位置" if requested_code in PAUSE_COMMANDS
                            else "已到达目标，保持当前位置",
                        }
                    pending_mode = requested_mode if requested_mode != mode and not rejected else None
                    calibration = {
                        "phase": ("idle", "queued", "writing", "verifying", "done", "rejected", "unverified")[int(zero_phase)],
                        "ready": bool(not zero_active and mode == "position" and not pending_mode
                                      and not position_phase and not self.return_deadline
                                      and not (self.action and self.action.active)
                                      and not (self.recording and self.recording.active)
                                      and now - last_feedback < 0.25 and still_since
                                      and last_feedback - still_since >= ZERO_STILL_SECONDS
                                      and not any(motor_faults)),
                        "positions": [value if math.isfinite(value) else None for value in zero_positions[:len(motors)]],
                        "tolerance": ZERO_TOLERANCE,
                    }
                    return {
                        "state": "calibrating" if zero_active else "homing" if self.return_deadline else "connected",
                        "calibration": calibration,
                        "session": self.session.id,
                        "mode": mode,
                        "pendingMode": pending_mode,
                        "modeMessage": "关节速度未稳定，已保留原控制模式" if rejected else None,
                        "positionPhase": position_phase,
                        "positionStatus": position_status,
                        "appliedCommand": int(applied_id),
                        "appliedAt": applied_at,
                        "action": self.action.status() if self.action else None,
                        "recording": self.recording.status() if self.recording else None,
                        "positions": positions[:ARM_JOINTS],
                        "gripper": motors[-1]["position"] if len(motors) == MOTOR_SLOTS else None,
                        "sampleTime": last_feedback - self.started,
                        "motors": motors,
                        "limits": list(zip(limits[:ARM_JOINTS], limits[ARM_JOINTS:])),
                        "maxSpeed": max_speed,
                        "gripperLimits": [gripper_min, gripper_max] if math.isfinite(gripper_min) else None,
                    }
            if zero_active or zero_phase == ZERO_UNVERIFIED:
                return self._fail_worker(self.message, zero_positions)
            return self._fail_worker(self.message)

        if self.message:
            return self._connection_error()
        return {"state": "disconnected"}

    def _connection_error(self):
        return {"state": "error", "message": self.message, "calibration": self.zero_failure}

    def _fail_worker(self, message, zero_positions=None):
        if self.return_deadline:
            message = f"回零未完成：{message}"
        if self.zero_deadline or zero_positions is not None:
            message = f"零位结果未确认：{message}"
            self.zero_failure = {
                "phase": "unverified",
                "positions": [value if math.isfinite(value) else None for value in (zero_positions or [])],
                "tolerance": ZERO_TOLERANCE,
            }
        self.message = message
        self._stop_worker()
        return self._connection_error()

    def _stop_worker(self):
        if self.recording:
            self.recording.active = False
        if self.worker is None:
            return True
        if self.action and self.action.active:
            self.action.phase = "error"
            self.action.message = self.message or "真机连接已结束"
        if self.worker.is_alive():
            self.session.stop.set()
            self.worker.join(timeout=1)
        if self.worker.is_alive():
            self.worker.terminate()
            self.worker.join(timeout=1)
        if self.worker.is_alive():
            self.worker.kill()
            self.worker.join(timeout=1)
        if self.worker.is_alive():
            return False
        self.worker.close()
        self.errors.close()
        self.worker = None
        self.session = None
        self.return_deadline = 0
        self.zero_deadline = 0
        return True


def is_finite_number(value):
    return type(value) in (int, float) and -math.inf < value < math.inf


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
        if self.path == PAGE_ENDPOINT:
            self.respond_with_request(self.server.robot.page_lifecycle)
        elif self.path == POSITION_ENDPOINT:
            self.respond_with_request(self.server.robot.set_position)
        elif self.path == PAUSE_POSITION_ENDPOINT:
            self.respond_with_request(self.server.robot.pause_position)
        elif self.path == MODE_ENDPOINT:
            self.respond_with_request(self.server.robot.set_mode)
        elif self.path == CALIBRATION_ENDPOINT:
            self.respond_with_request(self.server.robot.write_zero)
        elif self.path == ACTION_ENDPOINT:
            self.respond_with_request(self.server.robot.start_action, max_length=1_500_000)
        elif self.path == ESTIMATE_ACTION_ENDPOINT:
            self.respond_with_request(self.server.robot.estimate_action, max_length=1_500_000)
        elif self.path == STOP_ACTION_ENDPOINT:
            self.respond_with_request(self.server.robot.stop_action)
        elif self.path == START_RECORDING_ENDPOINT:
            self.respond_with_request(self.server.robot.start_recording)
        elif self.path == STOP_RECORDING_ENDPOINT:
            self.respond_with_request(self.server.robot.stop_recording)
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
        try:
            result = action(request)
        except SharedStateTimeout as exc:
            result = 409, {"message": str(exc)}
        self.send_json(*result)

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
