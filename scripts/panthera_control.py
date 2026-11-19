"""SDK 工作进程、反馈采集和控制模式交接；不包含 HTTP 或网页生命周期。"""

import logging
import math
import multiprocessing
import os
import subprocess
import sys
import time
from contextlib import contextmanager
from typing import NamedTuple
from pathlib import Path
import uuid

if __package__:
    from .control_trace import ControlTrace
    from .sdk_actions import MAX_NATIVE_FRAMES, NATIVE_WIDTH, native_target
else:
    from control_trace import ControlTrace
    from sdk_actions import MAX_NATIVE_FRAMES, NATIVE_WIDTH, native_target

SDK = Path(__file__).resolve().parents[1] / "Panthera-HT_SDK" / "panthera_python"

STARTUP_TIMEOUT = 15
FEEDBACK_TIMEOUT = 2
SHARED_STATE_TIMEOUT = 0.5
CONTROL_INTERVAL = 0.04
CONTROL_LEASE = 5
ARM_JOINTS = 6
MOTOR_SLOTS = ARM_JOINTS + 1
GRIPPER_MAX_SPEED = 0.3
GRIPPER_MAX_TORQUE = 0.15
SDK_SINE_INTERVAL = 0.002
SDK_REPLAY_INTERVAL = 0.01
SDK_REPLAY_KP = (30.0, 40.0, 55.0, 15.0, 7.0, 5.0)
SDK_REPLAY_KD = (3.0, 4.0, 5.5, 1.5, 0.7, 0.5)
SDK_REPLAY_TORQUE = (15.0, 30.0, 30.0, 15.0, 5.0, 5.0)
RETURN_SPEED = 0.2
RETURN_SETTLE_SAMPLES = 3
RETURN_SETTLE_SPEED = 0.03
POSITION, GRAVITY, GRAVITY_FRICTION, IMPEDANCE = range(4)
CONTROL_MODES = ("position", "gravity", "gravity_friction", "impedance")
# Position commands can move, stop in place, or hold when explicitly selecting the mode.
MOVE_POSITION, PAUSE_POSITION, STREAM_POSITION, SDK_SINE, SDK_REPLAY = range(len(CONTROL_MODES), len(CONTROL_MODES) + 5)
NATIVE_MODES = (SDK_SINE, SDK_REPLAY)
WRITE_ZERO = SDK_REPLAY + 1
POINT_POSITION, POINT_PAUSE = WRITE_ZERO + 1, WRITE_ZERO + 2
SDK_DAMPING = POINT_PAUSE + 1
POINT_COMMANDS = (POINT_POSITION, POINT_PAUSE)
PAUSE_COMMANDS = (PAUSE_POSITION, POINT_PAUSE)
ZERO_TOLERANCE = 0.01
ZERO_STILL_SECONDS = 0.3
ZERO_WRITE_TIMEOUT = 30
ZERO_VERIFY_TIMEOUT = 3
ZERO_QUEUED, ZERO_WRITING, ZERO_VERIFYING, ZERO_DONE, ZERO_REJECTED, ZERO_UNVERIFIED = range(1, 7)
POSITION_ACTIONS = (MOVE_POSITION, PAUSE_POSITION, *POINT_COMMANDS)
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


class ControlCommand(NamedTuple):
    """桥接与 SDK 工作进程共用的命令布局；保持现有共享内存格式。

    原生轨迹使用 speed 存放时长、targets[0] 存放帧数。
    return_deadline 非零表示回零后断开，不用于普通位置目标。
    """
    id: float
    mode: float
    speed: float = 0
    gripper_speed: float = 0
    gripper: float = math.nan
    return_deadline: float = 0
    targets: tuple = (0,) * ARM_JOINTS

    def pack(self):
        return [*self[:6], *self.targets]

    @classmethod
    def unpack(cls, values):
        return cls(*values[:6], tuple(values[6:]))


class SharedStateTimeout(RuntimeError):
    pass


class SessionArray:
    def __init__(self, context, values):
        self.values = context.RawArray("d", values)
        self.lock = context.RLock()

    @contextmanager
    def locked(self):
        if not self.lock.acquire(timeout=SHARED_STATE_TIMEOUT):
            raise SharedStateTimeout("Panthera SDK 共享状态读取或写入超时，请重新连接")
        try:
            yield
        finally:
            self.lock.release()

    def __getitem__(self, key):
        with self.locked():
            return self.values[key]

    def __setitem__(self, key, value):
        with self.locked():
            self.values[key] = value


class StopRequest:
    def __init__(self, context):
        # A worker can abort anywhere. Shutdown must never need a lock it held.
        self.requested = context.RawValue("b", False)

    def set(self):
        self.requested.value = True

    def is_set(self):
        return bool(self.requested.value)

    def wait(self, seconds):
        if not self.is_set():
            time.sleep(seconds)
        return self.is_set()


class RobotSession:
    def __init__(self, context, timed_control_started=None):
        self.id = uuid.uuid4().hex
        # Device configuration outlives an SDK worker. The bridge shares this across reconnects.
        self.timed_control_started = (timed_control_started if timed_control_started is not None
                                      else context.RawValue("b", False))
        self.stop = StopRequest(context)
        self.returned = context.RawValue("b", False)
        self.feedback = SessionArray(context, 7 + 6 * MOTOR_SLOTS)  # final field: observed position-mode axis mask
        self.limits = SessionArray(context, 2 * ARM_JOINTS + 3)
        self.command = SessionArray(context, ControlCommand(0, POSITION).pack())
        self.trajectory = context.RawArray("d", MAX_NATIVE_FRAMES * NATIVE_WIDTH)
        self.last_status_request = context.Value("d", 0, lock=False)
        self.zero = SessionArray(context, [0, 0, *([math.nan] * MOTOR_SLOTS)])


    @contextmanager
    def dispatch(self, observed_id):
        # While impedance waits for stillness, the old controller intentionally owns
        # the output. Its permission ends when the observed request is replaced.
        with self.command.locked():
            yield self.command[0] == observed_id and not self.stop.is_set()


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


def enable_control_watchdog(robot, session):
    # Mark before writing: a disconnected SDK call may not report whether the device received it.
    session.timed_control_started.value = True
    value = control_watchdog_ms(robot)
    robot.set_timeout(value)
    return value


def control_watchdog_ms(robot):
    timeout = robot.motor_timeout_ms
    return min(timeout, CONTROL_WATCHDOG_MS) if timeout > 0 else CONTROL_WATCHDOG_MS


def send_position_target(robot, arm, speeds, gripper, gripper_speed):
    """Stage arm and optional gripper in one SDK position frame."""
    has_gripper = math.isfinite(gripper)
    if arm is not None:
        if has_gripper:
            robot.Motors[robot.gripper_id - 1].pos_vel_MAXtqe(
                gripper, gripper_speed, GRIPPER_MAX_TORQUE
            )
        if not robot.Joint_Pos_Vel(arm, speeds, iswait=False):
            raise RuntimeError("SDK 拒绝了关节目标")
    elif has_gripper and not robot.gripper_control(gripper, gripper_speed, GRIPPER_MAX_TORQUE):
        raise RuntimeError("SDK 拒绝了夹爪目标")


def send_sdk_target(robot, mode, target, measured):
    positions, velocities, gripper, gripper_velocity, _ = target
    if mode == SDK_SINE:
        # The sine step inherits the gripper target reached during approach.
        if not robot.Joint_Pos_Vel(positions, velocities, iswait=False):
            raise RuntimeError("SDK 拒绝了正弦轨迹目标")
        return {"axes": ARM_AXES, "targets": list(positions), "velocities": list(velocities), "policy": "stream"}

    gravity = robot.get_Gravity(measured[:ARM_JOINTS])
    if len(gravity) != ARM_JOINTS or not all(math.isfinite(value) for value in gravity):
        raise RuntimeError("动力学计算返回了无效重力力矩")
    torques = [max(-limit, min(limit, value)) for value, limit in
               zip(gravity, map(min, SDK_REPLAY_TORQUE, robot.max_torque))]
    if robot.has_gripper:
        if math.isfinite(gripper):
            gripper_command = (gripper, gripper_velocity, 0, 5.0, 0.5)
        else:
            gripper_command = (0, 0, 0, 0, 0)
        if not robot.gripper_control_MIT(*gripper_command, send=False):
            raise RuntimeError("SDK 拒绝了夹爪回放目标")
    if not robot.pos_vel_tqe_kp_kd(positions, velocities, torques, SDK_REPLAY_KP, SDK_REPLAY_KD):
        raise RuntimeError("SDK 拒绝了关节回放目标")
    details = {"axes": ALL_AXES if robot.has_gripper else ARM_AXES, "targets": list(positions),
               "velocities": list(velocities), "torques": torques, "policy": "stream"}
    if robot.has_gripper:
        details["gripper_mit"] = gripper_command
    return details


ARM_AXES = tuple(f"J{i + 1}" for i in range(ARM_JOINTS))
ALL_AXES = (*ARM_AXES, "夹爪")


def trace_position(trace, command_id, kind, arm, gripper, policy):
    axes = list(ARM_AXES) if arm is not None else []
    targets = list(arm) if arm is not None else []
    if math.isfinite(gripper):
        axes.append("夹爪")
        targets.append(gripper)
    if axes:  # no-op calls are not dispatches
        trace.write(command_id, kind, {"axes": axes, "targets": targets, "policy": policy})


def external_serial_owners(ports):
    # fuser also matches aliases of the same character device.
    try:
        result = subprocess.run(["fuser", *map(str, ports)], capture_output=True,
                                text=True, timeout=2, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError("无法检查机械臂串口占用，请确认系统的 fuser 可用") from exc
    if result.returncode == 1 and not result.stdout.strip() and not result.stderr.strip():
        return []
    owners = result.stdout.split()
    if result.returncode != 0 or not owners or not all(pid.isdecimal() for pid in owners):
        raise RuntimeError("无法确认机械臂串口占用状态")
    return sorted({int(pid) for pid in owners} - {os.getpid()})


position_log = logging.getLogger("hthost.position")


class PositionFeedbackLog:
    """仅观察已发送的位置指令；首次完整反馈的耗时包含轮询延迟。"""

    def __init__(self, session_id):
        self.session_id = session_id
        self.command_id = None
        self.seen = set()

    def position_mask(self, command_id):
        if command_id != self.command_id:
            return 0
        return sum(1 << i for i, event in self.seen if event == "首次10")

    def observe(self, command_id, elapsed, arm, gripper, positions, velocities, modes):
        if self.command_id != command_id:
            self.command_id = command_id
            self.seen.clear()
        goals = (*(arm if arm is not None else (math.nan,) * ARM_JOINTS), gripper)
        changes = []
        for i, (goal, mode) in enumerate(zip(goals, modes)):
            if not math.isfinite(goal):
                continue
            if mode == MOTOR_POSITION_MODE:
                event = "首次10"
            elif mode == 11:
                event = "10后见11" if (i, "首次10") in self.seen else "见11，尚未见10"
            else:
                continue
            if (i, event) not in self.seen:
                self.seen.add((i, event))
                changes.append(f"{motor_name(i)} {event} 偏差={positions[i]-goal:+.6g}rad "
                               f"速度={velocities[i]:+.6g}rad/s")
        if changes:
            position_log.info("会话=%s 位置指令=%d +%.0fms %s", self.session_id, command_id,
                              elapsed * 1000, "；".join(changes))


def run_sdk_worker(session, errors):
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter('%(asctime)s pid=%(process)d %(message)s'))
    for logger in (position_log, logging.getLogger('hthost.trace')):
        logger.setLevel(logging.INFO)
        logger.addHandler(handler)
        logger.propagate = False
    monitor_robot(session, errors, exit_worker=os._exit)


def monitor_robot(session, errors, *, exit_worker=None):
    stop = session.stop
    trace = ControlTrace(session.id)
    loss_reported_for = None
    try:
        ports = sorted(port for port in Path("/dev").glob("ttyACM*") if port.is_char_device())
        if not ports:
            raise RuntimeError("未找到机械臂 USB 控制板")
        owners = external_serial_owners(ports)
        if owners:
            raise RuntimeError("机械臂串口已被其他程序占用，请退出 SDK 脚本后重连（PID "
                               + "、".join(map(str, owners)) + "）")

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
        feedback_log = PositionFeedbackLog(session.id)
        position_mode_seen = set()
        last_point_write = 0
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
        timed_configured = False
        active_mode = POSITION
        native_mode = None
        native_index = 0
        native_started = 0
        still_since = None
        zero_verify_deadline = 0
        zero_samples = 0
        position_tolerance = float(robot.config.get("control", {}).get("position_tolerance", 0.01))
        last_complete = time.monotonic()
        parent = multiprocessing.parent_process()
        while not stop.is_set() and parent.is_alive():
            robot.send_get_motor_state_cmd()
            robot.motor_send_cmd()
            interval = (SDK_SINE_INTERVAL if native_mode == SDK_SINE else
                        SDK_REPLAY_INTERVAL if native_mode == SDK_REPLAY else
                        TORQUE_INTERVAL if torque_control else CONTROL_INTERVAL)
            if stop.wait(interval):
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
                trace.sample(command_id, positions[:len(motors)], velocities[:len(motors)],
                             motor_modes[:len(motors)], motor_faults[:len(motors)])
                if command_id and command_id == applied_command_id and active_mode in (POSITION, *POSITION_ACTIONS):
                    feedback_log.observe(command_id, now - applied_at, target, gripper_target,
                                         positions, velocities, motor_modes)
                    # Refresh/homing failures are raised below. Single dispatch has no such exception.
                    if (loss_reported_for != command_id and not return_deadline and
                            (active_mode not in (POSITION, *POINT_COMMANDS) or not session.timed_control_started.value)):
                        mask = feedback_log.position_mask(command_id)
                        lost = [motor_name(i) for i, mode in enumerate(motor_modes)
                                if mask & (1 << i) and mode != MOTOR_POSITION_MODE]
                        goals = (*(target if target is not None else (math.nan,) * ARM_JOINTS), gripper_target)
                        missing = [motor_name(i) for i, goal in enumerate(goals)
                                   if math.isfinite(goal) and motor_modes[i] != MOTOR_POSITION_MODE]
                        if lost or (missing and now - applied_at >= POSITION_RESPONSE_SECONDS):
                            trace.failure(("位置模式丢失：" if lost else "未确认位置模式：") + "、".join(lost or missing))
                            loss_reported_for = command_id
                still = all(abs(value) <= POSITION_SETTLE_SPEED for value in velocities[:len(motors)])
                still_since = (now if still_since is None else still_since) if still else None
                session.zero[1] = still_since if still_since is not None else 0
                session.feedback[:] = [
                    now, confirmed_mode, rejected_mode_id, applied_command_id, applied_at, completed_position_id,
                    *positions, *velocities, *torques, *applied_targets, *motor_modes, *motor_faults,
                    feedback_log.position_mask(applied_command_id),
                ]
                pending = {motor.get_motor_id(): motor for motor in motors}
                if zero_verify_deadline:
                    at_zero = [abs(value) <= ZERO_TOLERANCE for value in positions[:len(motors)]]
                    healthy = not any(motor_faults)
                    zero_samples = zero_samples + 1 if all(at_zero) and still and healthy else 0
                    phase = ZERO_VERIFYING
                    if zero_samples >= 3:
                        phase = ZERO_DONE
                    elif not healthy or now >= zero_verify_deadline:
                        phase = ZERO_UNVERIFIED
                    session.zero[:] = [phase, session.zero[1], *positions]
                    if phase == ZERO_UNVERIFIED:
                        trace.failure("零位写入后读回未确认")
                    if phase != ZERO_VERIFYING:
                        zero_verify_deadline = 0
                    continue
                next_id, mode, speed, next_gripper_speed, next_gripper, deadline, next_target = ControlCommand.unpack(session.command[:])
                accept_command = next_id != command_id and next_id != rejected_mode_id
                if mode == SDK_DAMPING:
                    if accept_command:
                        with session.dispatch(next_id) as current:
                            if not current:
                                continue
                            if robot.set_brake() is False:
                                raise RuntimeError("SDK 阻尼指令发送失败")
                        trace.write(next_id, "sdk_damping", {"axes": ALL_AXES[:len(motors)], "policy": "once"})
                        native_mode = torque_control = target = None
                        gripper_target = math.nan
                        tracking_position = False
                        applied_targets = [math.nan] * MOTOR_SLOTS
                        active_mode = confirmed_mode = SDK_DAMPING
                        command_id = applied_command_id = next_id
                        applied_at = now
                    continue
                if accept_command and mode == WRITE_ZERO:
                    command_id = next_id
                    if (confirmed_mode != POSITION or torque_control or native_mode or any(motor_faults)
                            or still_since is None or now - still_since < ZERO_STILL_SECONDS
                            or any(abs(actual - expected) > ZERO_TOLERANCE
                                   for actual, expected in zip(positions[:ARM_JOINTS], next_target))
                            or (robot.gripper_installed and abs(positions[-1] - next_gripper) > ZERO_TOLERANCE)):
                        session.zero[0] = ZERO_REJECTED
                        continue
                    if stop.is_set() or now - session.last_status_request.value > CONTROL_LEASE:
                        session.zero[0] = ZERO_REJECTED
                        continue
                    target = None
                    gripper_target = math.nan
                    applied_targets = [math.nan] * MOTOR_SLOTS
                    tracking_position = False
                    with session.dispatch(next_id) as current:
                        if not current:
                            continue
                        session.zero[0] = ZERO_WRITING
                        robot.set_reset_zero()
                        robot.motor_send_cmd()
                    trace.write(command_id, "write_zero", {"axes": ALL_AXES[:len(motors)], "policy": "once"})
                    # A pre-write packet cannot verify a new zero.
                    timestamps = {motor_id: motor.get_current_motor_state().time
                                  for motor_id, motor in pending.items()}
                    last_complete = time.monotonic()
                    zero_verify_deadline = last_complete + ZERO_VERIFY_TIMEOUT
                    zero_samples = 0
                    still_since = None
                    session.zero[:] = [ZERO_VERIFYING, 0, *([math.nan] * MOTOR_SLOTS)]
                    continue
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
                        trace.failure(f"指令 {int(next_id)}：进入阻抗前未能停稳")
                        rejected_mode_id = next_id

                if accept_command:
                    command_id = next_id
                    position_mode_seen.clear()
                    active_mode = mode
                    native_mode = mode if mode in NATIVE_MODES else None
                    return_deadline = deadline
                    settled_samples = 0
                    tracking_position = mode in POSITION_ACTIONS
                    position_settled_since = None
                    if native_mode:
                        native_index = 0
                        native_started = now
                        target = None
                        gripper_target = math.nan
                    elif mode in (*PAUSE_COMMANDS, POSITION):
                        target = [
                            max(lower, min(upper, position))
                            for position, lower, upper in zip(
                                positions, robot.joint_limits["lower"], robot.joint_limits["upper"]
                            )
                        ] if mode != POINT_PAUSE or speed > 0 else None
                        hold_speed = min(RETURN_SPEED, min(robot.velocity_limits))
                        speeds = [hold_speed] * ARM_JOINTS if target is not None else None
                        gripper_target = (
                            max(gripper_limits[0], min(gripper_limits[1], positions[-1]))
                            if math.isfinite(gripper_limits[0]) and (mode != POINT_PAUSE or next_gripper_speed > 0)
                            else math.nan
                        )
                        gripper_speed = min(hold_speed, GRIPPER_MAX_SPEED)
                    elif mode == POINT_POSITION:
                        torque_control = None
                        target = next_target if speed else None
                        speeds = [speed] * ARM_JOINTS if speed else None
                        gripper_target = next_gripper if next_gripper_speed else math.nan
                        gripper_speed = next_gripper_speed
                    elif mode not in (POSITION, MOVE_POSITION, STREAM_POSITION):
                        torque_control = TorqueControl(robot, mode, positions, now, torque_control)
                        target = None
                        gripper_target = math.nan
                    else:
                        # Streaming/action commands carry their own goals; never reuse a prior target.
                        target = next_target if speed else None
                        speeds = [speed] * ARM_JOINTS if speed else None
                        gripper_target = next_gripper if next_gripper_speed else math.nan
                        gripper_speed = next_gripper_speed
                    if mode in (POSITION, *POINT_COMMANDS, MOVE_POSITION, PAUSE_POSITION, STREAM_POSITION):
                        torque_control = None
                    elif native_mode:
                        torque_control = None

                if (
                    tracking_position
                    and applied_command_id == command_id
                    and completed_position_id != command_id
                ):
                    stopped = all(abs(value) <= POSITION_SETTLE_SPEED for value in velocities[:len(motors)])
                    joint_goals = target if target is not None else [math.nan] * ARM_JOINTS
                    reached = all(
                        not math.isfinite(goal) or (
                            drive_mode == MOTOR_POSITION_MODE and not fault
                            and abs(position - goal) <= POSITION_TOLERANCE
                        )
                        for position, goal, drive_mode, fault in zip(
                            positions, (*joint_goals, gripper_target), motor_modes, motor_faults
                        )
                    )
                    if not stopped or not reached:
                        position_settled_since = None
                    elif position_settled_since is None:
                        position_settled_since = now
                    elif now - position_settled_since >= POSITION_SETTLE_SECONDS:
                        completed_position_id = command_id

                arm_command = target
                gripper_command = gripper_target
                has_gripper_target = math.isfinite(gripper_command)

                if active_mode in (POSITION, *POINT_COMMANDS) and command_id > 0:
                    if stop.is_set():
                        continue
                    fault_message = describe_motor_faults(motor_faults)
                    if fault_message:
                        raise RuntimeError(fault_message)
                    if applied_command_id == command_id:
                        if not return_deadline and not session.timed_control_started.value:
                            continue
                        controlled = list(range(ARM_JOINTS)) if arm_command is not None else []
                        if has_gripper_target:
                            controlled.append(ARM_JOINTS)
                        missing = [i for i in controlled if motor_modes[i] != MOTOR_POSITION_MODE]
                        lost = position_mode_seen.intersection(missing)
                        position_mode_seen.update(i for i in controlled if i not in missing)
                        if lost or (missing and now - applied_at >= POSITION_RESPONSE_SECONDS):
                            axes = "、".join(f"{motor_name(i)}（模式 {int(motor_modes[i])}）" for i in missing)
                            raise RuntimeError("未能保持位置控制，已停止刷新：" + axes)
                        if missing or not controlled:
                            continue
                        if return_deadline:
                            at_zero = all(abs(positions[i]) <= position_tolerance
                                          and abs(velocities[i]) <= RETURN_SETTLE_SPEED for i in controlled)
                            settled_samples = settled_samples + 1 if at_zero else 0
                            if settled_samples >= RETURN_SETTLE_SAMPLES:
                                session.returned.value = True
                                break
                    if not return_deadline and now - session.last_status_request.value > CONTROL_LEASE:
                        raise RuntimeError("控制页面已失联，位置控制已结束")
                    with session.dispatch(next_id) as current:
                        if not current:
                            continue
                        write_at = time.monotonic()
                        if (applied_command_id == command_id and session.timed_control_started.value
                                and write_at - last_point_write >= control_watchdog_ms(robot) / 1000):
                            raise RuntimeError("位置刷新间隔已超过设备超时，未重新发送目标")
                        send_position_target(robot, arm_command, speeds, gripper_command, gripper_speed)
                        last_point_write = write_at
                        if arm_command is not None:
                            applied_targets[:ARM_JOINTS] = arm_command
                        if has_gripper_target:
                            applied_targets[-1] = gripper_command
                    kind = ("home" if return_deadline else "mode_hold" if active_mode == POSITION else
                            "position_pause" if active_mode == POINT_PAUSE else "position")
                    policy = "home_refresh" if return_deadline else "watchdog_refresh" if session.timed_control_started.value else "once"
                    trace_position(trace, command_id, kind, arm_command, gripper_command, policy)
                    confirmed_mode = POSITION
                    if applied_command_id != command_id:
                        applied_command_id = command_id
                        applied_at = now
                    continue

                if (native_mode or torque_control or arm_command is not None or has_gripper_target) and not stop.is_set():
                    fault_message = describe_motor_faults(motor_faults)
                    if fault_message:
                        raise RuntimeError(fault_message)

                if native_mode and not stop.is_set():
                    if now - session.last_status_request.value > CONTROL_LEASE:
                        raise RuntimeError("控制页面已失联，SDK 轨迹已结束")
                    with session.dispatch(next_id) as current:
                        if not current:
                            continue
                        frame = native_target(native_mode == SDK_SINE, session.trajectory,
                                              int(next_target[0]), speed, now - native_started, native_index)
                        details = send_sdk_target(robot, native_mode, frame, positions)
                        trace.write(command_id, "sdk_sine" if native_mode == SDK_SINE else "sdk_replay", details)
                        if not timed_configured:
                            timeout_ms = enable_control_watchdog(robot, session)
                            trace.write(command_id, "watchdog", {"axes": ALL_AXES[:len(motors)], "timeout_ms": timeout_ms, "policy": "configure"})
                            timed_configured = True
                    native_index = frame[-1]
                    applied_targets[:ARM_JOINTS] = frame[0]
                    applied_targets[-1] = frame[2]
                    if applied_command_id != command_id:
                        applied_command_id = command_id
                        applied_at = now
                    continue

                if torque_control and not stop.is_set():
                    if now - session.last_status_request.value > CONTROL_LEASE:
                        raise RuntimeError("控制页面已失联，力矩控制已结束")
                    with session.dispatch(next_id) as current:
                        if not current:
                            continue
                        if torque_control.send(positions, velocities, now):
                            confirmed_mode = torque_control.mode
                    details = {"axes": ALL_AXES if robot.has_gripper else ARM_AXES, "policy": "stream",
                               "torques": torque_control.torque, "joint_mit_position": torque_control.zero}
                    if robot.has_gripper:
                        details["gripper_mit"] = (torque_control.gripper_position if torque_control.gripper_gain else 0.,
                                                  0., 0., .65 * torque_control.gripper_gain, .06 * torque_control.gripper_gain)
                    trace.write(command_id, CONTROL_MODES[int(torque_control.mode)], details)
                    # Match the official compensation/impedance scripts: do not
                    # configure a persistent watchdog that also affects later position goals.
                    applied_targets[:ARM_JOINTS] = torque_control.target or [math.nan] * ARM_JOINTS
                    applied_targets[-1] = (
                        torque_control.gripper_position
                        if torque_control.mode == IMPEDANCE and robot.has_gripper else math.nan
                    )
                    continue

                if (arm_command is not None or has_gripper_target) and not stop.is_set():
                    if now - session.last_status_request.value > CONTROL_LEASE:
                        raise RuntimeError("控制页面已失联，位置控制已结束")
                    # Serialize dispatch with the pause request, so a replaced target cannot be sent later.
                    with session.dispatch(next_id) as current:
                        if not current:
                            continue
                        send_position_target(robot, arm_command, speeds, gripper_command, gripper_speed)
                        trace_position(trace, command_id, "action_pause" if active_mode == PAUSE_POSITION else
                                       "action_position" if active_mode == MOVE_POSITION else "stream_position",
                                       arm_command, gripper_command, "stream")
                        if arm_command is not None:
                            applied_targets[:ARM_JOINTS] = arm_command
                        if not timed_configured:
                            timeout_ms = enable_control_watchdog(robot, session)
                            trace.write(command_id, "watchdog", {"axes": ALL_AXES[:len(motors)], "timeout_ms": timeout_ms, "policy": "configure"})
                            timed_configured = True
                    applied_targets[-1] = gripper_command
                    confirmed_mode = POSITION
                    if applied_command_id != command_id and command_id != rejected_mode_id:
                        applied_command_id = command_id
                        applied_at = now
                elif stop.wait(0.06):
                    break
            elif zero_verify_deadline and now >= zero_verify_deadline:
                trace.failure("零位读回未收到完整反馈")
                session.zero[0] = ZERO_UNVERIFIED
                zero_verify_deadline = 0
            elif zero_verify_deadline:
                continue
            elif now - last_complete > (TORQUE_FEEDBACK_TIMEOUT if torque_control or native_mode else FEEDBACK_TIMEOUT):
                missing = "、".join(motor.get_motor_name() for motor in pending.values())
                raise RuntimeError(f"未收到电机反馈（{missing}）")
    except KeyboardInterrupt:
        trace.failure("工作进程收到 KeyboardInterrupt")
    except (Exception, SystemExit) as exc:
        trace.failure(str(exc))
        print(f"Panthera: {exc}", file=sys.stderr, flush=True)
        message = str(exc) if isinstance(exc, RuntimeError) else "Panthera SDK 初始化或读取失败"
        # Keep the notification within a single atomic pipe write on Linux.
        errors.send(message[:512])
    finally:
        errors.close()
        if exit_worker is not None:
            # Never let SDK destruction send brake/reset commands on a reappearing USB link.
            # After USB loss, behavior depends on firmware and any existing device timeout.
            exit_worker(0)


def describe_motor_faults(faults):
    faults = [f"{motor_name(i)}（故障码 {int(code)}）" for i, code in enumerate(faults) if code]
    return f"电机报告故障：{'、'.join(faults)}" if faults else None


def motor_name(index):
    return f"J{index + 1}" if index < ARM_JOINTS else "夹爪"
