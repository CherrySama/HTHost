"""离线检查桥接重连和位置指令顺序，不导入原生 SDK、不打开串口。

运行 python -m unittest discover -s tests -p test_position_recovery.py -v
记录的是 SDK 调用及模拟协议操作，不是 USB/CAN 抓包。
不能证明电机固件已经清除旧目标，不能作为解锁真机的依据。
"""

import ast
import json
import math
import multiprocessing
import os
import subprocess
import sys
import time
import tempfile
import unittest
from contextlib import ExitStack
from functools import partial
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from pathlib import Path
from threading import Thread
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import panthera_bridge as bridge
from scripts import panthera_control as control


def sdk_methods():
    """只编译待检查的方法，绝不执行 SDK 导入、构造及串口初始化。"""
    path = control.SDK / "scripts/Panthera_lib/Panthera.py"
    tree = ast.parse(path.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
               and node.name == "Panthera")
    names = {"Joint_Pos_Vel", "gripper_control"}
    methods = [node for node in cls.body if isinstance(node, ast.FunctionDef)
               and node.name in names]
    assert {node.name for node in methods} == names
    # 测试目标均为合法角度。限位校验关闭，只保留待检查的发送路径。
    namespace = {"np": SimpleNamespace(asarray=list)}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"), namespace)
    return {name: namespace[name] for name in names}


METHODS = sdk_methods()


class PersistentDevice:
    """仅模拟超时与速度限制；模式 11 为测试约定，不是固件定义。

    对象跨 SDK 会话保留。没有模拟通信板内部轨迹或初始化副作用。
    """
    def __init__(self, position=0.4):
        self.now = 100.0
        self.positions = [position] * 7
        self.velocities = [0.0] * 7
        self.goals = [None] * 7
        self.timeout_ms = 0
        self.last_control = self.now
        self.mode = 11
        self.expired = False
        self.ignore_clear = False
        self.ignore_position = False

    def advance(self, now):
        dt = now - self.now
        self.now = now
        if self.timeout_ms and now - self.last_control > self.timeout_ms / 1000:
            self.mode = 11
            self.expired = True
        self.velocities = [0.0] * 7
        if self.mode == 10:
            for i, goal in enumerate(self.goals):
                if goal is not None:
                    target, speed, _ = goal
                    distance = max(-speed * dt, min(speed * dt, target - self.positions[i]))
                    self.positions[i] += distance
                    self.velocities[i] = distance / dt if dt else 0

    def receive(self, now, opcode, payload):
        self.advance(now)
        if opcode == 0x85:
            if payload[0] or not self.ignore_clear:
                self.timeout_ms = payload[0]
        elif opcode == 0x90:
            if self.ignore_position:
                return
            self.goals = [new if new is not None else old for new, old in zip(payload, self.goals)]
            self.mode = 10
            self.last_control = now
        elif opcode == "mit":
            self.mode = 1
            self.last_control = now


class RecorderRobot:
    def __init__(self, clock, measured, device=None):
        self.clock = clock
        self.device = device
        self.calls = []
        self.buffer = None
        self.stamp = 1
        self.freeze = False
        self.measured = measured
        self.gripper_installed = self.has_gripper = True
        self.gripper_id = 7
        self.motor_count = 6
        self.gripper_limits = {"lower": 0, "upper": 2}
        self.joint_limits = {"lower": [-2.7] * 6, "upper": [3.14] * 6}
        self.velocity_limits = [1] * 6
        self.max_torque = [21, 36, 36, 21, 10, 10]
        self.motor_timeout_ms = 0
        self.config = {}
        self.Motors = [self.make_motor(i) for i in range(7)]

    def make_motor(self, i):
        return SimpleNamespace(
            get_motor_id=lambda: i + 1,
            get_motor_name=lambda: f"J{i + 1}",
            get_current_motor_state=lambda: SimpleNamespace(
                ID=i + 1, time=self.stamp, position=self.device.positions[i] if self.device else self.measured[i],
                velocity=self.device.velocities[i] if self.device else 0, torque=0,
                mode=self.device.mode if self.device else 11, fault=0),
            pos_vel_MAXtqe=lambda p, v, t: self.stage(i, p, v, t),
        )

    def stage(self, i, position, speed, torque):
        if self.buffer is None or self.buffer[0] != 0x90:
            self.buffer = (0x90, [None] * 7)
        self.buffer[1][i] = (float(position), float(speed), float(torque))

    def motor_send_cmd(self):
        opcode, payload = self.buffer
        self.calls.append((round(self.clock.now, 4), opcode, tuple(payload)))
        if self.device:
            self.device.receive(self.clock.now, opcode, payload)

    def send_get_motor_state_cmd(self):
        if not self.freeze:
            self.stamp += 1
        self.buffer = (0x06, [0x7f])
        self.motor_send_cmd()

    def set_timeout(self, milliseconds):
        # 模拟 C++ robot::set_timeout 的五次写入，不能代表固件如何响应。
        for _ in range(5):
            self.buffer = (0x85, [milliseconds] * 7)
            self.motor_send_cmd()
            self.clock.now += 0.01

    def get_Gravity(self, positions):
        return [0.5] * 6

    def get_friction_compensation(self, *args):
        return [0.0] * 6

    def gripper_control_MIT(self, *args, **kwargs):
        return True

    def pos_vel_tqe_kp_kd(self, *args):
        self.buffer = ("mit", list(args[2]))
        self.motor_send_cmd()
        return True

    def Joint_Pos_Vel(self, *args, **kwargs):
        limits = self.joint_limits
        try:
            self.joint_limits = None
            return METHODS["Joint_Pos_Vel"](self, *args, **kwargs)
        finally:
            self.joint_limits = limits

    gripper_control = METHODS["gripper_control"]


def run_bridge(measured, *, commands=None, ticks=15, freeze_after=None, exit_worker=None, page_alive=True, device=None, page_until=None, timed_control_started=None):
    clock = SimpleNamespace(now=device.now if device else 100.0)
    session = control.RobotSession(multiprocessing.get_context("spawn"), timed_control_started)
    robot = RecorderRobot(clock, measured, device)
    errors = []
    counter = 0

    def wait(seconds):
        nonlocal counter
        counter += 1
        clock.now += seconds
        if device:
            device.advance(clock.now)
        if page_alive and (page_until is None or counter < page_until):
            session.last_status_request.value = clock.now
        if commands and counter in commands:
            session.command[:] = commands[counter]
        if freeze_after is not None and counter >= freeze_after:
            if not robot.freeze:
                robot.feedback_lost_at = clock.now
                robot.feedback_lost_call_index = len(robot.calls)
            robot.freeze = True
        return counter > ticks

    session.stop = SimpleNamespace(is_set=lambda: False, wait=wait)
    package = ModuleType("Panthera_lib")
    package.__path__ = []
    module = ModuleType("Panthera_lib.Panthera")
    module.Panthera = lambda _: robot
    port = SimpleNamespace(is_char_device=lambda: True)
    with ExitStack() as stack:
        stack.enter_context(patch.dict(sys.modules, {
            "Panthera_lib": package, "Panthera_lib.Panthera": module,
        }))
        stack.enter_context(patch.object(sys, "path", list(sys.path)))
        stack.enter_context(patch.object(control.Path, "glob", return_value=[port]))
        stack.enter_context(patch.object(control, "external_serial_owners", return_value=[]))
        stack.enter_context(patch.object(control.multiprocessing, "parent_process",
                                       return_value=SimpleNamespace(is_alive=lambda: True)))
        stack.enter_context(patch.object(control, "time", SimpleNamespace(monotonic=lambda: clock.now)))
        control.monitor_robot(session, SimpleNamespace(send=errors.append, close=lambda: None), exit_worker=exit_worker)
    return robot, session, errors


def publish_feedback(session, position):
    slots = control.MOTOR_SLOTS
    session.limits[:] = [-2.7] * 6 + [3.14] * 6 + [1, 0, 2]
    session.feedback[:] = [
        time.monotonic(), control.POSITION, 0, 0, 0, 0,
        *([position] * slots), *([0] * (2 * slots)),
        *([math.nan] * slots), *([11] * slots), *([0] * slots),
    ]


def failing_worker(session, errors, *, trigger, lock_held, field, crash):
    publish_feedback(session, 0.6)
    if trigger.wait(6):
        with getattr(session, field).locked():
            lock_held.set()
            if crash:
                os._exit(23)
            time.sleep(10)
    errors.close()


def reconnect_worker(session, errors, *, feedback_ready):
    if feedback_ready.wait(6):
        publish_feedback(session, 0.25)
        while not session.stop.wait(0.04):
            pass
    errors.close()


class BridgeReconnectTests(unittest.TestCase):
    def setUp(self):
        self.bridge = RobotConnectionServer()
        self.addCleanup(self.bridge.close)

    def check_reconnect(self, field, *, crash):
        server = self.bridge
        context = server.robot.context
        trigger, lock_held, feedback_ready = (context.Event() for _ in range(3))
        simulate = partial(failing_worker, trigger=trigger, lock_held=lock_held, field=field, crash=crash)
        with patch.object(bridge, "run_sdk_worker", simulate):
            code, _ = server.request("POST")
        self.assertEqual(code, 200)
        old = server.wait_for_connection()
        self.assertEqual(old["positions"][3], 0.6)
        server.robot.session.timed_control_started.value = True

        worker = server.robot.worker
        trigger.set()
        self.assertTrue(lock_held.wait(3))
        if crash:
            worker.join(timeout=2)
            self.assertEqual(worker.exitcode, 23)

        code, error = server.request("GET")
        self.assertEqual(code, 200)
        self.assertEqual(error["state"], "error")
        self.assertIn("进程已退出" if crash else "共享状态", error["message"])
        self.assertIsNone(server.robot.worker)
        self.assertIsNone(server.robot.session)

        with patch.object(bridge, "run_sdk_worker", partial(reconnect_worker, feedback_ready=feedback_ready)):
            code, status = server.request("POST")
        self.assertEqual((code, status), (200, {"state": "connecting"}))
        feedback_ready.set()
        new = server.wait_for_connection()
        self.assertNotEqual(new["session"], old["session"])
        self.assertTrue(server.robot.session.timed_control_started.value)
        self.assertEqual(new["positions"][3], 0.25)
        self.assertEqual(new["appliedCommand"], 0)
        self.assertTrue(all(motor["target"] is None for motor in new["motors"]))

        code, rejected = server.request("POST", bridge.POSITION_ENDPOINT,
                                        {"session": old["session"], "positions": [0.6] * 6, "speed": 0.2})
        self.assertEqual(code, 409)
        self.assertIn("连接已失效", rejected["message"])
        self.assertEqual(server.robot.session.command[:2], [0, control.POSITION])
        self.assertEqual(server.robot.disconnect(), {"state": "disconnected"})
        self.assertIsNone(server.robot.worker)

    def test_reconnect_after_worker_exits_holding_feedback_lock(self):
        self.check_reconnect("feedback", crash=True)

    def test_reconnect_after_worker_hangs_holding_command_lock(self):
        self.check_reconnect("command", crash=False)

    def test_normal_connection_accepts_new_position(self):
        server = self.bridge
        feedback_ready = server.robot.context.Event()
        feedback_ready.set()
        with patch.object(bridge, "run_sdk_worker", partial(reconnect_worker, feedback_ready=feedback_ready)):
            self.assertEqual(server.request("POST")[0], 200)
        connected = server.wait_for_connection()
        request = {"session": connected["session"], "positions": [0.4] * 6, "speed": 0.2}
        self.assertEqual(server.request("POST", bridge.POSITION_ENDPOINT, request)[0], 202)
        self.assertEqual(server.robot.session.command[1], control.POINT_POSITION)

    def test_position_request_fails_when_command_lock_hangs_after_status_check(self):
        server = self.bridge
        context = server.robot.context
        trigger, lock_held = context.Event(), context.Event()
        simulate = partial(failing_worker, trigger=trigger, lock_held=lock_held, field="command", crash=False)
        with patch.object(bridge, "run_sdk_worker", simulate):
            self.assertEqual(server.request("POST")[0], 200)
        session = server.wait_for_connection()["session"]
        read_status = server.robot._status

        def hang_after_status_check():
            status = read_status()
            trigger.set()
            self.assertTrue(lock_held.wait(2))
            return status

        with patch.object(server.robot, "_status", hang_after_status_check):
            code, result = server.request("POST", bridge.POSITION_ENDPOINT,
                                          {"session": session, "positions": [0.25] * 6, "speed": 0.2})
        self.assertEqual(code, 409)
        self.assertIn("共享状态", result["message"])
        self.assertEqual(server.request("GET")[1]["state"], "error")
        self.assertIsNone(server.robot.worker)


class RobotConnectionServer:
    def __init__(self):
        self.robot = bridge.RobotConnection()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), bridge.Handler)
        self.server.robot = self.robot
        self.thread = Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def request(self, method, path=bridge.ENDPOINT, payload=None):
        connection = HTTPConnection(*self.server.server_address, timeout=4)
        try:
            connection.request(method, path, json.dumps(payload) if payload is not None else None,
                               {"Content-Type": "application/json"})
            response = connection.getresponse()
            return response.status, json.load(response)
        finally:
            connection.close()

    def wait_for_connection(self):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            _, status = self.request("GET")
            if status["state"] != "connecting":
                if status["state"] != "connected":
                    raise AssertionError(status)
                return status
            time.sleep(0.02)
        raise AssertionError("模拟子进程没有发布关节反馈")

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)
        worker = self.robot.worker
        if worker is not None and worker.is_alive():
            worker.kill()
            worker.join(timeout=1)


class ModeTransitionTests(unittest.TestCase):
    def test_all_modes_return_after_page_close_without_clearing_watchdog(self):
        for mode in (control.POSITION, control.GRAVITY, control.GRAVITY_FRICTION, control.IMPEDANCE):
            with self.subTest(mode=mode):
                device = PersistentDevice()
                commands = {
                    2: [1, mode, 0, 0, math.nan, 0, *([0] * 6)],
                    200: [2, control.POINT_POSITION, 0.2, 0.2, 0, 120, *([0] * 6)],
                }
                robot, session, errors = run_bridge([0.4] * 7, device=device,
                    commands=commands, ticks=500, page_until=200)
                self.assertEqual(errors, [])
                self.assertTrue(session.returned.value)
                self.assertFalse(device.expired)
                if mode != control.POSITION:
                    self.assertTrue(any(c[1] == "mit" for c in robot.calls))
                self.assertFalse(any(c[1] == 0x85 and c[2][0] == 0 for c in robot.calls))
                self.assertEqual(device.timeout_ms, 0)
                homes = [c for c in robot.calls if c[1] == 0x90 and c[2][0][0] == 0]
                self.assertGreater(len(homes), 3)
                self.assertTrue(all(abs(p) <= control.POSITION_TOLERANCE for p in device.positions))
                self.assertTrue(all(c[1] == 0x06 for c in robot.calls[robot.calls.index(homes[-1]) + 1:]))

    def test_homing_stops_dispatch_when_feedback_is_lost(self):
        for mode in range(4):
            with self.subTest(mode=mode):
                device = PersistentDevice()
                commands = {2: [1, mode, 0, 0, math.nan, 0, *([0] * 6)],
                            200: [2, control.POINT_POSITION, .2, .2, 0, 120, *([0] * 6)]}
                robot, session, errors = run_bridge([.4] * 7, device=device,
                    commands=commands, freeze_after=205, ticks=400, page_until=200)
                self.assertTrue(any("未收到电机反馈" in error for error in errors))
                self.assertFalse(session.returned.value)
                # The query before simulated unplug has one complete response in flight.
                following = robot.calls[robot.feedback_lost_call_index:]
                self.assertLessEqual(sum(c[1] == 0x90 for c in following), 1)
                self.assertTrue(all(c[1] == 0x06 for c in following
                                    if c[0] > robot.feedback_lost_at + control.CONTROL_INTERVAL))
                self.assertFalse(any(c[1] == 0x85 and c[2][0] == 0 for c in robot.calls))

    def test_reconnect_reads_feedback_without_replaying_old_commands(self):
        for mode in (control.POSITION, control.GRAVITY, control.GRAVITY_FRICTION, control.IMPEDANCE):
            with self.subTest(mode=mode):
                device = PersistentDevice()
                shared = multiprocessing.get_context("spawn").RawValue("b", False)
                command = ([1, control.POINT_POSITION, 0.2, 0, math.nan, 0, *([0.8] * 6)]
                           if mode == control.POSITION else
                           [1, mode, 0, 0, math.nan, 0, *([0] * 6)])
                _, _, errors = run_bridge([0.4] * 7, device=device,
                    commands={2: command}, ticks=150, timed_control_started=shared)
                self.assertEqual(errors, [])
                timeout_before = device.timeout_ms
                goals_before = list(device.goals)
                device.advance(device.now + 1)  # USB absent, device remains powered.
                robot, session, errors = run_bridge([0.4] * 7, device=device,
                    timed_control_started=shared)
                self.assertEqual(errors, [])
                self.assertEqual({call[1] for call in robot.calls}, {0x06})
                self.assertEqual(device.timeout_ms, timeout_before)
                self.assertEqual(device.goals, goals_before)
                self.assertFalse(session.timed_control_started.value)



    def test_torque_reconnect_sends_position_once_without_configuring_watchdog(self):
        for mode in (control.GRAVITY, control.GRAVITY_FRICTION, control.IMPEDANCE):
            with self.subTest(mode=mode):
                device = PersistentDevice()
                device.ignore_clear = True  # Reproduce hardware where clearing did not work.
                shared = multiprocessing.get_context("spawn").RawValue("b", False)
                run_bridge([.4] * 7, device=device, timed_control_started=shared, ticks=150,
                    commands={2: [1, mode, 0, 0, math.nan, 0, *([0] * 6)]})
                device.advance(device.now + 1)
                goal = [1, control.POINT_POSITION, .2, .2, .6, 0, *([.6] * 6)]
                robot, session, errors = run_bridge([.4] * 7, device=device, commands={2: goal},
                    ticks=100, timed_control_started=shared)
                self.assertEqual(errors, [])
                self.assertFalse(shared.value)
                self.assertFalse(any(c[1] == 0x85 for c in robot.calls))
                goals = [c for c in robot.calls if c[1] == 0x90]
                self.assertEqual(len(goals), 1)
                self.assertTrue(all(c[2][0][0] == .6 for c in goals))
                self.assertEqual(session.feedback[5], 1)
                self.assertEqual(device.mode, 10)
                self.assertEqual(device.timeout_ms, 0)

    def test_torque_home_reconnect_single_goal_continues_without_host(self):
        for mode in (control.GRAVITY, control.GRAVITY_FRICTION, control.IMPEDANCE):
            with self.subTest(mode=mode):
                device = PersistentDevice()
                shared = multiprocessing.get_context("spawn").RawValue("b", False)
                robot, session, errors = run_bridge([.4] * 7, device=device,
                    timed_control_started=shared, ticks=500, page_until=200,
                    commands={2: [1, mode, 0, 0, math.nan, 0, *([0] * 6)],
                              200: [2, control.POINT_POSITION, .2, .2, 0, 120, *([0] * 6)]})
                self.assertEqual(errors, [])
                self.assertTrue(session.returned.value)
                self.assertFalse(any(c[1] == 0x85 for c in robot.calls))
                robot, _, errors = run_bridge([0] * 7, device=device,
                    timed_control_started=shared, ticks=5,
                    commands={2: [1, control.POINT_POSITION, .2, .2, .6, 0, *([.6] * 6)]})
                self.assertEqual(errors, [])
                self.assertEqual(sum(c[1] == 0x90 for c in robot.calls), 1)
                self.assertTrue(all(p < .6 for p in device.positions))
                # No further host commands. This verifies only the model's no-watchdog case.
                device.advance(device.now + 4)
                self.assertEqual(device.positions, [.6] * 7)
                device.advance(device.now + 2)
                self.assertEqual(device.positions, [.6] * 7)
                self.assertFalse(device.expired)

    def test_preexisting_timeout_is_not_cleared_by_torque_or_position(self):
        device = PersistentDevice()
        device.timeout_ms = 250
        robot, _, errors = run_bridge([.4] * 7, device=device, ticks=220,
            commands={2: [1, control.GRAVITY, 0, 0, math.nan, 0, *([0] * 6)],
                      200: [2, control.POINT_POSITION, .2, .2, .8, 0, *([.8] * 6)]})
        self.assertEqual(errors, [])
        self.assertFalse(any(c[1] == 0x85 for c in robot.calls))
        self.assertEqual(sum(c[1] == 0x90 for c in robot.calls), 1)
        self.assertEqual(device.timeout_ms, 250)
        self.assertEqual(device.mode, 11)
        self.assertTrue(all(p < .8 for p in device.positions))

    def test_position_replacement_and_pause_send_each_goal_once(self):
        device = PersistentDevice()
        shared = multiprocessing.get_context("spawn").RawValue("b", False)
        commands = {2: [1, control.POINT_POSITION, .2, .2, .8, 0, *([.8] * 6)],
                    12: [2, control.POINT_POSITION, .2, .2, .2, 0, *([.2] * 6)],
                    22: [3, control.POINT_PAUSE, 0, 0, math.nan, 0, *([0] * 6)]}
        robot, session, errors = run_bridge([.4] * 7, device=device, commands=commands,
            ticks=100, timed_control_started=shared)
        self.assertEqual(errors, [])
        goals = [c[2][0][0] for c in robot.calls if c[1] == 0x90]
        self.assertEqual(len(goals), 3)
        self.assertIn(.8, goals)
        self.assertIn(.2, goals)
        self.assertNotIn(.8, goals[goals.index(.2):])
        self.assertNotIn(goals[-1], (.8, .2))
        self.assertEqual(session.feedback[5], 3)
        self.assertFalse(device.expired)

    def test_timed_position_stops_refresh_when_feedback_is_lost(self):
        device = PersistentDevice()
        device.timeout_ms = 250
        shared = multiprocessing.get_context("spawn").RawValue("b", True)
        goal = [1, control.POINT_POSITION, .2, .2, .8, 0, *([.8] * 6)]
        robot, _, errors = run_bridge([.4] * 7, device=device, commands={2: goal},
            ticks=100, freeze_after=12, timed_control_started=shared)
        self.assertTrue(any("未收到电机反馈" in error for error in errors))
        following = robot.calls[robot.feedback_lost_call_index:]
        self.assertLessEqual(sum(c[1] == 0x90 for c in following), 1)
        self.assertTrue(all(c[1] == 0x06 for c in following
                            if c[0] > robot.feedback_lost_at + control.CONTROL_INTERVAL))


    def test_switch_to_position_then_back_to_torque_never_configures_watchdog(self):
        for mode in (control.GRAVITY, control.GRAVITY_FRICTION, control.IMPEDANCE):
            with self.subTest(mode=mode):
                device = PersistentDevice()
                commands = {2: [1, mode, 0, 0, math.nan, 0, *([0] * 6)],
                            200: [2, control.POSITION, 0, 0, math.nan, 0, *([0] * 6)],
                            260: [3, mode, 0, 0, math.nan, 0, *([0] * 6)]}
                robot, _, errors = run_bridge([.4] * 7, device=device, commands=commands, ticks=450)
                self.assertEqual(errors, [])
                self.assertEqual([c for c in robot.calls if c[1] == 0x85], [])
                self.assertEqual(sum(c[1] == 0x90 for c in robot.calls), 1)
                self.assertFalse(device.expired)



    def test_all_mode_pairs(self):
        for before in range(4):
            for after in range(4):
                with self.subTest(before=before, after=after):
                    device = PersistentDevice()
                    command = lambda i, mode: [i, mode, 0, 0, math.nan, 0, *([0] * 6)]
                    robot, session, errors = run_bridge([.4] * 7, device=device,
                        commands={2: command(1, before), 200: command(2, after)}, ticks=450)
                    self.assertEqual(errors, [])
                    self.assertEqual(session.feedback[1], after)
                    self.assertFalse(device.expired)

    def test_spawn_imports_extracted_session_types_without_sdk(self):
        script = """
import sys
sys.path.insert(0, SCRIPTS)
import panthera_bridge as bridge
from panthera_control import RobotSession

def check(session):
    assert session.command[0] == 0
    assert not session.stop.is_set()
    session.returned.value = True

if __name__ == '__main__':
    robot = bridge.RobotConnection()
    session = RobotSession(robot.context, robot.timed_control_started)
    process = robot.context.Process(target=check, args=(session,))
    process.start()
    process.join(5)
    assert process.exitcode == 0
    assert session.returned.value
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'spawn_check.py'
            path.write_text(script.replace('SCRIPTS', repr(str(ROOT / 'scripts'))))
            result = subprocess.run([sys.executable, str(path)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_torque_worker_exit_never_runs_sdk_destructor(self):
        for mode in (control.GRAVITY, control.GRAVITY_FRICTION, control.IMPEDANCE):
            with self.subTest(mode=mode):
                exited = []
                _, _, errors = run_bridge([0.4] * 7,
                    commands={2: [1, mode, 0, 0, math.nan, 0, *([0] * 6)]},
                    ticks=180, freeze_after=100, exit_worker=exited.append)
                self.assertTrue(errors)
                self.assertEqual(exited, [0])


class PageLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.robot = bridge.RobotConnection()
        self.robot.session = control.RobotSession(self.robot.context)
        self.session = self.robot.session
        self.threads = patch.object(bridge, "Thread").start()
        self.addCleanup(patch.stopall)

    def notify(self, stamp, leaving, page="page"):
        return self.robot.page_lifecycle({"session": self.session.id,
            "page": page, "stamp": stamp, "leaving": leaving})

    def fire(self, stamp):
        with patch.object(control.time, "sleep"), patch.object(
                self.robot, "_return_and_disconnect", return_value=(202, {})) as home:
            self.robot._return_after_page_close(self.session, "page", stamp)
            return home.call_count

    def test_frontend_notifies_close_and_restore_but_not_visibility(self):
        harness = r"""
import { createBridgePageLifecycle } from './src/bridge-page-lifecycle.js';
import assert from 'node:assert/strict';
const handlers = {}, requests = [];
let enabled = true;
const lifecycle = createBridgePageLifecycle({
  enabled: () => enabled,
  storage: { getItem: () => JSON.stringify({page:'page',session:'session'}) },
  events: { addEventListener: (name, callback) => { handlers[name] = callback; } },
  request: (url, options) => { requests.push({url, ...options}); return Promise.resolve(); },
});
assert.equal(handlers.visibilitychange, undefined);
handlers.pageshow(); handlers.pagehide(); handlers.pageshow();
assert.deepEqual(requests.map(r => JSON.parse(r.body).leaving), [false,true,false]);
assert.ok(requests.every(r => r.keepalive && r.method === 'POST'));
const stamps = requests.map(r => JSON.parse(r.body).stamp);
assert.ok(stamps[0] < stamps[1] && stamps[1] < stamps[2]);
lifecycle.setSession('session');
assert.equal(requests.length, 3);
lifecycle.setSession('new-session');
assert.equal(JSON.parse(requests.at(-1).body).session, 'new-session');
assert.equal(JSON.parse(requests.at(-1).body).leaving, false);
enabled = false;
handlers.pagehide();
assert.equal(requests.length, 4);
"""
        result = subprocess.run(["node", "--input-type=module", "-e", harness],
                                cwd=ROOT, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_close_uses_shared_return_entry(self):
        self.notify(1, False)
        self.notify(2, True)
        self.assertEqual(self.fire(2), 1)
        self.assertEqual(self.notify(2, True)[0], 200)
        self.assertEqual(self.threads.call_count, 1)

    def test_refresh_cancels_even_when_close_notification_arrives_late(self):
        self.notify(2, True)
        self.notify(3, False)
        self.assertEqual(self.fire(2), 0)
        self.assertEqual(self.notify(2, True)[0], 200)
        self.assertEqual(self.fire(2), 0)

    def test_other_open_page_prevents_return(self):
        self.notify(1, False, "other")
        self.notify(2, True)
        self.assertEqual(self.fire(2), 0)

    def test_old_session_cannot_return_new_connection(self):
        self.notify(2, True)
        self.robot.session = control.RobotSession(self.robot.context)
        self.assertEqual(self.fire(2), 0)
        self.assertEqual(self.notify(3, True)[0], 409)

    def test_manual_return_queues_same_point_command_only_once(self):
        status = {"state": "connected", "session": self.session.id,
                  "limits": [(-2, 2)] * 6, "gripperLimits": (0, 2),
                  "maxSpeed": 1, "positions": [0.2] * 6, "gripper": 0.1}
        with patch.object(self.robot, "_status", return_value=status):
            self.assertEqual(self.robot.return_and_disconnect({"session": self.session.id})[0], 202)
            self.assertEqual(self.robot.return_and_disconnect({"session": self.session.id})[0], 202)
        self.assertEqual(self.session.command[:2], [1, control.POINT_POSITION])
        self.assertEqual(self.threads.call_count, 1)


class PositionRecoveryTests(unittest.TestCase):
    """下面的记录器替代硬件，生产控制流程保持原样。"""

    def test_return_refreshes_until_settled_without_page_or_timeout_configuration(self):
        command = [1, control.POINT_POSITION, 0.2, 0.2, 0, 110, *([0] * 6)]
        robot, session, errors = run_bridge([0] * 7, commands={2: command}, page_alive=False)
        self.assertEqual(errors, [])
        self.assertTrue(session.returned.value)
        self.assertGreater(sum(c[1] == 0x90 for c in robot.calls), 1)
        self.assertFalse(any(c[1] == 0x85 for c in robot.calls))

    def test_return_timeout_stops_refreshing_without_reporting_success(self):
        command = [1, control.POINT_POSITION, 0.2, 0.2, 0, 100.3, *([0] * 6)]
        robot, session, errors = run_bridge([0.25] * 7, commands={2: command}, page_alive=False)
        self.assertFalse(session.returned.value)
        self.assertIn("等待到位超时", errors)
        self.assertGreater(sum(c[1] == 0x90 for c in robot.calls), 1)
        self.assertFalse(any(c[1] == 0x85 for c in robot.calls))

    def test_sdk_position_is_single_dispatch(self):
        robot = RecorderRobot(SimpleNamespace(now=0), [0] * 7)
        robot.Joint_Pos_Vel([0.6] * 6, [0.2] * 6, iswait=False)
        self.assertEqual(len(robot.calls), 1)
        self.assertEqual(robot.calls[0][1], 0x90)
        self.assertEqual(robot.calls[0][2][0][:2], (0.6, 0.2))

    def test_fresh_connection_only_requests_feedback(self):
        robot, _, errors = run_bridge([0.25] * 7)
        self.assertEqual(errors, [])
        self.assertEqual({call[1] for call in robot.calls}, {0x06})

    def test_position_sends_one_target_without_timeout_configuration(self):
        point = [1, control.POINT_POSITION, 0.2, 0.2, 0.12, 0, *([0.12] * 6)]
        robot, _, errors = run_bridge([0.25] * 7, commands={2: point})
        self.assertEqual(errors, [])
        commands = [call for call in robot.calls if call[1] != 0x06]
        self.assertEqual([call[1] for call in commands], [0x90])
        self.assertEqual([command[:2] for command in commands[-1][2]], [(0.12, 0.2)] * 7)

    def test_stream_watchdog_is_not_cleared_or_hidden_by_point_refresh(self):
        point = [1, control.POINT_POSITION, 0.2, 0, math.nan, 0, *([0.1] * 6)]
        stream = [2, control.STREAM_POSITION, 0.2, 0, math.nan, 0, *([0.12] * 6)]
        next_point = [3, control.POINT_POSITION, 0.2, 0, math.nan, 0, *([0.6] * 6)]
        robot, _, errors = run_bridge([0.25] * 7, device=PersistentDevice(0.25),
                                      commands={2: point, 5: stream, 10: next_point}, ticks=80)
        self.assertEqual(errors, [])
        timeouts = [call[2][0] for call in robot.calls if call[1] == 0x85]
        self.assertEqual(timeouts, [250] * 5)
        goals = [call[2][0][0] for call in robot.calls if call[1] == 0x90]
        self.assertEqual(goals.count(0.1), 1)
        self.assertGreater(goals.count(0.12), 1)
        self.assertEqual(goals.count(0.6), 1)
        self.assertEqual(robot.device.timeout_ms, 250)
        self.assertTrue(robot.device.expired)  # 已有设备超时仍是未解决的边界。

    def test_gripper_only_goal_does_not_resend_previous_arm_goal(self):
        arm = [1, control.POINT_POSITION, 0.2, 0, math.nan, 0, *([0.12] * 6)]
        gripper = [2, control.POINT_POSITION, 0, 0.2, 0.12, 0, *([0] * 6)]
        robot, session, errors = run_bridge([0.12] * 7, commands={2: arm, 8: gripper})
        self.assertEqual(errors, [])
        positions = [call[2] for call in robot.calls if call[1] == 0x90]
        self.assertEqual(len(positions), 2)
        self.assertIsNone(positions[0][-1])
        self.assertEqual(positions[1][:6], (None,) * 6)
        self.assertEqual(positions[1][-1][:2], (0.12, 0.2))
        self.assertEqual(session.feedback[5], 2)

    def test_reconnect_does_not_replay_old_host_target(self):
        old_goal = [1, control.POINT_POSITION, 0.2, 0.2, 0.6, 0, *([0.6] * 6)]
        old, old_session, errors = run_bridge([0] * 7, commands={2: old_goal})
        self.assertEqual(errors, [])
        self.assertTrue(any(call[1] == 0x90 for call in old.calls))
        new_goal = [1, control.POINT_POSITION, 0.2, 0.2, 0.12, 0, *([0.12] * 6)]
        new, new_session, errors = run_bridge([0.25] * 7, commands={2: new_goal})
        self.assertEqual(errors, [])
        self.assertNotEqual(old_session.id, new_session.id)
        self.assertEqual(sum(call[1] == 0x90 for call in new.calls), 1)
        for _, opcode, payload in new.calls:
            if opcode == 0x90:
                for command in payload:
                    if command is not None:
                        self.assertEqual(command[:2], (0.12, 0.2))

    def test_arrival_keeps_polling_without_resending(self):
        point = [1, control.POINT_POSITION, 0.2, 0.2, 0.12, 0, *([0.12] * 6)]
        robot, session, errors = run_bridge([0.12] * 7, commands={2: point})
        self.assertEqual(errors, [])
        self.assertEqual(session.feedback[5], 1)  # 已确认到位。
        self.assertEqual(sum(call[1] == 0x90 for call in robot.calls), 1)
        self.assertTrue(all(call[1] in (0x06, 0x85, 0x90) for call in robot.calls))
        self.assertEqual(robot.calls[-1][1], 0x06)  # 下一轮仍先查询反馈。

    def test_pause_sends_current_position_once(self):
        point = [1, control.POINT_POSITION, 0.2, 0.2, 0.6, 0, *([0.6] * 6)]
        pause = [2, control.POINT_PAUSE, 0, 0, math.nan, 0, *([0] * 6)]
        robot, session, errors = run_bridge([0.25] * 7, commands={2: point, 6: pause})
        self.assertEqual(errors, [])
        goals = [call[2] for call in robot.calls if call[1] == 0x90]
        self.assertEqual(len(goals), 2)
        self.assertEqual([entry[0] for entry in goals[-1]], [0.25] * 7)
        self.assertEqual(session.feedback[5], 2)

    def test_lost_feedback_does_not_continue_position_dispatch(self):
        point = [1, control.POINT_POSITION, 0.2, 0.2, 0.6, 0, *([0.6] * 6)]
        exited = []
        robot, _, errors = run_bridge([0.25] * 7, commands={2: point},
                                      ticks=100, freeze_after=4, exit_worker=exited.append)
        self.assertEqual(len(errors), 1)
        self.assertIn("未收到电机反馈", errors[0])
        following = robot.calls[robot.feedback_lost_call_index:]
        self.assertEqual(sum(call[1] == 0x90 for call in robot.calls), 1)
        self.assertTrue(all(call[1] == 0x06 for call in following))
        self.assertGreater(robot.clock.now - robot.feedback_lost_at, control.FEEDBACK_TIMEOUT)
        self.assertEqual(exited, [0])  # 生产子进程直接退出，绕过 SDK 析构时的制动指令。


if __name__ == "__main__":
    unittest.main()
