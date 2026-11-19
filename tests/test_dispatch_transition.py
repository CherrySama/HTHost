import unittest
from types import SimpleNamespace
from unittest.mock import patch

from test_position_recovery import PersistentDevice, control, run_bridge


def point(i=1, *, home=False):
    return control.ControlCommand(i, control.POINT_POSITION, speed=.2,
        targets=((0 if home else .6),) * 6, return_deadline=104 if home else 0).pack()


class DispatchTransitionTests(unittest.TestCase):
    def test_watchdog_survives_sdk_reconnect_and_gravity_before_position(self):
        shared = SimpleNamespace(value=False)
        device = PersistentDevice(.2)
        sine = control.ControlCommand(1, control.SDK_SINE, speed=2, targets=(1,0,0,0,0,0))
        run_bridge(device.positions, device=device, timed_control_started=shared,
                   commands={2:sine.pack()}, freeze_after=12, ticks=160)
        self.assertTrue(shared.value)
        device.advance(device.now + 1)
        robot, _, errors = run_bridge(device.positions, device=device, timed_control_started=shared,
            commands={8:control.ControlCommand(1, control.GRAVITY).pack(), 60:point(2)}, ticks=220)
        self.assertEqual(errors, [])
        writes = [c for c in robot.calls if c[1] != 0x06]
        self.assertEqual(writes[0][1], 'mit')
        points = [c for c in writes if c[1] == 0x90]
        self.assertGreater(len(points), 2)
        self.assertTrue(all(c[2][3][0] == .6 for c in points))
        self.assertEqual(device.mode, 10)
        self.assertAlmostEqual(device.positions[3], .6)

    def test_replaced_request_cannot_emit_stale_frame_or_watchdog_config(self):
        dispatch = control.RobotSession.dispatch
        def replace(session, observed):
            if observed == 1:
                session.command[:] = point(2)
            return dispatch(session, observed)
        for mode in (control.GRAVITY, control.GRAVITY_FRICTION, control.IMPEDANCE,
                     control.SDK_REPLAY, control.SDK_SINE, control.STREAM_POSITION, control.POINT_POSITION):
            with self.subTest(mode=mode), patch.object(control.RobotSession, 'dispatch', replace):
                first = control.ControlCommand(1, mode, speed=.2, targets=(1,0,0,0,0,0)).pack()
                robot, _, errors = run_bridge([.2]*7, commands={2:first}, ticks=80)
                self.assertEqual(errors, [])
                writes = [c for c in robot.calls if c[1] != 0x06]
                self.assertEqual([c[1] for c in writes], [0x90])
                self.assertEqual(writes[0][2][0][0], .6)

    def test_failed_position_feedback_does_not_rearm_watchdog_or_complete_home(self):
        for home in (False, True):
            for initially_good in (False, True):
                class Device(PersistentDevice):
                    def advance(self, now):
                        super().advance(now)
                        if initially_good and now >= 100.21:
                            self.mode = 11
                with self.subTest(home=home, initially_good=initially_good):
                    device = Device(0)
                    device.ignore_position = not initially_good
                    device.timeout_ms = 250
                    robot, session, errors = run_bridge(device.positions, device=device, ticks=60,
                        commands={2:point(home=home)}, timed_control_started=SimpleNamespace(value=True))
                    self.assertTrue(errors and '模式 11' in errors[0], errors)
                    self.assertFalse(session.returned.value)
                    writes = [c for c in robot.calls if c[1] == 0x90]
                    if initially_good:
                        self.assertGreater(len(writes), 1)
                        self.assertLess(writes[-1][0], 100.21)
                    else:
                        self.assertEqual(len(writes), 1)
