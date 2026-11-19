import unittest
from unittest.mock import patch
from test_position_recovery import RecorderRobot, PersistentDevice, control, run_bridge


class SdkDampingTests(unittest.TestCase):
    def test_stop_replaces_trajectory_without_position_hold_or_retry(self):
        def brake(robot):
            robot.buffer = ('brake', [True] * 7)
            robot.motor_send_cmd()
            return True

        for mode in (control.MOVE_POSITION, control.SDK_SINE, control.SDK_REPLAY):
            with self.subTest(mode=mode), patch.object(RecorderRobot, 'set_brake', brake, create=True):
                prior = control.ControlCommand(1, mode, speed=.2, targets=(1, 0, 0, 0, 0, 0))
                stop = control.ControlCommand(2, control.SDK_DAMPING)
                robot, session, errors = run_bridge([.2] * 7, device=PersistentDevice(.2),
                    commands={2: prior.pack(), 10: stop.pack()}, ticks=40)
                self.assertEqual(errors, [])
                codes = [c[1] for c in robot.calls if c[1] != 0x06]
                idx = codes.index('brake')
                self.assertGreater(idx, 0)
                self.assertEqual(codes[idx:], ['brake'])
                self.assertEqual(session.feedback[1], control.SDK_DAMPING)
                self.assertEqual(session.feedback[3], 2)
