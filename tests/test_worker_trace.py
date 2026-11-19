import unittest
from unittest.mock import patch

from test_position_recovery import PersistentDevice, control, run_bridge


class WorkerTraceTests(unittest.TestCase):
    def test_clean_point_logs_gripper_scope_and_one_incident_without_resend(self):
        device = PersistentDevice(.2)
        device.timeout_ms = 250  # Unknown to this host.
        goal = control.ControlCommand(1, control.POINT_POSITION, gripper=.5, gripper_speed=.2)
        with patch.object(control, 'ControlTrace') as factory:
            robot, session, errors = run_bridge(device.positions, device=device,
                commands={2: goal.pack()}, ticks=55)
        trace = factory.return_value
        factory.assert_called_once_with(session.id)
        trace.write.assert_called_once_with(1, 'position', {'axes': ['夹爪'], 'targets': [.5], 'policy': 'once'})
        trace.failure.assert_called_once_with('位置模式丢失：夹爪')
        self.assertEqual(errors, [])
        self.assertEqual([c[1] for c in robot.calls if c[1] != 0x06], [0x90])
        samples = [c.args for c in trace.sample.call_args_list]
        self.assertTrue(any(c[0] == 1 and c[3][-1] == 10 for c in samples))
        self.assertTrue(any(c[0] == 1 and c[3][-1] == 11 for c in samples))

    def test_native_write_precedes_timeout_configuration_and_feedback_failure(self):
        device = PersistentDevice(.2)
        sine = control.ControlCommand(1, control.SDK_SINE, speed=.2, targets=(1,0,0,0,0,0))
        with patch.object(control, 'ControlTrace') as factory:
            _, _, errors = run_bridge(device.positions, device=device, commands={2: sine.pack()},
                                      freeze_after=10, ticks=200)
        trace = factory.return_value
        writes = [c.args for c in trace.write.call_args_list]
        self.assertEqual([w[1] for w in writes[:2]], ['sdk_sine', 'watchdog'])
        self.assertEqual(writes[0][2]['axes'], control.ARM_AXES)
        self.assertEqual(writes[1][2]['timeout_ms'], 250)
        self.assertIn('未收到电机反馈', errors[0])
        trace.failure.assert_called_once_with(errors[0])
