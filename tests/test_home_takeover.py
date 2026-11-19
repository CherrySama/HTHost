import unittest
from unittest.mock import patch

from test_position_recovery import bridge, control


class HomeTakeoverTests(unittest.TestCase):
    def setUp(self):
        self.robot = bridge.RobotConnection()
        self.session = self.robot.session = control.RobotSession(self.robot.context)
        self.status = dict(state='connected', session=self.session.id, limits=[(-2, 2)] * 6,
                           gripperLimits=(0, 2), maxSpeed=1, positions=[.2] * 6, gripper=.1)
        self.robot.action = bridge.ActionRun(.2, 1, [('回放', 0, [0], [], None)])
        self.robot.recording = bridge.TrajectoryRecording(self.session.id)
        self.status_mock = patch.object(self.robot, '_status', side_effect=lambda: self.status.copy()).start()
        self.watcher = patch.object(bridge, 'Thread').start()
        self.addCleanup(patch.stopall)

    def disconnect(self):
        return self.robot.return_and_disconnect({'session': self.session.id})

    def verify_takeover(self):
        self.assertEqual(self.robot.action.phase, 'cancelled')
        self.assertFalse(self.robot.recording.active)
        command = control.ControlCommand.unpack(self.session.command[:])
        self.assertEqual(command.mode, control.POINT_POSITION)
        self.assertEqual(command.targets, (0,) * 6)
        self.assertEqual(command.return_deadline, self.robot.return_deadline)
        self.assertGreater(command.return_deadline, 0)
        # Resume producers as if they had been waiting for the connection lock.
        self.status_mock.reset_mock()
        self.robot._run_action(self.robot.action)
        self.robot._record_trajectory(self.robot.recording)
        self.status_mock.assert_not_called()
        self.assertEqual(self.session.command[0], command.id)
        self.watcher.assert_called_once()

    def test_manual_disconnect_takes_over_and_preserves_existing_action_error(self):
        self.robot.action.phase = 'stopping'
        self.robot.action.message = '未能到达动作目标'
        code, status = self.disconnect()
        self.assertEqual(code, 202)
        self.assertEqual(status['action']['phase'], 'cancelled')
        self.assertEqual(status['action']['message'], '未能到达动作目标')
        self.assertFalse(status['recording']['active'])
        self.verify_takeover()

    def test_last_page_close_takes_over_active_replay(self):
        self.robot.action.phase = 'playing'
        self.robot.pages['page'] = (2, True)
        with patch.object(bridge.time, 'sleep'):
            self.robot._return_after_page_close(self.session, 'page', 2)
        self.verify_takeover()

    def test_invalid_home_or_failed_connection_retains_error_and_does_not_cancel(self):
        self.status['limits'][0] = (1, 2)
        self.assertEqual(self.disconnect(), (409, {'message': '零位超出真机限位'}))
        self.status = {'state': 'error', 'message': '电机反馈已中断'}
        self.assertEqual(self.disconnect(), (409, {'message': '电机反馈已中断'}))
        self.assertTrue(self.robot.action.active)
        self.assertTrue(self.robot.recording.active)
        self.assertEqual(self.session.command[0], 0)
        self.assertFalse(self.robot.return_deadline)
        self.watcher.assert_not_called()
