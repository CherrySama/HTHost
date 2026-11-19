import unittest
from unittest.mock import patch

from test_position_recovery import bridge, control


class BridgeCommandLogTests(unittest.TestCase):
    def setUp(self):
        self.robot = bridge.RobotConnection()
        self.session = self.robot.session = control.RobotSession(self.robot.context)
        self.status = dict(state='connected', session=self.session.id, mode='position',
                           pendingMode=None, positionPhase=None, maxSpeed=1, gripper=.2,
                           gripperLimits=(0, 2), limits=[(-2, 2)] * 6, positions=[0] * 6,
                           motors=[{'position': 0}] * 7, calibration={'ready': True})
        self.req = {'session': self.session.id}
        self.enter_patches()

    def enter_patches(self):
        for mock in (patch.object(self.robot, '_status', side_effect=lambda: self.status.copy()),
                     patch.object(bridge, 'Thread')):
            mock.start()
            self.addCleanup(mock.stop)

    def test_accepted_sources_match_shared_command_ids(self):
        with self.assertLogs(bridge.command_log, 'INFO') as logs:
            self.robot.set_position(dict(self.req, gripper=.3, speed=.2))
            self.status.update(positionPhase='moving', positionStatus={'commandId': 1})
            self.robot.pause_position(dict(self.req, commandId=1))
            self.status.update(positionPhase=None)
            self.robot.set_mode(dict(self.req, mode='gravity'))
            self.robot.write_zero(dict(self.req, positions=[0] * 7))
            self.robot.return_and_disconnect(self.req)
            self.robot.pages['p'] = (1, True)
            with patch.object(bridge.time, 'sleep'):
                self.robot._return_after_page_close(self.session, 'p', 1)
        sources = ['用户夹爪位置', '用户位置暂停 原命令=1', '用户模式切换 position->gravity', '用户写零',
                   '主动断开回零', '关页回零']
        self.assertEqual([r.getMessage() for r in logs.records],
                         [f'会话={self.session.id} 命令={i} 阶段=桥接入队 来源={s}'
                          for i, s in enumerate(sources, 1)])
        self.assertEqual(self.session.command[0], len(sources))

    def test_rejected_or_noop_request_does_not_report_a_new_command(self):
        with self.assertNoLogs(bridge.command_log, 'INFO'):
            self.assertEqual(self.robot.set_mode(dict(self.req, mode='position'))[0], 200)
            self.assertEqual(self.robot.set_position(dict(self.req, speed=-1, gripper=.3))[0], 400)
        self.assertEqual(self.session.command[0], 0)
