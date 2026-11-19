import unittest
from unittest.mock import patch

from test_position_recovery import PersistentDevice, bridge, control, run_bridge


class ManualPauseScopeTests(unittest.TestCase):
    def test_pause_holds_only_groups_requested_even_if_it_replaced_an_unread_goal(self):
        for arm, gripper in ((False, True), (True, False), (True, True)):
            for consumed in (False, True):
                with self.subTest(arm=arm, gripper=gripper, consumed=consumed):
                    connection = bridge.RobotConnection()
                    connection.session = session = control.RobotSession(connection.context)
                    status = dict(state='connected', session=session.id, mode='position',
                                  pendingMode=None, positionPhase=None, maxSpeed=1,
                                  gripperLimits=[0, 2], limits=[[-2, 2]] * 6)
                    request = dict(session=session.id, speed=.2)
                    if arm:
                        request['positions'] = [.8] * 6
                    if gripper:
                        request['gripper'] = .8
                    with patch.object(connection, '_status', return_value=status):
                        code, goal = connection.set_position(request)
                        self.assertEqual(code, 202)
                        original = session.command[:]
                        status.update(positionPhase='moving', positionStatus=goal)
                        code, _ = connection.pause_position(dict(session=session.id, commandId=goal['commandId']))
                        self.assertEqual(code, 202)
                    commands = {6: session.command[:]}
                    if consumed:
                        commands[2] = original
                    robot, worker, errors = run_bridge([.25] * 7, device=PersistentDevice(.25), commands=commands)
                    self.assertEqual(errors, [])
                    writes = [c[2] for c in robot.calls if c[1] == 0x90]
                    self.assertEqual(len(writes), 1 + consumed)
                    hold = writes[-1]
                    self.assertEqual([x is not None for x in hold], [arm] * 6 + [gripper])
                    for i, entry in enumerate(hold):
                        if entry is not None:
                            self.assertAlmostEqual(entry[0], robot.device.positions[i])
                            self.assertLess(entry[0], .8)
                    self.assertEqual(worker.feedback[5], goal['commandId'] + 1)
