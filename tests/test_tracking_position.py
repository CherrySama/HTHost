import unittest
from unittest.mock import patch

from test_position_recovery import RecorderRobot, control, run_bridge


class TrackingPositionTests(unittest.TestCase):
    def test_stationary_gripper_requires_its_own_position_mode(self):
        original = RecorderRobot.make_motor
        for gripper_mode, fault, completed in [(11, 0, 0), (10, 0, 1), (10, 1, 0)]:
            with self.subTest(gripper_mode=gripper_mode, fault=fault):
                def make_motor(robot, i):
                    motor = original(robot, i)
                    read = motor.get_current_motor_state
                    def feedback():
                        state = read()
                        state.mode = gripper_mode if i == 6 else 11
                        if i == 6 and any(call[1] == 0x90 for call in robot.calls):
                            state.fault = fault
                        return state
                    motor.get_current_motor_state = feedback
                    return motor

                goal = control.ControlCommand(1, control.POINT_POSITION, gripper_speed=.2, gripper=.4)
                with patch.object(RecorderRobot, 'make_motor', make_motor):
                    robot, session, errors = run_bridge([.4] * 7, commands={2: goal.pack()}, ticks=40)
                self.assertEqual(bool(errors), bool(fault))
                self.assertEqual(session.feedback[5], completed)
                self.assertEqual([c[1] for c in robot.calls if c[1] != 0x06], [0x90])
