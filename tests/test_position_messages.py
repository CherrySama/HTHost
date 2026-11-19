import unittest
from test_position_recovery import bridge


class PositionMessages(unittest.TestCase):
    def test_distinguishes_exit_from_no_response_even_when_stationary_at_goal(self):
        motor = dict(target=0, position=0, velocity=0, driveMode=11)
        self.assertIn('尚未观察到', bridge.describe_position_status([motor], 1, 1, 0)['message'])
        self.assertIn('已退出', bridge.describe_position_status([motor], 1, .2, 1)['message'])
        motor['driveMode'] = 10
        self.assertIn('确认停稳', bridge.describe_position_status([motor], 1, .3, 1)['message'])
