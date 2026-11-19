import logging
import unittest
from scripts.control_trace import ControlTrace


class IncidentTests(unittest.TestCase):
    def test_last_active_window_survives_loss_and_is_logged_once(self):
        t = [10.0]
        log = logging.getLogger('trace.test')
        trace = ControlTrace('s', clock=lambda: t[0], logger=log)
        with self.assertLogs(log, level='INFO') as got:
            for i in range(30):
                t[0] += .05
                trace.write(7, 'position', {'target': i})
                trace.sample(7, [i], [0], [10], [0])
            t[0] += 2
            trace.failure('反馈超时')
            trace.failure('反馈超时')
        self.assertEqual(len(got.records), 2)
        self.assertIn('2000.0ms', got.output[-1])
        self.assertIn('"target":29', got.output[-1])
        self.assertNotIn('"target":0}', got.output[-1])
        self.assertIn('"反馈"', got.output[-1])
