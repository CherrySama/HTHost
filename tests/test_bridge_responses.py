"""Exercise real handler dispatch and trajectory preparation without USB or a socket."""
import io
import json
import runpy
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from scripts import panthera_bridge as bridge


class BridgeResponseTests(unittest.TestCase):
    def test_prepare_native_without_usb(self):
        recorder = runpy.run_path(str(bridge.SDK / 'scripts/Panthera_lib/recorder.py'))['Recorder']
        points = [{'t': t, 'positions': [v] * 6} for t, v in [(0, .25), (1, .3)]]
        with patch.object(sys, 'path', sys.path.copy()), patch.dict(sys.modules, {'Panthera_lib': SimpleNamespace(TrajectoryRecorder=recorder)}):
            for kind in ('joint-sine', 'recording'):
                with self.subTest(kind=kind):
                    trajectory = bridge.RobotConnection._prepare_native(kind, points, .2, .2, {'limits': [(-2, 2)] * 6})
                    self.assertGreater(trajectory.duration, 0)

    def test_get_and_post_failures_are_json_and_not_retried(self):
        for method, path, action in [('GET', bridge.ENDPOINT, 'status'), ('POST', bridge.ACTION_ENDPOINT, 'start_action')]:
            with self.subTest(method=method):
                handler = object.__new__(bridge.Handler)
                handler.command, handler.path = method, path
                handler.headers = {'Content-Type': 'application/json', 'Content-Length': '2'}
                handler.connection, handler.rfile, handler.wfile = Mock(), io.BytesIO(b'{}'), io.BytesIO()
                handler.send_response, handler.send_header, handler.end_headers = Mock(), Mock(), Mock()
                callback = Mock(side_effect=RuntimeError('internal detail'))
                handler.server = SimpleNamespace(robot=SimpleNamespace(**{action: callback}))
                with self.assertLogs(bridge.__name__, level='ERROR'):
                    getattr(handler, 'do_' + method)()
                callback.assert_called_once()
                handler.send_response.assert_called_once_with(500)
                handler.send_header.assert_any_call('Content-Type', 'application/json; charset=utf-8')
                message = json.loads(handler.wfile.getvalue())['message']
                self.assertIn('结果未确认', message)
                self.assertNotIn('internal detail', message)
