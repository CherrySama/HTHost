"""Bounded worker-side incident history; never poll hardware to produce a trace."""
from collections import deque
import json
import logging
import math
from time import monotonic


class ControlTrace:
    def __init__(self, session_id, *, clock=monotonic, logger=None):
        self.session_id = session_id
        self.clock = clock
        self.logger = logger if logger is not None else logging.getLogger('hthost.trace')
        self.history = deque(maxlen=1024)
        self.command_id = None
        self.written = set()
        self.incidents = set()
        self.last_write = None

    def _record(self, now, command_id, kind, details):
        if command_id != self.command_id:
            self.command_id = command_id
            self.written.clear()
            self.incidents.clear()
        self.history.append((now, command_id, kind, details))
        while self.history and now - self.history[0][0] > 1.0:
            self.history.popleft()

    @staticmethod
    def _json(value):
        return json.dumps(value, ensure_ascii=False, separators=(',', ':'), default=str)

    def write(self, command_id, kind, details):
        now = self.clock()
        self._record(now, command_id, kind, details)
        self.last_write = now
        if kind not in self.written:
            self.written.add(kind)
            self.logger.info('会话=%s 指令=%s 单调时间=%.6f SDK调用返回=%s %s',
                             self.session_id, command_id, now, kind, self._json(details))

    def sample(self, command_id, positions, velocities, modes, faults):
        self._record(self.clock(), command_id, '反馈', (tuple(positions), tuple(velocities), tuple(modes), tuple(faults)))

    def failure(self, reason):
        reason = str(reason)
        if reason in self.incidents:
            return
        self.incidents.add(reason)
        now = self.clock()
        age = '无' if self.last_write is None else f'{(now-self.last_write)*1000:.1f}ms'
        # Preserve the last active window even if the feedback timeout itself is >1s.
        def row(entry):
            stamp, cmd, event, payload = entry
            if event == '反馈':
                payload = [[round(v, 5) if math.isfinite(v) else None for v in group] for group in payload]
            return [round((stamp-now)*1000, 1), cmd, event, payload]
        self.logger.warning('会话=%s 指令=%s 单调时间=%.6f 异常=%s 距最近SDK写调用返回=%s '
                            '近期记录[相对异常ms,指令,事件,详情]=%s', self.session_id, self.command_id,
                            now, reason, age, self._json([row(entry) for entry in self.history]))
