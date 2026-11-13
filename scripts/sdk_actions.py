"""Validated motion plans shared with the browser SDK panel."""

import math
import uuid
from bisect import bisect_right


def number(value):
    return type(value) in (int, float) and -math.inf < value < math.inf


def validate_plan(request, status, gripper_max_speed):
    speed, loops, raw = request.get("speed"), request.get("loops"), request.get("segments")
    if not number(speed) or not 0.01 <= speed <= status["maxSpeed"]:
        raise ValueError("速度超出真机允许范围")
    if type(loops) is not int or not 1 <= loops <= 100:
        raise ValueError("循环次数须为 1–100 的整数")
    if not isinstance(raw, list) or not 1 <= len(raw) <= 64:
        raise ValueError("动作数量超出范围")
    segments = []
    count = 0
    for item in raw:
        if not isinstance(item, dict):
            raise ValueError("动作步骤格式无效")
        dwell, points, label = item.get("dwell"), item.get("points"), item.get("label")
        if not number(dwell) or not 0 <= dwell <= 300:
            raise ValueError("停留时间超出范围")
        if not isinstance(points, list) or not points:
            raise ValueError("动作缺少姿态点")
        count += len(points)
        if count > 6000:
            raise ValueError("动作采样帧数超出范围")
        frames = []
        times = []
        elapsed = 0.0
        for point in points:
            if not isinstance(point, dict):
                raise ValueError("姿态格式无效")
            t, positions, gripper = point.get("t"), point.get("positions"), point.get("gripper")
            if (not number(t) or not 0 <= t <= 1800
                    or (not times and t != 0) or (times and t <= previous_time)):
                raise ValueError("轨迹时间戳无效")
            if (not isinstance(positions, list) or len(positions) != len(status["limits"])
                    or any(not number(pos) or not low <= pos <= high
                           for pos, (low, high) in zip(positions, status["limits"]))):
                raise ValueError("轨迹关节超出限位")
            if gripper is not None:
                limits = status["gripperLimits"]
                if not number(gripper) or not limits or not limits[0] <= gripper <= limits[1]:
                    raise ValueError("轨迹夹爪超出限位或夹爪不可用")
            if frames:
                previous, previous_gripper = frames[-1]
                travel = max(abs(a - b) / speed for a, b in zip(positions, previous))
                if previous_gripper is not None and gripper is not None:
                    travel = max(travel, abs(gripper - previous_gripper) / min(speed, gripper_max_speed))
                elapsed += max(t - previous_time, travel)
            frames.append((positions, gripper))
            times.append(elapsed)
            previous_time = t
        segments.append((str(label or "动作")[:100], dwell, times, frames))
    return speed, loops, segments


class ActionRun:
    ACTIVE = ("approach", "playing", "finishing", "dwell", "stopping")

    def __init__(self, speed, loops, segments):
        self.id = uuid.uuid4().hex
        self.speed, self.loops, self.segments = speed, loops, segments
        self.index = 0
        self.round = 1
        self.phase = "approach"
        self.command_id = None
        self.since = 0
        self.deadline = 0
        self.message = ""

    @property
    def segment(self):
        return self.segments[self.index]

    @property
    def active(self):
        return self.phase in self.ACTIVE

    def status(self):
        return {
            "id": self.id, "phase": self.phase, "round": self.round, "total": self.loops,
            "step": self.index + 1, "steps": len(self.segments), "label": self.segment[0],
            "message": self.message,
        }

    def advance(self):
        if self.index + 1 < len(self.segments):
            self.index += 1
        elif self.round < self.loops:
            self.round += 1
            self.index = 0
        else:
            self.phase = "done"
            return False
        return True

    def sample(self, elapsed):
        _, _, times, frames = self.segment
        upper = bisect_right(times, elapsed)
        if upper == len(times):
            return frames[-1]
        lower = upper - 1
        fraction = (elapsed - times[lower]) / (times[upper] - times[lower])
        a, ag = frames[lower]
        b, bg = frames[upper]
        return ([x + (y - x) * fraction for x, y in zip(a, b)],
                None if ag is None or bg is None else ag + (bg - ag) * fraction)
