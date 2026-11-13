"""Validated motion plans shared with the browser SDK panel."""

import math
import uuid
from bisect import bisect_right

MAX_RECORDING_FRAMES = 5999  # Leave room in a motion plan for the optional return step.
RECORDING_INTERVAL = 0.05
NATIVE_WIDTH = 15  # time, six positions, six velocities, gripper position and velocity
MAX_NATIVE_FRAMES = 180_002


class NativeTrajectory:
    def __init__(self, kind, rows, duration=None):
        self.kind = kind
        self.rows = rows
        self.duration = rows[-1][0] if duration is None else duration
        self.first = (rows[0][1:7], self._gripper(rows[0]))
        self.last = (rows[-1][1:7], self._gripper(rows[-1]))

    @staticmethod
    def _gripper(row):
        return row[13] if math.isfinite(row[13]) else None


def prepare_native_trajectory(kind, points, speed, gripper_speed, status, recorder):
    if len(points) < 2:
        raise ValueError("SDK 轨迹至少需要两个采样点")
    if kind == "joint-sine":
        center = points[0]["positions"]
        gripper = points[0].get("gripper")
        limits = [(-math.pi, math.pi), (0, math.pi), (0, math.pi), (0, math.pi / 2),
                  (-math.pi / 2, math.pi / 2), (-math.pi, math.pi)]
        amplitudes = []
        for value, (low, high), (sdk_low, sdk_high), preset in zip(
                center, status["limits"], limits, (0.4, 0.6, 0.6, 0.5, 0.4, 0)):
            room = min(value - max(low, sdk_low), min(high, sdk_high) - value)
            if room < 0:
                raise ValueError("SDK 正弦中心超出关节限位")
            amplitudes.append(min(preset, room * 0.8))
        period = max(5, 2 * math.pi * max(amplitudes) / speed)
        row = [0, *center, *amplitudes, math.nan if gripper is None else gripper, 0]
        return NativeTrajectory(kind, [row], duration=2 * period)

    has_gripper = points[0].get("gripper") is not None
    source = []
    for point in points:
        gripper = point.get("gripper")
        if (gripper is not None) != has_gripper:
            raise ValueError("轨迹夹爪数据不一致")
        frame = {"t": point["t"], "pos": point["positions"]}
        if has_gripper:
            frame.update(gripper_pos=gripper, gripper_vel=0)
        source.append(frame)
    # The SDK uses a 10 ms grid and a seven-sample moving average. Keep a
    # minimum two-sample grid so the SDK can compute a velocity gradient.
    interval = min(0.01, source[-1]["t"])
    try:
        prepared = recorder._prepare_playback_frames(source, interval, 7)
    except (ValueError, IndexError) as exc:
        raise ValueError("SDK 无法处理该轨迹的时间戳") from exc
    if len(prepared) < 2 or len(prepared) > MAX_NATIVE_FRAMES:
        raise ValueError("SDK 回放采样数量超出范围")

    samples = []
    for frame in prepared:
        positions = [max(low, min(high, value)) for value, (low, high) in zip(frame["pos"], status["limits"])]
        if has_gripper:
            low, high = status["gripperLimits"]
            positions.append(max(low, min(high, frame["gripper_pos"])))
        samples.append(positions)

    caps = [speed] * 6 + ([min(speed, gripper_speed)] if has_gripper else [])
    stretch = []
    for a, b, before, after in zip(samples, samples[1:], prepared, prepared[1:]):
        dt = after["t"] - before["t"]
        stretch.append(max(0.0, max(abs(y - x) / (dt * cap)
                                   for x, y, cap in zip(a, b, caps)) - 1))

    # Extend local slowdowns into neighboring intervals so the playback clock
    # changes gradually without ever shortening a speed-limited interval.
    decay = math.exp(-interval / 0.10)
    for i in range(1, len(stretch)):
        stretch[i] = max(stretch[i], stretch[i - 1] * decay)
    for i in range(len(stretch) - 2, -1, -1):
        stretch[i] = max(stretch[i], stretch[i + 1] * decay)
    times = [0.0]
    for extra, before, after in zip(stretch, prepared, prepared[1:]):
        times.append(times[-1] + (after["t"] - before["t"]) * (1 + extra))
    if not math.isfinite(times[-1]) or times[-1] > 1800:
        raise ValueError("按当前速度回放将超过 30 分钟")

    rows = []
    for i, (stamp, positions) in enumerate(zip(times, samples)):
        if i == 0 or i == len(samples) - 1:
            velocities = [0.0] * len(caps)
        else:
            dt = times[i + 1] - times[i - 1]
            velocities = [(b - a) / dt for a, b in zip(samples[i - 1], samples[i + 1])]
        gripper = positions[6] if has_gripper else math.nan
        rows.append([stamp, *positions[:6], *velocities[:6], gripper,
                     velocities[6] if has_gripper else 0])
    return NativeTrajectory(kind, rows)


def native_target(mode_is_sine, data, count, duration, elapsed, index):
    if mode_is_sine:
        _, *values = data[:NATIVE_WIDTH]
        center, amplitudes, gripper = values[:6], values[6:12], values[12]
        omega = 4 * math.pi / duration
        phase = omega * min(elapsed, duration)
        if elapsed >= duration:
            return center, [0.0] * 6, gripper, 0, 0
        return ([p + a * math.sin(phase) for p, a in zip(center, amplitudes)],
                [a * omega * math.cos(phase) for a in amplitudes], gripper, 0, 0)
    while index + 1 < count and data[(index + 1) * NATIVE_WIDTH] <= elapsed:
        index += 1
    start = index * NATIVE_WIDTH
    left = data[start:start + NATIVE_WIDTH]
    if index + 1 == count:
        return left[1:7], [0.0] * 6, left[13], 0, index
    right = data[start + NATIVE_WIDTH:start + 2 * NATIVE_WIDTH]
    weight = (elapsed - left[0]) / (right[0] - left[0])
    target = [a + (b - a) * weight for a, b in zip(left[1:13], right[1:13])]
    gripper = left[13] + (right[13] - left[13]) * weight
    return target[:6], target[6:12], gripper, left[14] + (right[14] - left[14]) * weight, index


def number(value):
    return type(value) in (int, float) and -math.inf < value < math.inf


class TrajectoryRecording:
    def __init__(self, session):
        self.id = uuid.uuid4().hex
        self.session = session
        self.active = True
        self.frames = []
        self.origin = None

    def sample(self, status):
        stamp = status["sampleTime"]
        if self.origin is None:
            self.origin = stamp
        elapsed = stamp - self.origin
        if self.frames and elapsed < self.frames[-1]["t"] + RECORDING_INTERVAL:
            return
        frame = {"t": elapsed, "pos": status["positions"],
                 "vel": [motor["velocity"] for motor in status["motors"][:6]]}
        if status["gripper"] is not None:
            frame.update(gripper_pos=status["gripper"], gripper_vel=status["motors"][6]["velocity"])
        self.frames.append(frame)
        if len(self.frames) >= MAX_RECORDING_FRAMES:
            self.active = False

    def status(self):
        return {"id": self.id, "active": self.active, "count": len(self.frames)}


def validate_plan(request, status, gripper_max_speed, prepare_native=None):
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
        native = item.get("native")
        if native not in (None, "joint-sine", "recording") or (native and prepare_native is None):
            raise ValueError("未知的 SDK 轨迹类型")
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
        trajectory = prepare_native(native, points, speed, gripper_max_speed, status) if native else None
        if trajectory:
            times, frames = [0, trajectory.duration], [trajectory.first, trajectory.last]
        segments.append((str(label or "动作")[:100], dwell, times, frames, trajectory))
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
        _, _, times, frames, _ = self.segment
        upper = bisect_right(times, elapsed)
        if upper == len(times):
            return frames[-1]
        lower = upper - 1
        fraction = (elapsed - times[lower]) / (times[upper] - times[lower])
        a, ag = frames[lower]
        b, bg = frames[upper]
        return ([x + (y - x) * fraction for x, y in zip(a, b)],
                None if ag is None or bg is None else ag + (bg - ag) * fraction)
