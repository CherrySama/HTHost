import { Quaternion, Vector3 } from "three";

export const MAX_ACTION_SEGMENTS = 64;
export const MAX_ACTION_FRAMES = 6000;
const GRIPPER_SPEED = 0.3;

function bounded(value, [lower, upper]) {
  return Math.max(lower, Math.min(upper, value));
}

export function capturePose(pose, limits, gripperLimits) {
  return {
    positions: pose.positions.map((value, i) => bounded(value, limits[i])),
    gripper: gripperLimits && Number.isFinite(pose.gripper) ? bounded(pose.gripper, gripperLimits) : null,
  };
}

export function validatePose(pose, limits, gripperLimits) {
  if (pose.positions.length !== limits.length || pose.positions.some((value, i) =>
    !Number.isFinite(value) || value < limits[i][0] || value > limits[i][1])) {
    throw new Error("关节目标超出限位");
  }
  if (pose.gripper != null && (!gripperLimits || !Number.isFinite(pose.gripper)
      || pose.gripper < gripperLimits[0] || pose.gripper > gripperLimits[1])) {
    throw new Error("夹爪目标超出限位或夹爪不可用");
  }
}

function cartesianFrames(item, start, limits, hasGripper, simulation) {
  const origin = simulation.actionTcp(start.positions, hasGripper);
  const axis = [0, 0, 0];
  axis[item.axis] = 1;
  const rotation = item.kind === "rotate";
  const size = rotation ? item.amount * Math.PI / 180 : item.amount / 1000;
  const count = Math.max(12, Math.ceil(Math.abs(size) / (rotation ? 0.025 : 0.002)));
  let seed = start.positions;
  const points = [{ ...start, t: 0 }];
  for (let i = 1; i <= count; i++) {
    const u = i / count;
    const factor = item.kind === "sine" ? Math.sin(2 * Math.PI * u)
      : item.kind === "smooth" ? u * u * (3 - 2 * u) : u;
    const goal = {
      position: rotation ? origin.position : origin.position.map((value, j) => value + axis[j] * size * factor),
      rotation: rotation ? new Quaternion().setFromAxisAngle(new Vector3(...axis), size * factor)
        .multiply(origin.rotation) : origin.rotation,
    };
    const angles = simulation.solveActionTcp(goal, seed, limits, hasGripper);
    if (!angles || angles.some((angle, j) => Math.abs(angle - seed[j]) > 0.16)) {
      throw new Error("末端路径在第 " + i + " 个采样点不可达或逆解不连续");
    }
    points.push({ positions: angles, gripper: start.gripper, t: item.duration * u });
    seed = angles;
  }
  return points;
}

function jointSineFrames(start, limits, speed) {
  // Parameters from Panthera SDK's 3_sin_trajectory_control.py.
  const center = [-0.3, 1.1, 1.1, 0.2, -0.3, 0];
  const presetAmplitudes = [0.4, 0.6, 0.6, 0.5, 0.4, 0];
  const sdkLimits = [[-Math.PI, Math.PI], [0, Math.PI], [0, Math.PI],
    [0, Math.PI / 2], [-Math.PI / 2, Math.PI / 2], [-Math.PI, Math.PI]];
  const amplitudes = center.map((position, i) => {
    const [lower, upper] = limits[i];
    const [sdkLower, sdkUpper] = sdkLimits[i];
    const room = Math.min(Math.min(upper, sdkUpper) - position, position - Math.max(lower, sdkLower));
    if (room < 0) throw new Error("SDK 正弦中心超出当前关节限位");
    return Math.min(presetAmplitudes[i], room * 0.8);
  });
  const cycles = 2;
  const period = Math.max(5, 2 * Math.PI * Math.max(...amplitudes) / speed);
  const samples = cycles * 64;
  return Array.from({ length: samples + 1 }, (_, i) => {
    const t = period * cycles * i / samples;
    const factor = i === samples ? 0 : Math.sin(2 * Math.PI * cycles * i / samples);
    return {
      t, gripper: start.gripper,
      positions: center.map((position, joint) => position + amplitudes[joint] * factor),
    };
  });
}

function speedLimitedFrames(points, speed) {
  let elapsed = 0;
  return points.map((point, index) => {
    if (index > 0) {
      const previous = points[index - 1];
      elapsed += Math.max(point.t - previous.t,
        ...point.positions.map((value, i) => Math.abs(value - previous.positions[i]) / speed),
        point.gripper != null && previous.gripper != null
          ? Math.abs(point.gripper - previous.gripper) / Math.min(speed, GRIPPER_SPEED) : 0);
    }
    return { ...point, t: elapsed };
  });
}

export function compileActions(items, start, settings, context) {
  if (!items.length || items.length > MAX_ACTION_SEGMENTS) throw new Error("动作数量超出范围");
  const { limits, gripperLimits, simulation } = context;
  const segments = [];
  let cursor = start;
  let firstGoal;
  let transition;
  let frames = 0;
  function append(label, points, dwell) {
    if (segments.length >= MAX_ACTION_SEGMENTS) throw new Error("最多 64 个动作步骤（含返回起点）");
    points.forEach((pose) => validatePose(pose, limits, gripperLimits));
    frames += points.length;
    if (frames > MAX_ACTION_FRAMES) throw new Error("动作采样帧数超出范围");
    segments.push({ label, points: speedLimitedFrames(points, settings.speed), dwell });
  }
  for (const item of items) {
    try {
      if (item.kind === "transition") {
        if (transition) throw new Error("请在两个过渡之间插入一个动作");
        transition = item;
        continue;
      }
      let points;
      if (item.kind === "pose") points = [{ ...item.pose, t: 0 }];
      else if (item.kind === "zero") points = [{ positions: limits.map(() => 0), gripper: gripperLimits ? 0 : null, t: 0 }];
      else if (item.kind === "hold") points = [{ ...cursor, t: 0 }];
      else if (item.kind === "joint-sine") points = jointSineFrames(cursor, limits, settings.speed);
      else points = cartesianFrames(item, cursor, limits, Boolean(gripperLimits), simulation);
      if (item.kind === "joint-sine") {
        append("回零准备", [{ positions: limits.map(() => 0), gripper: cursor.gripper, t: 0 }], 0);
        cursor = segments.at(-1).points[0];
        firstGoal ??= cursor;
      }
      firstGoal ??= points[0];
      if (transition) {
        append(transition.label, [{ ...cursor, t: 0 }, { ...points[0], t: transition.duration }], 0);
        cursor = points[0];
        transition = undefined;
      }
      if (item.kind === "zero" && item.duration) {
        points = [{ ...cursor, t: 0 }, { ...points[0], t: item.duration }];
      }
      append(item.label, points, item.kind === "hold" ? item.duration : settings.dwell);
      cursor = points.at(-1);
      if (item.kind === "joint-sine") {
        append("正弦结束回零", [{ positions: limits.map(() => 0), gripper: cursor.gripper, t: 0 }], 0);
        cursor = segments.at(-1).points[0];
      }
    } catch (error) {
      throw new Error(item.label + " · " + error.message);
    }
  }
  if (transition) throw new Error("请在过渡之后添加一个动作");
  if (settings.returnToStart && firstGoal
      && (cursor.gripper !== firstGoal.gripper
        || cursor.positions.some((angle, i) => angle !== firstGoal.positions[i]))) {
    append("返回起点", [{ ...firstGoal, t: 0 }], settings.dwell);
  }
  return { segments, speed: settings.speed, loops: settings.loops };
}

export function actionFrame(points, time) {
  if (time >= points.at(-1).t) return points.at(-1);
  let lower = 0;
  let upper = points.length - 1;
  while (upper - lower > 1) {
    const middle = Math.floor((lower + upper) / 2);
    if (points[middle].t > time) upper = middle;
    else lower = middle;
  }
  const from = points[lower];
  const to = points[upper];
  const fraction = (time - from.t) / (to.t - from.t);
  return {
    positions: from.positions.map((value, i) => value + (to.positions[i] - value) * fraction),
    gripper: from.gripper == null || to.gripper == null ? null
      : from.gripper + (to.gripper - from.gripper) * fraction,
  };
}

export function actionPreviewPoints(plan, start) {
  const result = [start];
  for (const segment of plan.segments) {
    const entry = [{ ...result.at(-1), t: 0 }, { ...segment.points[0], t: 1 }];
    for (let i = 1; i < 12; i++) result.push(actionFrame(entry, i / 12));
    result.push(...segment.points);
  }
  return result;
}

export function travelTime(from, to, speed) {
  return Math.max(...from.positions.map((value, i) => Math.abs(value - to.positions[i]) / speed),
    from.gripper == null || to.gripper == null ? 0
      : Math.abs(from.gripper - to.gripper) / Math.min(speed, GRIPPER_SPEED));
}

export class SimulationActions {
  constructor(plan, pose, now) {
    this.plan = plan;
    this.index = 0;
    this.round = 1;
    this.begin(pose, now);
  }

  get segment() {
    return this.plan.segments[this.index];
  }

  get status() {
    return { phase: this.phase, round: this.round, step: this.index + 1, label: this.segment.label,
      total: this.plan.loops, steps: this.plan.segments.length };
  }

  begin(pose, now) {
    this.phase = "approach";
    this.since = now;
    const first = this.segment.points[0];
    this.entry = [{ ...pose, t: 0 }, { ...first, t: travelTime(pose, first, this.plan.speed) }];
    this.stable = null;
  }

  tick(simulation) {
    const now = simulation.data.time;
    const { points, dwell } = this.segment;
    const time = now - this.since;
    const goal = this.phase === "approach" ? actionFrame(this.entry, time)
      : this.phase === "playing" ? actionFrame(points, time) : points.at(-1);
    goal.positions.forEach((angle, i) => simulation.setTarget(i, angle));
    simulation.setGripperTarget(goal.gripper);
    if (this.phase === "dwell") {
      if (time < dwell) return true;
      if (this.index + 1 === this.plan.segments.length) {
        if (this.round === this.plan.loops) return false;
        this.round++;
        this.index = 0;
      } else this.index++;
      this.begin(simulation.getActionPose(), now);
      return true;
    }
    const duration = this.phase === "approach" ? this.entry.at(-1).t : points.at(-1).t;
    if (time < duration) return true;
    if (!simulation.actionPoseSettled(goal)) {
      this.stable = null;
      if (time > duration + 8) throw new Error("未能到位，动作已停止");
      return true;
    }
    this.stable ??= now;
    if (now - this.stable < 0.15) return true;
    this.phase = this.phase === "approach" && points.length > 1 ? "playing" : "dwell";
    this.since = now;
    this.stable = null;
    return true;
  }
}
