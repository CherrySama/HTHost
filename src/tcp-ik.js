const POSITION_TOLERANCE = 0.004;
const DAMPING = 0.02;
const MAX_JOINT_STEP = 0.12;
const MAX_ITERATIONS = 30;
const LIMIT_EPSILON = 1e-7;

function dampedStep(columns, error, enabled) {
  // Solve (J Jᵀ + λ² I) y = error, then compute Jᵀ y.
  let xx = DAMPING ** 2, yy = xx, zz = xx;
  let xy = 0, xz = 0, yz = 0;
  columns.forEach(([x, y, z], index) => {
    if (!enabled[index]) return;
    xx += x * x;
    yy += y * y;
    zz += z * z;
    xy += x * y;
    xz += x * z;
    yz += y * z;
  });
  const a = yy * zz - yz * yz;
  const b = xz * yz - xy * zz;
  const c = xy * yz - xz * yy;
  const d = xx * zz - xz * xz;
  const e = xy * xz - xx * yz;
  const f = xx * yy - xy * xy;
  const determinant = xx * a + xy * b + xz * c;
  const [ex, ey, ez] = error;
  const rx = (a * ex + b * ey + c * ez) / determinant;
  const ry = (b * ex + d * ey + e * ez) / determinant;
  const rz = (c * ex + e * ey + f * ez) / determinant;
  return columns.map(([x, y, z], i) => enabled[i] ? x * rx + y * ry + z * rz : 0);
}

export function solveTcpPosition(evaluate, initial, limits, goal) {
  if (!goal.every(Number.isFinite)) return null;
  const clamp = (value, i) => Math.max(limits[i][0], Math.min(limits[i][1], value));
  const errorFor = (position) => goal.map((value, i) => value - position[i]);
  let angles = initial.map(clamp);
  let pose = evaluate(angles);
  let error = errorFor(pose.position);
  let distance = Math.hypot(...error);

  for (let iteration = 0; iteration < MAX_ITERATIONS && distance > POSITION_TOLERANCE; iteration++) {
    const enabled = angles.map(() => true);
    let step = dampedStep(pose.columns, error, enabled);
    // Remove columns whose step points outside an already active joint limit.
    for (let pass = 0; pass < angles.length; pass++) {
      let changed = false;
      angles.forEach((angle, i) => {
        if (
          (angle <= limits[i][0] + LIMIT_EPSILON && step[i] < 0) ||
          (angle >= limits[i][1] - LIMIT_EPSILON && step[i] > 0)
        ) {
          enabled[i] = false;
          changed = true;
        }
      });
      if (!changed) break;
      step = dampedStep(pose.columns, error, enabled);
    }

    const maxStep = Math.max(...step.map(Math.abs));
    if (maxStep < 1e-8) break;
    let improved = false;
    for (let scale = Math.min(1, MAX_JOINT_STEP / maxStep); scale >= 0.015; scale /= 2) {
      const candidate = angles.map((angle, i) => clamp(angle + scale * step[i], i));
      const nextPose = evaluate(candidate);
      const nextError = errorFor(nextPose.position);
      const nextDistance = Math.hypot(...nextError);
      if (nextDistance >= distance) continue;
      angles = candidate;
      pose = nextPose;
      error = nextError;
      distance = nextDistance;
      improved = true;
      break;
    }
    if (!improved) break;
  }
  return distance <= POSITION_TOLERANCE ? angles : null;
}

// Solve the six-dimensional damped normal equations with pivoting.
function poseStep(columns, error) {
  const matrix = error.map((value, i) => [
    ...error.map((_, j) => columns.reduce((sum, col) => sum + col[i] * col[j], i === j ? DAMPING ** 2 : 0)),
    value,
  ]);
  for (let i = 0; i < error.length; i++) {
    let pivot = i;
    for (let j = i + 1; j < error.length; j++) {
      if (Math.abs(matrix[j][i]) > Math.abs(matrix[pivot][i])) pivot = j;
    }
    [matrix[i], matrix[pivot]] = [matrix[pivot], matrix[i]];
    const divisor = matrix[i][i];
    matrix[i] = matrix[i].map((value) => value / divisor);
    matrix.forEach((row, j) => {
      if (i === j) return;
      const factor = row[i];
      matrix[j] = row.map((value, k) => value - factor * matrix[i][k]);
    });
  }
  return columns.map((column) => column.reduce((sum, value, i) => sum + value * matrix[i].at(-1), 0));
}

export function solveTcpPose(evaluate, seed, limits) {
  let angles = seed.slice();
  let pose = evaluate(angles);
  for (let i = 0; i < 45; i++) {
    if (Math.hypot(...pose.error.slice(0, 3)) <= 0.0015
        && Math.hypot(...pose.error.slice(3)) <= 0.003) return angles;
    const step = poseStep(pose.columns, pose.error);
    const distance = Math.hypot(...pose.error);
    let improved = false;
    for (let scale = Math.min(1, MAX_JOINT_STEP / Math.max(...step.map(Math.abs))); scale >= 0.015; scale /= 2) {
      const candidate = angles.map((angle, j) => Math.max(limits[j][0], Math.min(limits[j][1], angle + scale * step[j])));
      const next = evaluate(candidate);
      if (Math.hypot(...next.error) >= distance) continue;
      angles = candidate;
      pose = next;
      improved = true;
      break;
    }
    if (!improved) break;
  }
  return null;
}
