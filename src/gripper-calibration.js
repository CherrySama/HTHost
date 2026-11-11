// Total opening in mm from the supplied calibration document.
// The (0 rad, 0 mm) endpoint was confirmed by the user.
export const GRIPPER_CALIBRATION = [
  [0, 0],
  [0.25, 8.04],
  [0.5, 17.14],
  [0.75, 28.01],
  [1, 39.91],
  [1.25, 52.81],
  [1.5, 65.72],
  [1.75, 76.91],
  [2, 82],
];

export function gripperOpening(angle) {
  const value = Math.max(0, Math.min(2, angle));
  for (let i = 1; i < GRIPPER_CALIBRATION.length; i++) {
    const [a, width] = GRIPPER_CALIBRATION[i];
    const [previous, previousWidth] = GRIPPER_CALIBRATION[i - 1];
    if (value <= a)
      return (
        previousWidth +
        ((width - previousWidth) * (value - previous)) / (a - previous)
      );
  }
  return 82;
}
