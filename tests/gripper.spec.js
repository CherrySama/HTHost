import { test, expect } from "@playwright/test";

test("夹爪标定点、对称开合、端点和复位", async ({ page }) => {
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.goto("/");
  await expect(page.locator("#loading")).toBeHidden({ timeout: 60000 });
  await page.locator("#pause").click();
  await page.locator("#gripper-angle").fill("1");
  await page.locator("#gripper-angle").press("Enter");
  await expect(page.locator("#gripper-target-opening")).toHaveText(
    "目标总开口 39.91 mm",
  );
  const results = await page.evaluate(() => {
    const sim = panthera.simulation;
    const points = [
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
    return points.map(([angle, expected]) => {
      sim.setGripperTarget(angle);
      sim.paused = false;
      for (let step = 0; step < 120; step++) sim.advance(0.025);
      sim.paused = true;
      return { ...sim.getGripperState(), expected };
    });
  });
  for (const result of results) {
    expect(result.targetOpening).toBeCloseTo(result.expected, 5);
    expect(result.opening).toBeCloseTo(result.expected, 1);
    expect(Math.abs(result.left + result.right)).toBeLessThan(0.00001);
  }
  await page.locator("#gripper-close").click();
  await expect(page.locator("#gripper-angle")).toHaveValue("0.00");
  await page.locator("#gripper-open").click();
  await expect(page.locator("#gripper-angle")).toHaveValue("2.00");
  await page.locator("#gripper-angle").fill("9");
  await page.locator("#gripper-angle").press("Enter");
  await expect(page.locator("#gripper-angle")).toHaveValue("2.00");
  await page.locator("#reset").click();
  expect(
    await page.evaluate(() => panthera.simulation.getGripperState().opening),
  ).toBe(82);
  expect(errors).toEqual([]);
});

test("J5 与 J6 高亮各自的连杆", async ({ page }) => {
  await page.goto("/");
  await expect(page.locator("#loading")).toBeHidden({ timeout: 60000 });
  await page.locator("#slider-4").dispatchEvent("pointerdown");
  expect(
    await page.evaluate(() => {
      const { simulation: sim, viewer } = panthera;
      const link5 = sim.mj.mj_name2id(
        sim.model,
        sim.mj.mjtObj.mjOBJ_BODY.value,
        "link5",
      );
      const items = viewer.highlight.items;
      return items.length > 0 && items.every((item) => item.bodyId === link5);
    }),
  ).toBe(true);
  await page.locator("#slider-5").dispatchEvent("pointerdown");
  await page.locator("#slider-5").fill("0.3");
  const state = await page.evaluate(() => {
    const { simulation: sim, viewer } = panthera;
    const id = (name) =>
      sim.mj.mj_name2id(sim.model, sim.mj.mjtObj.mjOBJ_BODY.value, name);
    const bodies = viewer.highlight.items.map((item) => item.bodyId);
    return {
      center: bodies.includes(id("gripper_center_link")),
      output: bodies.includes(id("link6")),
      housing: bodies.includes(id("link5")),
      fingers:
        bodies.includes(id("gripper_L_link")) ||
        bodies.includes(id("gripper_R_link")),
      camera: bodies.includes(id("camera_link")),
      rings: viewer.scene.children.some(
        (item) => item.geometry?.type === "TorusGeometry",
      ),
    };
  });
  expect(state).toEqual({
    center: true,
    output: true,
    housing: false,
    fingers: false,
    camera: false,
    rings: false,
  });
  await page.evaluate(() =>
    window.dispatchEvent(new PointerEvent("pointerup")),
  );
  await page.waitForFunction(() => !panthera.viewer.highlight);
});

test("夹爪控件独立高亮左右夹指", async ({ page }) => {
  await page.goto("/");
  await expect(page.locator("#loading")).toBeHidden({ timeout: 60000 });

  const readHighlight = () => page.evaluate(() => {
    const { simulation: sim, viewer } = panthera;
    const bodies = [...new Set(viewer.highlight.items.map((item) => item.bodyId))];
    const fingers = ["gripper_L_link", "gripper_R_link"].map((name) =>
      sim.mj.mj_name2id(sim.model, sim.mj.mjtObj.mjOBJ_BODY.value, name),
    );
    return {
      held: viewer.highlight.held,
      onlyFingers:
        bodies.length === fingers.length &&
        fingers.every((id) => bodies.includes(id)),
    };
  });

  const slider = page.locator("#gripper-slider");
  await slider.dispatchEvent("pointerdown");
  await slider.fill("0.5");
  expect(await readHighlight()).toEqual({ held: true, onlyFingers: true });
  await slider.dispatchEvent("pointerup");
  expect(await readHighlight()).toEqual({ held: false, onlyFingers: true });
  await page.waitForFunction(() => !panthera.viewer.highlight);

  const angle = page.locator("#gripper-angle");
  await angle.fill("1");
  await angle.press("Enter");
  expect(await readHighlight()).toEqual({ held: false, onlyFingers: true });

  for (const button of ["#gripper-close", "#gripper-open"]) {
    await page.waitForFunction(() => !panthera.viewer.highlight);
    await page.locator(button).click();
    expect(await readHighlight()).toEqual({ held: false, onlyFingers: true });
  }
});
