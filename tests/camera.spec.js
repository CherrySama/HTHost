import { test, expect } from "@playwright/test";

test("腕部相机按固定安装姿态渲染并随 J6 运动", async ({ page }) => {
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.goto("/");
  await expect(page.locator("#loading")).toBeHidden({ timeout: 60000 });
  await page.locator("#pause").click();
  const result = await page.evaluate(() => {
    const { simulation: sim, viewer } = window.panthera;
    const camera = sim.mj.mj_name2id(
      sim.model,
      sim.mj.mjtObj.mjOBJ_BODY.value,
      "camera_link",
    );
    const wrist = sim.mj.mj_name2id(
      sim.model,
      sim.mj.mjtObj.mjOBJ_BODY.value,
      "link6",
    );
    sim.reset();
    const before = Array.from(sim.data.xpos.slice(camera * 3, camera * 3 + 3));
    sim.setTarget(5, 0.35);
    sim.paused = false;
    for (let step = 0; step < 160; step++) sim.advance(0.025);
    sim.paused = true;
    viewer.render(sim.data);
    const after = Array.from(sim.data.xpos.slice(camera * 3, camera * 3 + 3));
    const meshItems = viewer.items.filter(
      ({ id }) => sim.model.geom_bodyid[id] === camera,
    );
    return {
      parent: sim.model.body_parentid[camera],
      wrist,
      mount: Array.from(sim.model.body_pos.slice(camera * 3, camera * 3 + 3)),
      mass: sim.model.body_mass[camera],
      visuals: meshItems.length,
      visible: meshItems.every(
        ({ mesh }) => mesh.visible && mesh.parent === viewer.scene,
      ),
      colliders: Array.from(sim.model.geom_bodyid).filter(
        (id, i) => id === camera && sim.model.geom_group[i] === 3,
      ).length,
      movement: Math.hypot(...after.map((value, i) => value - before[i])),
      actual: sim.data.qpos[sim.joints[5].q],
    };
  });
  expect(result.parent).toBe(result.wrist);
  expect(result.mass).toBeCloseTo(0.08, 8);
  expect(result.mount[0]).toBeCloseTo(0.0773830142568786, 8);
  expect(result.mount[1]).toBeCloseTo(0.00875000000001109, 8);
  expect(result.mount[2]).toBeCloseTo(0.0657518869921395, 8);
  expect(result.visuals).toBe(2);
  expect(result.visible).toBe(true);
  expect(result.colliders).toBeGreaterThan(0);
  expect(result.movement).toBeGreaterThan(0.01);
  expect(result.actual).toBeCloseTo(0.35, 2);
  expect(errors).toEqual([]);
});
