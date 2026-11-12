import { expect, test } from "@playwright/test";

const layers = [
  ["collision", "碰撞"],
  ["inertia", "惯量"],
  ["centerOfMass", "质心"],
  ["bodyAxes", "坐标轴"],
  ["jointAxes", "关节轴"],
];

test("视图开关独立切换，显示层跟随 MuJoCo 姿态", async ({ page }) => {
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.goto("/");
  await expect(page.locator("#loading")).toBeHidden({ timeout: 60000 });
  await page.evaluate(() => {
    window.panthera.simulation.paused = true;
  });

  const modelAppearance = () => page.evaluate(() => {
    const { items } = panthera.viewer;
    return items.map(({ mesh }) => ({
      transparent: mesh.material.transparent,
      opacity: mesh.material.opacity,
      depthWrite: mesh.material.depthWrite,
      castShadow: mesh.castShadow,
    }));
  });
  const opaqueAppearance = await modelAppearance();
  const expectTranslucentModel = async () => {
    expect((await modelAppearance()).every((item) =>
      item.transparent && item.opacity > 0 && item.opacity < 1,
    )).toBe(true);
  };

  expect(await page.evaluate(() => panthera.viewer.overlays.size)).toBe(0);
  const overlayButton = (label) => page.getByRole("button", { name: label, exact: true });
  for (const [name, label] of layers) {
    const button = overlayButton(label);
    await expect(button).toHaveAttribute("aria-pressed", "false");
    await button.click();
    await expect(button).toHaveAttribute("aria-pressed", "true");
    if (name === "collision" || name === "inertia") {
      expect(await modelAppearance()).toEqual(opaqueAppearance);
    } else {
      await expectTranslucentModel();
    }
  }

  const collision = page.getByRole("button", { name: "碰撞", exact: true });
  await collision.click();
  expect(await page.evaluate(() =>
    [...panthera.viewer.overlays].map(([name, group]) => [name, group.visible]),
  )).toEqual(layers.map(([name]) => [name, name !== "collision"]));
  await collision.click();

  const previousFingerPosition = await page.evaluate(() => {
    const { simulation: sim, viewer } = panthera;
    const finger = sim.gripperJoints[0];
    const bodyId = sim.model.jnt_bodyid[finger.id];
    const axes = viewer.overlays.get("bodyAxes");
    const marker = axes.children.find((item) => item.userData.bodyId === bodyId);
    const previousPosition = marker.position.toArray();
    sim.data.qpos[finger.q] = -0.02;
    sim.mj.mj_forward(sim.model, sim.data);
    return previousPosition;
  });

  await expect.poll(() => page.evaluate((previousPosition) => {
    const { simulation: sim, viewer } = panthera;
    const id = sim.model.jnt_bodyid[sim.gripperJoints[0].id];
    const marker = viewer.overlays.get("bodyAxes").children.find(
      (item) => item.userData.bodyId === id,
    );
    return marker.position.toArray().some(
      (value, index) => Math.abs(value - previousPosition[index]) > 0.01,
    );
  }, previousFingerPosition)).toBe(true);

  await page.evaluate(() => {
    const sim = panthera.simulation;
    sim.data.qpos[sim.joints[1].q] = 0.4;
    sim.mj.mj_forward(sim.model, sim.data);
  });

  await expect.poll(() => page.evaluate(() => {
    const { simulation: sim, viewer } = panthera;
    const { data, model } = sim;
    const near = (actual, expected) => actual.every(
      (value, index) => Math.abs(value - expected[index]) < 1e-6,
    );
    const at = (values, id, size = 3) => values.subarray(id * size, (id + 1) * size);
    const markersMatch = (name, count, matches) => {
      const { children } = viewer.overlays.get(name);
      return children.length === count && children.every(matches);
    };
    const bodiesWithMass = model.body_mass.filter((mass, id) => id > 0 && mass > 0).length;
    const collisionBodies = new Set();
    for (let id = 0; id < model.ngeom; id++) {
      if (model.geom_group[id] === 3) collisionBodies.add(model.geom_bodyid[id]);
    }
    const bodyPositionMatches = (item) => near(
      item.position.toArray(), at(data.xpos, item.userData.bodyId),
    );
    const centerMatches = (item) => near(
      item.position.toArray(), at(data.xipos, item.userData.bodyId),
    );
    const armJointIds = new Set(sim.joints.map((joint) => joint.id));
    return {
      collision: markersMatch("collision", collisionBodies.size, bodyPositionMatches),
      inertia: markersMatch("inertia", bodiesWithMass, (item) => {
        const id = item.userData.bodyId;
        const [x, y, z] = item.scale.toArray().map((radius) => radius ** 2);
        const massFactor = model.body_mass[id] / 5;
        return centerMatches(item) && near(
          [y + z, z + x, x + y].map((value) => value * massFactor),
          at(model.body_inertia, id),
        );
      }),
      centers: markersMatch("centerOfMass", bodiesWithMass, centerMatches),
      bodyAxes: markersMatch("bodyAxes", model.nbody - 1, (item) => {
        const [w, x, y, z] = at(data.xquat, item.userData.bodyId, 4);
        return bodyPositionMatches(item) && near(item.quaternion.toArray(), [x, y, z, w]);
      }),
      jointAxes: markersMatch("jointAxes", armJointIds.size, (arrow) => {
        const id = arrow.userData.jointId;
        if (!armJointIds.delete(id) || arrow.children.length !== 4) return false;
        const direction = arrow.position.clone().set(0, 1, 0).applyQuaternion(arrow.quaternion);
        const tip = arrow.children[3];
        const tangent = direction.clone().set(0, 1, 0).applyQuaternion(tip.quaternion);
        return tip.position.clone().cross(tangent).y > 0 &&
          near(arrow.position.toArray(), at(data.xanchor, id)) &&
          near(direction.toArray(), at(data.xaxis, id));
      }),
    };
  })).toEqual({
    collision: true, inertia: true, centers: true, bodyAxes: true, jointAxes: true,
  });

  await page.locator("#slider-5").dispatchEvent("pointerdown");
  expect(await page.evaluate(() => {
    const { simulation, viewer } = panthera;
    return viewer.highlight?.jointId === simulation.joints[5].id;
  })).toBe(true);
  await page.locator("#slider-5").dispatchEvent("pointerup");

  for (const label of ["质心", "坐标轴"]) {
    await overlayButton(label).click();
    await expectTranslucentModel();
  }
  await overlayButton("关节轴").click();
  expect(await modelAppearance()).toEqual(opaqueAppearance);
  for (const label of ["质心", "坐标轴", "关节轴"]) {
    await overlayButton(label).click();
    await expectTranslucentModel();
    await overlayButton(label).click();
    expect(await modelAppearance()).toEqual(opaqueAppearance);
  }
  expect(errors).toEqual([]);
});

test("窄屏能完整显示视图工具栏", async ({ page }) => {
  await page.setViewportSize({ width: 320, height: 800 });
  await page.goto("/");
  await expect(page.locator("#loading")).toBeHidden({ timeout: 60000 });
  const toolbar = page.getByRole("group", { name: "视图显示" });
  await expect(toolbar.getByRole("button")).toHaveCount(7);
  expect(await toolbar.evaluate((element) => {
    const viewport = element.closest("#viewport").getBoundingClientRect();
    return Array.from(element.querySelectorAll("button")).every((button) => {
      const rect = button.getBoundingClientRect();
      return rect.left >= viewport.left && rect.right <= viewport.right &&
        rect.top >= viewport.top && rect.bottom <= viewport.bottom;
    });
  })).toBe(true);
});
