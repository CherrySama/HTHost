import { test, expect } from "@playwright/test";

test("自碰撞物理阻挡、零位稳定和反向脱离", async ({ page }) => {
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.goto("/");
  await expect(page.locator("#loading")).toBeHidden({ timeout: 60000 });
  await page.locator("#pause").click();
  const result = await page.evaluate(() => {
    const sim = window.panthera.simulation;
    const run = (seconds) => {
      let penetration = 0;
      sim.paused = false;
      for (let step = 0; step < seconds * 40; step++) {
        sim.advance(0.025);
        for (const contact of sim.getSelfContacts())
          penetration = Math.max(penetration, -contact.distance);
      }
      sim.paused = true;
      return {
        actual: Array.from(sim.data.qpos),
        contacts: sim.getSelfContacts().length,
        penetration,
        velocity: Math.max(...Array.from(sim.data.qvel).map(Math.abs)),
      };
    };
    sim.reset();
    const home = run(2);
    sim.setTarget(3, -1.57);
    const blocked = run(5);
    const target = sim.targets[3];
    sim.setTarget(3, 0);
    const released = run(5);

    // Control experiment: remove only self-contact filtering and keep floor contact.
    const saved = sim.model.geom_conaffinity.slice();
    for (let i = 0; i < sim.model.ngeom; i++) {
      if (sim.model.geom_group[i] === 3) sim.model.geom_conaffinity[i] = 1;
    }
    sim.reset();
    sim.setTarget(3, -1.57);
    const withoutSelfContact = run(5);
    sim.model.geom_conaffinity.set(saved);
    sim.reset();
    return { home, blocked, released, withoutSelfContact, target };
  });
  expect(result.home.contacts).toBe(0);
  expect(Math.max(...result.home.actual.map(Math.abs))).toBeLessThan(0.003);
  expect(result.target).toBe(-1.57);
  expect(result.blocked.contacts).toBeGreaterThan(0);
  expect(result.blocked.actual[3]).toBeGreaterThan(-1.25);
  expect(result.blocked.penetration).toBeLessThan(0.002);
  expect(result.blocked.velocity).toBeLessThan(0.03);
  expect(result.released.contacts).toBe(0);
  expect(Math.max(...result.released.actual.map(Math.abs))).toBeLessThan(0.02);
  expect(result.withoutSelfContact.contacts).toBe(0);
  expect(result.withoutSelfContact.actual[3]).toBeLessThan(
    result.blocked.actual[3] - 0.3,
  );
  console.log("WASM 自碰撞验证", JSON.stringify(result));
  expect(errors).toEqual([]);
});
