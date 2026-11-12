import { test, expect } from "@playwright/test";

test("模型、六轴控制、限位与复位", async ({ page }) => {
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.goto("/");
  await expect(page.locator("#loading")).toBeHidden({ timeout: 60000 });
  await expect(page.getByRole("slider")).toHaveCount(7);
  await expect(page.locator(".unit")).toHaveText("rad");
  await expect(page.locator("#angle-0")).toHaveAttribute("step", "0.01");
  await page.locator("#angle-0").fill("0.25");
  await page.locator("#angle-0").press("Enter");
  await expect
    .poll(
      async () =>
        page.evaluate(() =>
          Math.abs(window.panthera.simulation.data.qpos[0] - 0.25),
        ),
      { timeout: 15000 },
    )
    .toBeLessThan(0.02);
  await expect(page.locator("#actual-0")).toContainText("rad");
  await page.getByRole("button", { name: /全部复位/ }).click();
  expect(await page.evaluate(() => window.panthera.viewer.items.length)).toBe(
    20,
  );

  // Advance the actual MuJoCo controller deterministically instead of relying on rendering speed.
  await page.evaluate(() => {
    window.panthera.simulation.paused = true;
  });
  for (let i = 0; i < 6; i++) {
    await page.getByRole("button", { name: /全部复位/ }).click();
    const input = page.locator(`#angle-${i}`);
    await input.fill("0.35");
    await input.press("Enter");
    const result = await page.evaluate((index) => {
      const sim = window.panthera.simulation;
      sim.paused = false;
      for (let step = 0; step < 160; step++) sim.advance(0.025);
      sim.paused = true;
      return {
        actual: sim.data.qpos[sim.joints[index].q],
        finite: Array.from(sim.data.qpos).every(Number.isFinite),
        time: sim.data.time,
      };
    }, i);
    expect(result.finite).toBe(true);
    expect(Math.abs(result.actual - 0.35)).toBeLessThan(0.02);
    expect(result.time).toBeGreaterThan(3.9);
  }

  await page.locator("#angle-0").fill("999");
  await page.locator("#angle-0").press("Tab");
  expect(
    await page.evaluate(() => {
      const sim = window.panthera.simulation;
      return sim.targets[0] === sim.joints[0].max;
    }),
  ).toBe(true);

  await page.locator("#angle-0").fill("-999");
  await page.locator("#angle-0").press("Tab");
  expect(
    await page.evaluate(() => {
      const sim = window.panthera.simulation;
      return sim.targets[0] === sim.joints[0].min;
    }),
  ).toBe(true);
  await page.locator("#slider-0").fill("0.5");
  expect(await page.evaluate(() => window.panthera.simulation.targets[0])).toBe(
    0.5,
  );

  await page.getByRole("button", { name: /全部复位/ }).click();
  expect(
    await page.evaluate(() => {
      const sim = window.panthera.simulation;
      return sim.joints.map(({ q }) => sim.data.qpos[q]);
    }),
  ).toEqual([0, 0, 0, 0, 0, 0]);
  await page.locator("#grid").click();
  await expect(page.locator("#grid")).toHaveAttribute("aria-pressed", "false");
  await page.locator("#home").click();
  await page.screenshot({ path: "preview.png" });
  expect(errors).toEqual([]);
});

test("窄屏不产生水平溢出", async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto("/");
  await expect(page.locator("#loading")).toBeHidden({ timeout: 60000 });
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBe(true);
  await expect(page.locator("#angle-5")).toBeVisible();
});
