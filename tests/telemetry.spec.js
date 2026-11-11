import { test, expect } from "@playwright/test";

test("电机曲线的实数据、关节切换、暂停及复位", async ({ page }) => {
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.goto("/");
  await expect(page.locator("#loading")).toBeHidden({ timeout: 60000 });
  await page.locator("#angle-0").fill("0.3");
  await page.locator("#angle-0").press("Enter");
  await page.locator("#monitor-tab").click();
  await expect(page.locator("#motor-monitor")).toBeVisible();
  await expect(page.locator(".data-source")).toHaveText("仿真数据");
  await expect(page.locator("#motor-monitor canvas")).toHaveCount(4);
  await expect
    .poll(() => page.evaluate(() => panthera.telemetry.history.length))
    .toBeGreaterThan(4);
  await expect
    .poll(() => page.locator("#monitor-position").textContent())
    .toBe("0.300");
  expect(
    await page.evaluate(() =>
      panthera.telemetry.history.some(
        (sample) => sample.joints[0].velocity > 0.02,
      ),
    ),
  ).toBe(true);
  await expect(page.locator("#monitor-target")).toHaveText("0.300");

  await page.locator("#control-tab").click();
  await page.locator("#angle-3").fill("-1.57");
  await page.locator("#angle-3").press("Enter");
  await page.locator("#monitor-tab").click();
  await page.locator("#monitor-joint").selectOption("3");
  await expect(page.locator("#monitor-target")).toHaveText("-1.570");
  await expect
    .poll(
      async () => Number(await page.locator("#monitor-error").textContent()),
      { timeout: 15000 },
    )
    .toBeLessThan(-0.5);
  await expect(page.locator('[data-chart="error"] canvas')).toHaveAttribute(
    "aria-label",
    "J4 位置误差曲线（rad）",
  );
  const actual = await page.evaluate(() => {
    const sim = panthera.simulation;
    const state = sim.getMotorTelemetry();
    const joint = sim.joints[3];
    return {
      source: state.source,
      torque: state.joints[3].torque,
      engineTorque: sim.data.qfrc_actuator[joint.v],
      error: state.joints[3].error,
      engineError: sim.targets[3] - sim.data.qpos[joint.q],
    };
  });
  expect(actual.source).toBe("simulation");
  expect(actual.torque).toBe(actual.engineTorque);
  expect(actual.error).toBe(actual.engineError);

  await page.locator("#control-tab").click();
  await page.locator("#pause").click();
  await page.locator("#monitor-tab").click();
  await expect(page.locator("#monitor-state")).toContainText("已暂停");
  const count = await page.evaluate(() => panthera.telemetry.history.length);
  await page.waitForTimeout(350);
  expect(await page.evaluate(() => panthera.telemetry.history.length)).toBe(
    count,
  );
  await page.locator("#monitor-joint").selectOption("0");
  await expect(page.locator("#monitor-target")).toHaveText("0.300");

  // Exercise bounded history with the real physics clock, including nonzero torques.
  const history = await page.evaluate(() => {
    const { simulation: sim, telemetry } = panthera;
    sim.paused = false;
    for (let i = 0; i < 800; i++) {
      sim.advance(0.025);
      telemetry.update(sim.getMotorTelemetry(), false);
    }
    sim.paused = true;
    telemetry.update(sim.getMotorTelemetry(), true);
    return {
      count: telemetry.history.length,
      span: telemetry.history.at(-1).time - telemetry.history[0].time,
      error: telemetry.history.at(-1).joints[3].error,
    };
  });
  expect(history.count).toBeLessThanOrEqual(151);
  expect(history.span).toBeLessThanOrEqual(15.001);
  expect(history.error).toBeLessThan(-0.5);

  await page.locator("#control-tab").click();
  await page.locator("#reset").click();
  await page.locator("#monitor-tab").click();
  await expect(page.locator("#monitor-time")).toHaveText("0.0 s");
  await expect(page.locator("#monitor-position")).toHaveText("0.000");
  expect(await page.evaluate(() => panthera.telemetry.history.length)).toBe(0);
  expect(errors).toEqual([]);
});

test("窄屏曲线布局与键盘面板切换", async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto("/");
  await expect(page.locator("#loading")).toBeHidden({ timeout: 60000 });
  await page.locator("#control-tab").focus();
  await page.keyboard.press("ArrowRight");
  await expect(page.locator("#monitor-tab")).toHaveAttribute(
    "aria-selected",
    "true",
  );
  await expect(page.locator("#monitor-joint")).toBeVisible();
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBe(true);
});
