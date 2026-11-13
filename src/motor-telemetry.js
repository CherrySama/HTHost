const WINDOW_SECONDS = 15;
const SAMPLE_INTERVAL = 0.1;
const CHARTS = [
  { key: "position", title: "位置", unit: "rad", color: "#0071e3", range: 0.1 },
  {
    key: "velocity",
    title: "速度",
    unit: "rad/s",
    color: "#248a66",
    range: 0.2,
  },
  {
    key: "torque",
    title: "执行器力矩",
    unit: "N·m",
    color: "#8054c7",
    range: 1,
  },
  {
    key: "error",
    title: "位置误差",
    unit: "rad",
    color: "#c16e20",
    range: 0.02,
  },
];

const formatValue = (value) => Number.isFinite(value)
  ? (Math.abs(value) < 0.0005 ? 0 : value).toFixed(3)
  : "—";

export class MotorTelemetry {
  constructor(root, labels, writeZero) {
    this.root = root;
    this.selected = 0;
    this.history = [];
    this.paused = false;
    root.innerHTML = `<div class="panel-heading"><div><div class="eyebrow">MOTOR TELEMETRY</div><h2>电机监控</h2></div><span class="data-source">仿真数据</span></div>
      <section class="motor-overview" hidden>
        <table aria-label="实时电机状态">
          <thead><tr><th scope="col">电机</th><th scope="col">位置<br><small>rad</small></th><th scope="col">速度<br><small>rad/s</small></th><th scope="col">力矩<br><small>N·m</small></th><th scope="col">状态</th></tr></thead>
          <tbody>${[...labels.map((_, i) => "J" + (i + 1)), "夹爪"].map((name, i) => `<tr>
            <th scope="row"><button type="button" data-motor="${i}">${name}</button></th>
            <td></td><td></td><td></td><td></td></tr>`).join("")}</tbody>
        </table>
        <details class="motor-calibration">
          <summary>零位校准</summary>
          <p>核对表中位置，将六轴摆到机械零位，已安装的夹爪摆到闭合位置。</p>
          <button type="button" id="monitor-write-zero" disabled>写入零位</button>
          <p id="monitor-calibration-status" role="status"></p>
        </details>
      </section>
      <p class="monitor-intro">选择电机，查看最近 15 秒的运动与输出。 <span id="monitor-drive" hidden></span></p>
      <label class="motor-picker" for="monitor-joint"><span>监控电机</span><select id="monitor-joint">${labels.map((label, index) => `<option value="${index}">J${index + 1} · ${label}</option>`).join("")}</select></label>
      <div class="monitor-state"><span id="monitor-state">实时采样</span><span><span id="monitor-clock">仿真时间</span> <output id="monitor-time">0.0 s</output></span></div>
      <div class="motor-charts">${CHARTS.map(
        (
          chart,
        ) => `<section class="motor-chart" data-chart="${chart.key}"><div class="chart-heading"><h3>${chart.title} <span>${chart.unit}</span></h3><output id="monitor-${chart.key}">—</output></div>
        ${chart.key === "position" ? '<div class="chart-legend"><span class="legend-actual">实际</span><span class="legend-target">目标 <output id="monitor-target">—</output> rad</span></div>' : ""}
        <canvas height="100" role="img"></canvas></section>`,
      ).join("")}</div>
      <p class="monitor-note">力矩为 MuJoCo 执行器输出，误差 = 目标 − 实际。<br>当前未连接真实电机。</p>`;
    this.select = root.querySelector("select");
    this.armCount = labels.length;
    this.overview = root.querySelector(".motor-overview");
    this.rows = [...this.overview.querySelectorAll("tbody tr")];
    this.writeButton = root.querySelector("#monitor-write-zero");
    this.calibrationStatus = root.querySelector("#monitor-calibration-status");
    this.drive = root.querySelector("#monitor-drive");
    this.calibrationPanel = root.querySelector(".motor-calibration");
    this.calibrationPanel.ontoggle = () => this.render();
    this.rows.forEach((row, i) => {
      row.querySelector("button").onclick = () => {
        this.select.value = i;
        this.selected = i;
        this.render();
      };
    });
    this.writeButton.onclick = async () => {
      if (this.writeButton.disabled) return;
      const positions = this.latest.joints.map((joint) => joint.position);
      if (!window.confirm("将当前姿态写为六轴及已安装夹爪的新零位。请确认已核对姿态并支撑好机械臂。确定写入吗？")) return;
      const previous = this.calibration;
      this.zeroSubmitting = true;
      this.zeroRequestError = "";
      this.renderCalibration();
      try {
        const result = await writeZero(positions);
        if (this.calibration === previous || this.calibration?.ready) this.calibration = result;
      } catch (error) {
        this.zeroRequestError = error.message;
      } finally {
        this.zeroSubmitting = false;
        this.renderCalibration();
      }
    };
    this.select.addEventListener("change", () => {
      this.selected = Number(this.select.value);
      this.render();
    });
    this.charts = CHARTS.map((config) => ({
      ...config,
      canvas: root.querySelector(`[data-chart="${config.key}"] canvas`),
    }));
    this.observer = new ResizeObserver(() => this.render());
    this.observer.observe(root);
  }

  update(sample, paused) {
    if (this.latest && (
      sample.source !== this.latest.source ||
      sample.session !== this.latest.session ||
      sample.time < this.latest.time
    )) {
      this.history = [];
      this.calibration = null;
      this.zeroRequestError = "";
    }
    const gripperOption = this.select.options[this.armCount];
    if (sample.joints.length > this.armCount && !gripperOption) {
      this.select.add(new Option("夹爪", this.armCount));
    } else if (sample.joints.length <= this.armCount && gripperOption) {
      gripperOption.remove();
      if (this.selected >= this.armCount) {
        this.selected = 0;
        this.select.value = "0";
      }
    }
    const last = this.history.at(-1);
    this.latest = sample;
    this.paused = paused;
    this.disconnected = false;
    if (
      !paused &&
      (!last || sample.time - last.time >= SAMPLE_INTERVAL - 1e-6)
    ) {
      this.history.push(sample);
      while (
        this.history.length &&
        this.history[0].time < sample.time - WINDOW_SECONDS
      )
        this.history.shift();
      // Bound memory even if a future source sends timestamps at a higher rate.
      if (this.history.length > 151)
        this.history.splice(0, this.history.length - 151);
    }
    this.render();
  }

  reset(sample) {
    this.history = [];
    this.latest = sample;
    this.render();
  }

  markDisconnected() {
    if (this.latest?.source !== "robot") return;
    this.disconnected = true;
    this.render();
  }

  updateCalibration(calibration) {
    if (calibration?.phase !== this.calibration?.phase) this.zeroRequestError = "";
    this.calibration = calibration;
    this.render();
  }

  renderCalibration() {
    const result = this.calibration;
    this.writeButton.disabled = this.disconnected || this.zeroSubmitting || !result?.ready;
    const messages = {
      queued: "正在等待写入…", writing: "正在写入零位…", verifying: "正在核对电机反馈…",
      done: "全部电机反馈已通过零位验证。",
      rejected: "姿态或电机状态已变化，本次未写入。",
      unverified: "零位结果未确认，请核对各电机反馈。",
    };
    const message = this.zeroRequestError || (this.zeroSubmitting ? messages.queued
      : messages[result?.phase] || (result?.ready ? "" : "请在位置模式下等待电机停稳。"));
    if (this.calibrationStatus.textContent !== message) this.calibrationStatus.textContent = message;
  }

  render() {
    if (this.root.hidden || !this.latest) return;
    const joint = this.latest.joints[this.selected];
    const live = this.latest.source === "robot";
    const zeroPausesSampling = live && !this.disconnected &&
      ["queued", "writing"].includes(this.calibration?.phase);
    this.overview.hidden = !live;
    this.drive.hidden = !live;
    if (live) {
      this.drive.textContent = "驱动模式 " + (joint.driveMode ?? "—");
      const result = this.calibration;
      this.rows.forEach((row, i) => {
        const motor = this.latest.joints[i];
        row.hidden = !motor;
        if (!motor) return;
        const position = result?.positions?.[i];
        const checked = this.calibrationPanel.open && ["done", "unverified"].includes(result?.phase);
        const failed = checked && (!Number.isFinite(position) || Math.abs(position) > result.tolerance);
        const fault = Number.isInteger(motor.fault) && motor.fault !== 0;
        const state = zeroPausesSampling ? "已暂停"
          : fault ? "0x" + motor.fault.toString(16).toUpperCase().padStart(2, "0")
          : this.disconnected && !checked ? "已失联"
          : checked ? failed ? "未确认" : "零位已验" : Number.isInteger(motor.fault) ? "正常" : "—";
        row.classList.toggle("motor-fault", fault || failed);
        row.querySelector("button").setAttribute("aria-pressed", String(i === this.selected));
        row.cells[4].title = checked ? "校准反馈 " + formatValue(position) + " rad" : "";
        [motor.position, motor.velocity, motor.torque].map(formatValue)
          .concat(state).forEach((value, j) => {
            if (row.cells[j + 1].textContent !== value) row.cells[j + 1].textContent = value;
          });
      });
    }
    this.renderCalibration();
    this.root.querySelector(".data-source").textContent = zeroPausesSampling
      ? "采样暂停" : live ? "真机数据" : "仿真数据";
    this.root.querySelector("#monitor-clock").textContent = live ? "连接时长" : "仿真时间";
    this.root.querySelector(".monitor-note").textContent = live
      ? "力矩来自电机反馈。位置误差 = 已下发目标 − 实际角度。"
      : "力矩为 MuJoCo 执行器输出，误差 = 目标 − 实际。";
    this.root.querySelector("#monitor-state").textContent = this.disconnected
      ? "已失联 · 历史保留"
      : zeroPausesSampling
      ? "采样暂停 · 保留写入前反馈"
      : live && this.calibration?.phase === "verifying"
      ? "已恢复采样 · 正在验证零位"
      : this.paused
      ? "已暂停 · 历史保留"
      : "实时采样";
    this.root.querySelector("#monitor-time").textContent =
      `${this.latest.time.toFixed(1)} s`;
    this.root.querySelector("#monitor-target").textContent = formatValue(
      joint.target,
    );
    for (const chart of this.charts) {
      const title = live && chart.key === "torque" ? "电机反馈力矩" : chart.title;
      this.root.querySelector(`[data-chart="${chart.key}"] h3`).firstChild.nodeValue = `${title} `;
      this.root.querySelector(`#monitor-${chart.key}`).textContent =
        formatValue(joint[chart.key]);
      chart.canvas.setAttribute(
        "aria-label",
        `${joint.name} ${title}曲线（${chart.unit}）`,
      );
      this.draw(chart);
    }
  }

  draw(chart) {
    const canvas = chart.canvas;
    const width = canvas.clientWidth;
    if (!width) return;
    const height = 100;
    const ratio = Math.min(devicePixelRatio || 1, 2);
    if (
      canvas.width !== Math.round(width * ratio) ||
      canvas.height !== height * ratio
    ) {
      canvas.width = Math.round(width * ratio);
      canvas.height = height * ratio;
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, width, height);
    const samples = this.history.filter(
      (sample) => sample.time >= this.latest.time - WINDOW_SECONDS,
    );
    const values = samples.flatMap((sample) =>
      chart.key === "position"
        ? [
            sample.joints[this.selected].position,
            sample.joints[this.selected].target,
          ]
        : [sample.joints[this.selected][chart.key]],
    ).filter(Number.isFinite);
    let low = Math.min(0, ...values),
      high = Math.max(0, ...values);
    const span = Math.max(high - low, chart.range);
    const middle = (high + low) / 2;
    low = middle - span * 0.6;
    high = middle + span * 0.6;
    const plot = { left: 41, top: 9, right: width - 5, bottom: 78 };
    const y = (value) =>
      plot.bottom - ((value - low) / (high - low)) * (plot.bottom - plot.top);
    const end = this.latest.time;
    const x = (time) =>
      plot.right - ((end - time) / WINDOW_SECONDS) * (plot.right - plot.left);
    ctx.font = "9px -apple-system, sans-serif";
    ctx.fillStyle = "#909099";
    ctx.textAlign = "right";
    ctx.fillText(formatValue(high), plot.left - 6, plot.top + 3);
    ctx.fillText(formatValue(low), plot.left - 6, plot.bottom + 3);
    ctx.strokeStyle = "#e9e9ef";
    ctx.lineWidth = 1;
    ctx.setLineDash([]);
    for (const line of [plot.top, y(0), plot.bottom]) {
      ctx.beginPath();
      ctx.moveTo(plot.left, line);
      ctx.lineTo(plot.right, line);
      ctx.stroke();
    }
    ctx.textAlign = "left";
    ctx.fillText("−15 s", plot.left, 96);
    ctx.textAlign = "right";
    ctx.fillText("0 s", plot.right, 96);
    ctx.save();
    ctx.beginPath();
    ctx.rect(
      plot.left,
      plot.top,
      plot.right - plot.left,
      plot.bottom - plot.top,
    );
    ctx.clip();
    const series =
      chart.key === "position"
        ? [
            { key: "target", color: "#a0a0ab", dashed: true },
            { key: "position", color: chart.color },
          ]
        : [{ key: chart.key, color: chart.color }];
    for (const line of series) {
      ctx.strokeStyle = line.color;
      ctx.fillStyle = line.color;
      ctx.lineWidth = 1.6;
      ctx.setLineDash(line.dashed ? [4, 3] : []);
      ctx.beginPath();
      let previousValid = false;
      let points = 0;
      let lastPoint;
      for (const sample of samples) {
        const value = sample.joints[this.selected][line.key];
        if (!Number.isFinite(value)) {
          previousValid = false;
          continue;
        }
        lastPoint = [x(sample.time), y(value)];
        if (previousValid) ctx.lineTo(...lastPoint);
        else ctx.moveTo(...lastPoint);
        previousValid = true;
        points++;
      }
      ctx.stroke();
      if (points === 1) {
        ctx.beginPath();
        ctx.arc(
          lastPoint[0] - 1,
          lastPoint[1],
          2,
          0,
          Math.PI * 2,
        );
        ctx.fill();
      }
    }
    ctx.restore();
  }
}
