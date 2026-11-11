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

const formatValue = (value) =>
  (Math.abs(value) < 0.0005 ? 0 : value).toFixed(3);

export class MotorTelemetry {
  constructor(root, labels) {
    this.root = root;
    this.selected = 0;
    this.history = [];
    this.paused = false;
    root.innerHTML = `<div class="panel-heading"><div><div class="eyebrow">MOTOR TELEMETRY</div><h2>电机监控</h2></div><span class="data-source">仿真数据</span></div>
      <p class="monitor-intro">选择关节，查看最近 15 秒的运动与输出。</p>
      <label class="motor-picker" for="monitor-joint"><span>监控关节</span><select id="monitor-joint">${labels.map((label, index) => `<option value="${index}">J${index + 1} · ${label}</option>`).join("")}</select></label>
      <div class="monitor-state"><span id="monitor-state">实时采样</span><span>仿真时间 <output id="monitor-time">0.0 s</output></span></div>
      <div class="motor-charts">${CHARTS.map(
        (
          chart,
        ) => `<section class="motor-chart" data-chart="${chart.key}"><div class="chart-heading"><h3>${chart.title} <span>${chart.unit}</span></h3><output id="monitor-${chart.key}">—</output></div>
        ${chart.key === "position" ? '<div class="chart-legend"><span class="legend-actual">实际</span><span class="legend-target">目标 <output id="monitor-target">—</output> rad</span></div>' : ""}
        <canvas height="100" role="img"></canvas></section>`,
      ).join("")}</div>
      <p class="monitor-note">力矩为 MuJoCo 执行器输出，误差 = 目标 − 实际。<br>当前未连接真实电机。</p>`;
    this.select = root.querySelector("select");
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
    if (this.latest && sample.time < this.latest.time) this.history = [];
    const last = this.history.at(-1);
    this.latest = sample;
    this.paused = paused;
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

  render() {
    if (this.root.hidden || !this.latest) return;
    const joint = this.latest.joints[this.selected];
    this.root.querySelector("#monitor-state").textContent = this.paused
      ? "已暂停 · 历史保留"
      : "实时采样";
    this.root.querySelector("#monitor-time").textContent =
      `${this.latest.time.toFixed(1)} s`;
    this.root.querySelector("#monitor-target").textContent = formatValue(
      joint.target,
    );
    for (const chart of this.charts) {
      this.root.querySelector(`#monitor-${chart.key}`).textContent =
        formatValue(joint[chart.key]);
      chart.canvas.setAttribute(
        "aria-label",
        `J${this.selected + 1} ${chart.title}曲线（${chart.unit}）`,
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
    );
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
      samples.forEach((sample, index) => {
        const px = x(sample.time),
          py = y(sample.joints[this.selected][line.key]);
        if (index === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      if (samples.length === 1) {
        ctx.beginPath();
        ctx.arc(
          x(samples[0].time) - 1,
          y(samples[0].joints[this.selected][line.key]),
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
