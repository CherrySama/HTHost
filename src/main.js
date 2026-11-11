import "./style.css";
import { Simulation } from "./simulation.js";
import { Viewer } from "./viewer.js";
import { MotorTelemetry } from "./motor-telemetry.js";

document.querySelector("#app").innerHTML = `
  <header><a class="brand" href="/" aria-label="Panthera Studio 首页"><span class="brand-icon">P</span>Panthera <span>Studio</span></a>
    <span class="mode"><i></i> 本地仿真</span><span class="engine">MuJoCo · WebAssembly</span></header>
  <main><section id="viewport" aria-label="机械臂三维视图">
    <div class="scene-heading"><div class="eyebrow">YOUR ROBOT. YOUR SPACE.</div><h1>探索每一个自由度。</h1><p>Panthera HT <span> / </span> 六轴机械臂</p></div>
    <div class="view-tools" role="group" aria-label="视图显示">
      <button id="home" title="恢复默认视角">⌖ <span>默认视角</span></button><button id="grid" aria-pressed="true">▦ <span>网格</span></button>
      <button data-overlay="collision" aria-pressed="false" disabled>碰撞</button>
      <button data-overlay="inertia" aria-pressed="false" disabled>惯量</button>
      <button data-overlay="centerOfMass" aria-pressed="false" disabled>质心</button>
      <button data-overlay="bodyAxes" aria-pressed="false" disabled>坐标轴</button>
      <button data-overlay="jointAxes" aria-pressed="false" disabled>关节轴</button>
    </div>
    <div id="loading" role="status"><span class="spinner"></span><span id="load-message">正在准备工作空间…</span></div>
    <div class="scene-footer"><span><i class="live-dot"></i><span id="status">正在加载</span></span><span>拖动旋转 · 滚轮缩放 · 右键平移</span></div>
  </section>
  <aside><nav class="sidebar-tabs" role="tablist" aria-label="侧边面板"><button id="control-tab" role="tab" aria-selected="true" aria-controls="control-panel">关节控制</button><button id="monitor-tab" role="tab" aria-selected="false" aria-controls="motor-monitor" tabindex="-1" disabled>电机监控</button></nav>
    <section class="sidebar-panel" id="control-panel" role="tabpanel" aria-labelledby="control-tab"><div class="panel-heading"><div><div class="eyebrow">JOINT CONTROL</div><h2>关节控制</h2></div><span class="unit">rad</span></div>
    <p class="intro">调整目标角度，探索机械臂的运动。</p>
    <div class="column-key"><span>关节 / 目标角度</span><span>实际角度</span></div>
    <div id="joints"></div>
    <section class="joint" aria-label="夹爪控制"><div class="joint-heading"><label for="gripper-angle"><b>夹爪</b><span>电机目标 · rad</span></label><output id="gripper-opening">82.00 mm</output></div>
    <div class="joint-input"><input id="gripper-slider" type="range" aria-label="夹爪电机角度滑条（rad）" min="0" max="2" step="0.01" value="2"><div class="number-wrap"><input id="gripper-angle" type="number" aria-label="夹爪电机目标（rad）" min="0" max="2" step="0.01" value="2"></div></div>
    <div class="limits"><span>闭合 · 0 rad</span><span>张开 · 2 rad</span></div><p class="gripper-summary" id="gripper-target-opening">目标总开口 82.00 mm</p>
    <div class="gripper-buttons"><button id="gripper-close" disabled>闭合</button><button id="gripper-open" disabled>张开</button></div></section>
    <div class="actions"><button id="reset" disabled>↺　全部复位</button><button id="pause" disabled aria-pressed="false">暂停</button></div>
    <div class="info"><span class="info-icon">◎</span><div><strong>在虚拟空间，自由探索。</strong><p>自碰撞物理阻挡 · 关节限位保护<br>仅控制虚拟机械臂</p></div></div>
    </section><section class="sidebar-panel" id="motor-monitor" role="tabpanel" aria-labelledby="monitor-tab" hidden></section>
    <div class="panel-bottom"><span>Panthera HT</span><span>6 DOF <b>·</b> V1.1.0</span></div>
  </aside></main>`;

const simulation = new Simulation();
let viewer;
let telemetry;
let ready = false;
const controls = [];
const labels = [
  "基座旋转",
  "肩部俯仰",
  "肘部俯仰",
  "腕部旋转",
  "腕部俯仰",
  "末端旋转",
];

function buildControls() {
  simulation.joints.forEach((joint, i) => {
    const row = document.createElement("section");
    row.className = "joint";
    const min = joint.min,
      max = joint.max;
    // Align the slider's 0.01-rad steps to zero, within the model's exact limits.
    const sliderMin = Math.ceil(min * 100) / 100;
    const sliderMax = Math.floor(max * 100) / 100;
    row.innerHTML = `<div class="joint-heading"><label for="angle-${i}"><b>J${i + 1}</b><span>${labels[i]}</span></label><output id="actual-${i}" aria-label="J${i + 1} 实际角度（rad）">0.00 rad</output></div>
      <div class="joint-input"><input type="range" id="slider-${i}" aria-label="J${i + 1} 目标角度滑条（rad）" min="${sliderMin}" max="${sliderMax}" step="0.01" value="0"><div class="number-wrap"><input type="number" id="angle-${i}" aria-label="J${i + 1} 目标角度（rad）" min="${min}" max="${max}" step="0.01" value="0.00"></div></div>
      <div class="limits"><span>${min.toFixed(5)} rad</span><span>${max.toFixed(5)} rad</span></div>`;
    document.querySelector("#joints").appendChild(row);
    const slider = row.querySelector("[type=range]"),
      number = row.querySelector("[type=number]");
    const update = (value) => {
      simulation.setTarget(i, value);
      const radians = simulation.targets[i];
      slider.value = radians;
      number.value = String(Number(radians.toFixed(5)));
      slider.style.setProperty(
        "--fill",
        `${((Number(slider.value) - sliderMin) / (sliderMax - sliderMin)) * 100}%`,
      );
    };
    slider.addEventListener("pointerdown", () =>
      viewer.highlightJoint(joint.id, true),
    );
    slider.addEventListener("input", () => {
      update(Number(slider.value));
      viewer.highlightJoint(joint.id);
    });
    number.addEventListener("change", () => {
      update(
        number.value === "" ? simulation.targets[i] : Number(number.value),
      );
      viewer.highlightJoint(joint.id);
    });
    number.addEventListener("keydown", (event) => {
      if (event.key === "Enter") number.blur();
    });
    update(0);
    controls.push({ update, actual: row.querySelector("output") });
  });
}

function updateGripper(value) {
  simulation.setGripperTarget(value);
  const state = simulation.getGripperState();
  document.querySelector("#gripper-slider").value = state.target;
  document
    .querySelector("#gripper-slider")
    .style.setProperty("--fill", `${(state.target / 2) * 100}%`);
  document.querySelector("#gripper-angle").value = state.target.toFixed(2);
  document.querySelector("#gripper-target-opening").textContent =
    `目标总开口 ${state.targetOpening.toFixed(2)} mm`;
}
function applyGripperTarget(value) {
  if (!ready) return;
  updateGripper(value);
  viewer.highlightJoint(simulation.gripperHighlightJointId);
}

document.querySelector("#gripper-slider").onpointerdown = () => {
  if (ready) viewer.highlightJoint(simulation.gripperHighlightJointId, true);
};
document.querySelector("#gripper-slider").oninput = (event) => {
  applyGripperTarget(Number(event.target.value));
};
document.querySelector("#gripper-angle").onchange = (event) => {
  applyGripperTarget(
    event.target.value === ""
      ? simulation.gripperTarget
      : Number(event.target.value),
  );
};
document.querySelector("#gripper-angle").onkeydown = (event) => {
  if (event.key === "Enter") event.target.blur();
};
document.querySelector("#gripper-close").onclick = () => applyGripperTarget(0);
document.querySelector("#gripper-open").onclick = () => applyGripperTarget(2);

document.querySelector("#home").onclick = () => viewer?.home();
document.querySelector("#grid").onclick = (event) => {
  if (!viewer) return;
  viewer.grid.visible = !viewer.grid.visible;
  viewer.dirty = true;
  event.currentTarget.setAttribute("aria-pressed", String(viewer.grid.visible));
};
const overlayButtons = document.querySelectorAll(".view-tools [data-overlay]");
overlayButtons.forEach((button) => {
  button.onclick = () => {
    const visible = button.getAttribute("aria-pressed") !== "true";
    viewer.setOverlayVisible(button.dataset.overlay, visible);
    button.setAttribute("aria-pressed", String(visible));
  };
});
document.querySelector("#reset").onclick = () => {
  viewer.clearJointHighlight();
  simulation.reset();
  controls.forEach((control) => control.update(0));
  telemetry.reset(simulation.getMotorTelemetry());
  updateGripper(2);
};
window.addEventListener("pointerup", () => viewer?.endJointInteraction());
window.addEventListener("pointercancel", () => viewer?.endJointInteraction());
window.addEventListener("blur", () => viewer?.endJointInteraction());

const tabs = [...document.querySelectorAll('.sidebar-tabs [role="tab"]')];
function selectPanel(tab) {
  if (tab.disabled) return;
  for (const item of tabs) {
    const selected = item === tab;
    item.setAttribute("aria-selected", String(selected));
    item.tabIndex = selected ? 0 : -1;
    document.getElementById(item.getAttribute("aria-controls")).hidden =
      !selected;
  }
  document.querySelector("aside").scrollTop = 0;
  telemetry?.render();
}
tabs.forEach((tab, index) => {
  tab.onclick = () => selectPanel(tab);
  tab.onkeydown = (event) => {
    if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
    event.preventDefault();
    const next =
      event.key === "Home"
        ? tabs[0]
        : event.key === "End"
          ? tabs.at(-1)
          : tabs[
              (index + (event.key === "ArrowLeft" ? -1 : 1) + tabs.length) %
                tabs.length
            ];
    if (!next.disabled) {
      selectPanel(next);
      next.focus();
    }
  };
});
document.querySelector("#pause").onclick = (event) => {
  simulation.paused = !simulation.paused;
  event.currentTarget.textContent = simulation.paused ? "继续" : "暂停";
  event.currentTarget.setAttribute("aria-pressed", String(simulation.paused));
};

try {
  viewer = new Viewer(document.querySelector("#viewport"));
  await simulation.init((message) => {
    document.querySelector("#load-message").textContent = message;
  });
  viewer.load(
    simulation.model,
    simulation.highlightBodies,
    simulation.mj.mjtJoint.mjJNT_HINGE.value,
  );
  overlayButtons.forEach((button) => (button.disabled = false));
  buildControls();
  updateGripper(2);
  document.querySelector("#gripper-close").disabled = false;
  document.querySelector("#gripper-open").disabled = false;
  telemetry = new MotorTelemetry(
    document.querySelector("#motor-monitor"),
    labels,
  );
  telemetry.update(simulation.getMotorTelemetry(), simulation.paused);
  document.querySelector("#monitor-tab").disabled = false;
  document.querySelector("#loading").hidden = true;
  document.querySelector("#reset").disabled = false;
  document.querySelector("#pause").disabled = false;
  ready = true;
  // Diagnostic access for browser verification.
  window.panthera = { simulation, viewer, telemetry };
} catch (error) {
  console.error(error);
  document.querySelector("#loading").classList.add("error");
  document.querySelector("#load-message").textContent =
    `加载失败：${error.message}。请刷新重试。`;
}

let previous = performance.now(),
  lastUI = 0;
function frame(now) {
  requestAnimationFrame(frame);
  if (ready) {
    simulation.advance(document.hidden ? 0 : (now - previous) / 1000);
    if (now - lastUI > 100) {
      document.querySelector("#gripper-opening").textContent =
        `${simulation.getGripperState().opening.toFixed(2)} mm`;
      telemetry.update(simulation.getMotorTelemetry(), simulation.paused);
      controls.forEach((control, i) => {
        control.actual.textContent = `${simulation.data.qpos[simulation.joints[i].q].toFixed(2)} rad`;
      });
      document.querySelector("#status").textContent = simulation.paused
        ? "仿真已暂停"
        : simulation.getSelfContacts().length
          ? "自身接触 · 物理阻挡"
          : "仿真运行中";
      lastUI = now;
    }
  }
  previous = now;
  viewer?.render(ready ? simulation.data : null);
}
requestAnimationFrame(frame);
