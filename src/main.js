import "./style.css";
import { Simulation } from "./simulation.js";
import { Viewer } from "./viewer.js";
import { MotorTelemetry } from "./motor-telemetry.js";
import { gripperOpening } from "./gripper-calibration.js";

document.querySelector("#app").innerHTML = `
  <header><a class="brand" href="/" aria-label="Panthera Studio 首页"><span class="brand-icon">P</span>Panthera <span>Studio</span></a>
    <span class="mode"><i></i> 本地仿真</span>
    <div class="robot-connection" data-state="disconnected">
      <button id="robot-connect" type="button" disabled>连接真机</button>
      <span id="robot-status" class="robot-status" role="status"><i aria-hidden="true"></i><span>未连接</span></span>
    </div></header>
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
    <p id="joint-intro" class="intro">调整目标角度，探索机械臂的运动。</p>
    <section id="robot-motion" hidden>
      <label for="robot-speed">全局速度 <span>rad/s</span></label>
      <div class="joint-input">
        <input id="robot-speed-slider" type="range" aria-label="全局速度滑条（rad/s）" min="0.01" step="0.01">
        <div class="number-wrap"><input id="robot-speed" type="number" min="0.01" step="0.01" value="0.2"></div>
      </div>
      <div class="actions"><button id="send-robot-position" type="button" disabled>发送位置</button><button id="reset-robot" type="button" disabled>全部复位</button><button id="return-to-simulation" type="button" hidden>返回仿真</button></div>
      <p id="target-preview-key" hidden>蓝色半透明模型为待发送目标</p>
      <p id="robot-motion-status" role="status" hidden></p>
    </section>
    <div class="column-key"><span>关节 / 目标角度</span><span>实际角度</span></div>
    <div id="joints"></div>
    <section id="gripper-controls" class="joint" aria-label="夹爪控制"><div class="joint-heading"><label for="gripper-angle"><b>夹爪</b><span>电机目标 · rad</span></label><output id="gripper-opening" aria-label="夹爪实际开口">82.00 mm</output></div>
    <div class="joint-input"><input id="gripper-slider" type="range" aria-label="夹爪电机角度滑条（rad）" min="0" max="2" step="0.01" value="2"><div class="number-wrap"><input id="gripper-angle" type="number" aria-label="夹爪电机目标（rad）" min="0" max="2" step="0.01" value="2"></div></div>
    <div class="limits"><span>闭合 · 0 rad</span><span>张开 · 2 rad</span></div><p class="gripper-summary" id="gripper-target-opening">目标总开口 82.00 mm</p>
    <div class="gripper-buttons"><button id="gripper-close" disabled>闭合</button><button id="gripper-open" disabled>张开</button></div></section>
    <div id="simulation-actions" class="actions"><button id="reset" disabled>↺　全部复位</button><button id="pause" disabled aria-pressed="false">暂停</button></div>
    <div id="simulation-info" class="info"><span class="info-icon">◎</span><div><strong>在虚拟空间，自由探索。</strong><p>自碰撞物理阻挡 · 关节限位保护<br>仅控制虚拟机械臂</p></div></div>
    </section><section class="sidebar-panel" id="motor-monitor" role="tabpanel" aria-labelledby="monitor-tab" hidden></section>
    <div class="panel-bottom"><span>Panthera HT</span><span>6 DOF <b>·</b> V1.1.0</span></div>
  </aside></main>`;

const robotButton = document.querySelector("#robot-connect");
const robotStatus = document.querySelector("#robot-status span");
const robotConnection = robotButton.parentElement;
const robotStatusLabels = {
  disconnected: "未连接",
  connecting: "连接中…",
  connected: "已连接",
  disconnecting: "断开中…",
  homing: "回零中…",
  error: "连接失败",
};
let robotStatusTimer;
let robotRequest = 0;
let robotPose;
let robotTargets = [];
const editedRobotJoints = new Set();
let robotHasCommand = false;
let robotGripperTarget;
let gripperEdited = false;
let gripperHasCommand = false;
let sendingPosition = false;

function showRobotStatus({ state, message }) {
  const stateChanged = robotConnection.dataset.state !== state;
  robotConnection.dataset.state = state;
  const text = message || robotStatusLabels[state];
  if (robotStatus.textContent !== text) robotStatus.textContent = text;
  robotButton.disabled = ["connecting", "disconnecting", "homing"].includes(state);
  const buttonText = ["connected", "disconnecting", "homing"].includes(state)
    ? "断开真机"
    : "连接真机";
  if (robotButton.textContent !== buttonText) robotButton.textContent = buttonText;
  if (state === "error") telemetry?.markDisconnected();
  if (stateChanged) updateMotionControls();
}

async function refreshRobotStatus(method = "GET", initial = false) {
  const request = ++robotRequest;
  try {
    const response = await fetch("/api/panthera/connection", {
      method,
      headers: method === "GET" ? undefined : { "Content-Type": "application/json" },
      body: method === "DELETE" ? JSON.stringify({ session: robotPose?.session }) : undefined,
      signal: AbortSignal.timeout(5000),
    });
    if (request !== robotRequest) return;
    const status = await response.json();
    if (request !== robotRequest) return;
    if (method === "DELETE" && response.status === 409) {
      showMotionMessage(status.message);
      showRobotStatus({ state: "connected", message: status.message });
      robotStatusTimer = setTimeout(refreshRobotStatus, 500);
      return;
    }
    if (!response.ok) throw new Error("连接服务不可用");
    if (status.state === "connected" || status.state === "homing") receiveRobotPose(status);
    if (status.state === "disconnected" && robotPose) {
      robotPose = undefined;
      updateMotionControls();
    }
    showRobotStatus(status);
    if (["connecting", "connected", "homing"].includes(status.state)) {
      robotStatusTimer = setTimeout(refreshRobotStatus, status.state === "connecting" ? 500 : 100);
    }
  } catch {
    if (!initial && request === robotRequest) {
      const returning = method === "DELETE" || robotConnection.dataset.state === "homing";
      showRobotStatus({
        state: "error",
        message: returning ? "回零未确认：无法访问真机连接服务" : "无法访问真机连接服务",
      });
    }
  }
}

robotButton.onclick = () => {
  clearTimeout(robotStatusTimer);
  const connected = robotConnection.dataset.state === "connected";
  showRobotStatus({ state: connected ? "disconnecting" : "connecting" });
  void refreshRobotStatus(connected ? "DELETE" : "POST");
};

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

const motionPanel = document.querySelector("#robot-motion");
const speedInput = document.querySelector("#robot-speed");
const speedSlider = document.querySelector("#robot-speed-slider");
const sendPosition = document.querySelector("#send-robot-position");
const resetRobot = document.querySelector("#reset-robot");
const motionStatus = document.querySelector("#robot-motion-status");
const returnToSimulation = document.querySelector("#return-to-simulation");
const previewKey = document.querySelector("#target-preview-key");
const gripperSection = document.querySelector("#gripper-controls");
const gripperSlider = document.querySelector("#gripper-slider");
const gripperInput = document.querySelector("#gripper-angle");
const gripperClose = document.querySelector("#gripper-close");
const gripperOpen = document.querySelector("#gripper-open");
const gripperActual = document.querySelector("#gripper-opening");

function receiveRobotPose(status) {
  const newSession = robotPose?.session !== status.session;
  robotPose = status;
  if (newSession) {
    robotTargets = status.positions.slice();
    editedRobotJoints.clear();
    robotHasCommand = false;
    gripperEdited = false;
    gripperHasCommand = false;
    robotGripperTarget = status.gripper;
    sendingPosition = false;
    showMotionMessage("");
    speedInput.max = speedSlider.max = status.maxSpeed;
    speedInput.value = speedSlider.value = Math.min(0.2, status.maxSpeed);
    updateSpeedTrack();
    controls.forEach((control, index) => {
      control.setLimits(...status.limits[index]);
      control.display(robotTargets[index]);
    });
    if (status.gripperLimits) setGripperLimits(...status.gripperLimits);
    updateMotionControls();
  } else if (!robotHasCommand && !sendingPosition) {
    controls.forEach((control, index) => {
      if (editedRobotJoints.has(index)) return;
      robotTargets[index] = status.positions[index];
      control.display(robotTargets[index], true);
    });
  }
  if (status.gripperLimits && !gripperEdited && !gripperHasCommand && !sendingPosition) {
    robotGripperTarget = status.gripper;
    displayGripperTarget(robotGripperTarget, true);
  }
  if (ready) {
    simulation.setRobotPose(status.positions, status.gripper);
    updateTargetPreview();
    telemetry.update({
      source: "robot",
      session: status.session,
      time: status.sampleTime,
      joints: status.motors.map((motor, index) => ({
        ...motor,
        name: index < labels.length ? `J${index + 1}` : "夹爪",
        error: motor.target === null ? null : motor.target - motor.position,
      })),
    }, false);
  }
}

function updateTargetPreview() {
  const show = Boolean(
    robotPose && robotConnection.dataset.state === "connected"
    && (editedRobotJoints.size || gripperEdited),
  );
  previewKey.hidden = !show;
  if (!show) {
    viewer?.setTargetPreview(null);
    return;
  }
  const data = simulation.setTargetPreview(
    editedRobotJoints.size ? robotTargets : robotPose.positions,
    gripperEdited ? robotGripperTarget : robotPose.gripper,
  );
  viewer.setTargetPreview(data, simulation.robotData);
}

function updateMotionControls() {
  if (!ready) return;
  const state = robotConnection.dataset.state;
  const live = state === "connected" && Boolean(robotPose);
  const realView = Boolean(robotPose);
  document.querySelector(".mode").lastChild.textContent = realView ? " 真机视图" : " 本地仿真";
  document.querySelector("#joint-intro").textContent = realView
    ? (state === "homing" ? "正在回零，到位后自动断开。" : "调整目标角度和速度，然后发送位置。")
    : "调整目标角度，探索机械臂的运动。";
  const returningToSimulation = !realView && !motionPanel.hidden;
  motionPanel.hidden = !realView;
  gripperSection.hidden = realView && !robotPose.gripperLimits;
  gripperSlider.disabled = gripperInput.disabled = gripperClose.disabled = gripperOpen.disabled =
    realView && (!live || sendingPosition);
  document.querySelector("#simulation-actions").hidden = realView;
  document.querySelector("#simulation-info").hidden = realView;
  returnToSimulation.hidden = !realView || state !== "error";
  sendPosition.disabled = !live || !(editedRobotJoints.size || gripperEdited) || sendingPosition;
  resetRobot.disabled = !live || sendingPosition ||
    robotPose.limits.some(([lower, upper]) => lower > 0 || upper < 0) ||
    Boolean(robotPose.gripperLimits &&
      (robotPose.gripperLimits[0] > 0 || robotPose.gripperLimits[1] < 0));
  speedInput.disabled = speedSlider.disabled = !live || sendingPosition;
  controls.forEach((control, index) => {
    control.setDisabled(realView && (!live || sendingPosition));
    if (returningToSimulation) {
      const { min, max } = simulation.joints[index];
      control.setLimits(min, max);
      control.display(simulation.targets[index]);
    }
  });
  if (returningToSimulation) {
    setGripperLimits(0, 2);
    displayGripperTarget(simulation.gripperTarget);
    telemetry.update(simulation.getMotorTelemetry(), simulation.paused);
  }
  updateTargetPreview();
}

function showMotionMessage(message) {
  motionStatus.textContent = message;
  motionStatus.hidden = !message;
}

function updateSpeedTrack() {
  const lower = Number(speedSlider.min);
  const upper = Number(speedSlider.max);
  const fill = upper > lower ? (speedSlider.valueAsNumber - lower) / (upper - lower) : 0;
  speedSlider.style.setProperty("--fill", `${fill * 100}%`);
}

speedSlider.oninput = () => {
  speedInput.value = speedSlider.value;
  updateSpeedTrack();
};
speedInput.oninput = () => {
  if (!speedInput.value || !speedInput.validity.valid) return;
  speedSlider.value = speedInput.value;
  updateSpeedTrack();
};
speedInput.onchange = () => {
  if (!speedInput.value || !speedInput.validity.valid) speedInput.value = speedSlider.value;
};
returnToSimulation.onclick = () => {
  robotPose = undefined;
  showRobotStatus({ state: "disconnected" });
};
resetRobot.onclick = () => {
  if (resetRobot.disabled) return;
  controls.forEach((control, index) => {
    robotTargets[index] = 0;
    editedRobotJoints.add(index);
    control.display(0);
  });
  if (robotPose.gripperLimits) {
    robotGripperTarget = 0;
    gripperEdited = true;
    displayGripperTarget(0);
  }
  showMotionMessage("复位目标已暂存，点击「发送位置」执行。");
  updateMotionControls();
};
sendPosition.onclick = async () => {
  if (sendPosition.disabled || !speedInput.reportValidity()) return;
  const { session } = robotPose;
  const positions = editedRobotJoints.size ? robotTargets.slice() : undefined;
  const gripper = gripperEdited ? robotGripperTarget : undefined;
  sendingPosition = true;
  showMotionMessage("");
  updateMotionControls();
  try {
    const response = await fetch("/api/panthera/position", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ session, positions, gripper, speed: speedInput.valueAsNumber }),
      signal: AbortSignal.timeout(5000),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.message);
    if (robotPose?.session !== session) return;
    if (positions) robotHasCommand = true;
    if (gripper !== undefined) gripperHasCommand = true;
    editedRobotJoints.clear();
    gripperEdited = false;
    showMotionMessage(result.message);
  } catch (error) {
    if (robotPose?.session === session) showMotionMessage(error.message || "发送失败");
  } finally {
    if (robotPose?.session === session) {
      sendingPosition = false;
      updateMotionControls();
    }
  }
};

function buildControls() {
  simulation.joints.forEach((joint, i) => {
    const row = document.createElement("section");
    row.className = "joint";
    const min = joint.min,
      max = joint.max;
    // Align the slider's 0.01-rad steps to zero, within the model's exact limits.
    let sliderMin = Math.ceil(min * 100) / 100;
    let sliderMax = Math.floor(max * 100) / 100;
    row.innerHTML = `<div class="joint-heading"><label for="angle-${i}"><b>J${i + 1}</b><span>${labels[i]}</span></label><output id="actual-${i}" aria-label="J${i + 1} 实际角度（rad）">0.00 rad</output></div>
      <div class="joint-input"><input type="range" id="slider-${i}" aria-label="J${i + 1} 目标角度滑条（rad）" min="${sliderMin}" max="${sliderMax}" step="0.01" value="0"><div class="number-wrap"><input type="number" id="angle-${i}" aria-label="J${i + 1} 目标角度（rad）" min="${min}" max="${max}" step="0.01" value="0.00"></div></div>
      <div class="limits"><span>${min.toFixed(5)} rad</span><span>${max.toFixed(5)} rad</span></div>`;
    document.querySelector("#joints").appendChild(row);
    const slider = row.querySelector("[type=range]"),
      number = row.querySelector("[type=number]");
    const limitLabels = row.querySelectorAll(".limits span");
    const setLimits = (lower, upper) => {
      sliderMin = Math.ceil(lower * 100) / 100;
      sliderMax = Math.floor(upper * 100) / 100;
      slider.min = sliderMin;
      slider.max = sliderMax;
      number.min = lower;
      number.max = upper;
      limitLabels[0].textContent = `${lower.toFixed(5)} rad`;
      limitLabels[1].textContent = `${upper.toFixed(5)} rad`;
    };
    const display = (radians, preserveInput = false) => {
      slider.value = radians;
      if (!preserveInput || document.activeElement !== number) {
        number.value = String(Number(radians.toFixed(5)));
      }
      slider.style.setProperty(
        "--fill",
        `${((Number(slider.value) - sliderMin) / (sliderMax - sliderMin)) * 100}%`,
      );
    };
    const update = (value, preserveInput = false) => {
      if (!robotPose) {
        simulation.setTarget(i, value);
        display(simulation.targets[i]);
        return;
      }
      if (!slider.disabled && Number.isFinite(value)) {
        const [lower, upper] = robotPose.limits[i];
        const radians = Math.max(lower, Math.min(upper, value));
        if (radians !== robotTargets[i]) {
          robotTargets[i] = radians;
          editedRobotJoints.add(i);
          showMotionMessage("");
          updateMotionControls();
        }
      }
      display(robotTargets[i], preserveInput);
    };
    slider.addEventListener("pointerdown", () =>
      viewer.highlightJoint(joint.id, true),
    );
    slider.addEventListener("input", () => {
      update(Number(slider.value));
      viewer.highlightJoint(joint.id);
    });
    number.addEventListener("input", () => {
      if (robotPose) update(number.valueAsNumber, true);
    });
    number.addEventListener("change", () => {
      update(
        number.value === ""
          ? (robotPose ? robotTargets[i] : simulation.targets[i])
          : Number(number.value),
      );
      viewer.highlightJoint(joint.id);
    });
    number.addEventListener("keydown", (event) => {
      if (event.key === "Enter") number.blur();
    });
    update(0);
    controls.push({
      update,
      display,
      setLimits,
      setDisabled(disabled) {
        slider.disabled = number.disabled = disabled;
      },
      actual: row.querySelector("output"),
    });
  });
}

function setGripperLimits(lower, upper) {
  gripperSlider.min = gripperInput.min = lower;
  gripperSlider.max = gripperInput.max = upper;
  const [closed, opened] = gripperSection.querySelectorAll(".limits span");
  closed.textContent = `闭合 · ${lower} rad`;
  opened.textContent = `张开 · ${upper} rad`;
}

function displayGripperTarget(target, preserveInput = false) {
  const lower = Number(gripperSlider.min);
  const upper = Number(gripperSlider.max);
  const displayTarget = Math.max(lower, Math.min(upper, target));
  gripperSlider.value = displayTarget;
  gripperSlider.style.setProperty("--fill", `${(displayTarget - lower) / (upper - lower) * 100}%`);
  if (!preserveInput || document.activeElement !== gripperInput) {
    gripperInput.value = displayTarget.toFixed(2);
  }
  document.querySelector("#gripper-target-opening").textContent =
    `目标总开口 ${gripperOpening(displayTarget).toFixed(2)} mm`;
}

function updateGripper(value) {
  simulation.setGripperTarget(value);
  displayGripperTarget(simulation.gripperTarget);
}

function applyGripperTarget(value, preserveInput = false) {
  if (!ready || gripperSlider.disabled || !Number.isFinite(value)) return;
  if (robotPose) {
    const [lower, upper] = robotPose.gripperLimits;
    const target = Math.max(lower, Math.min(upper, value));
    if (target !== robotGripperTarget) {
      robotGripperTarget = target;
      gripperEdited = true;
      showMotionMessage("");
      updateMotionControls();
    }
    displayGripperTarget(robotGripperTarget, preserveInput);
  } else {
    updateGripper(value);
  }
  viewer.highlightJoint(simulation.gripperHighlightJointId);
}

gripperSlider.onpointerdown = () => {
  if (ready) viewer.highlightJoint(simulation.gripperHighlightJointId, true);
};
gripperSlider.oninput = (event) => {
  applyGripperTarget(Number(event.target.value));
};
gripperInput.oninput = () => {
  if (robotPose) applyGripperTarget(gripperInput.valueAsNumber, true);
};
gripperInput.onchange = () => {
  if (gripperInput.value === "") {
    displayGripperTarget(robotPose ? robotGripperTarget : simulation.gripperTarget);
  } else {
    applyGripperTarget(gripperInput.valueAsNumber);
  }
};
gripperInput.onkeydown = (event) => {
  if (event.key === "Enter") event.target.blur();
};
gripperClose.onclick = () => applyGripperTarget(Number(gripperSlider.min));
gripperOpen.onclick = () => applyGripperTarget(Number(gripperSlider.max));

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
  showRobotStatus({ state: "disconnected" });
  void refreshRobotStatus("GET", true);
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
    if (!robotPose) simulation.advance(document.hidden ? 0 : (now - previous) / 1000);
    if (now - lastUI > 100) {
      if (robotPose && Number.isFinite(robotPose.gripper)) {
        gripperActual.textContent = `${gripperOpening(robotPose.gripper).toFixed(2)} mm`;
      } else if (!robotPose) {
        gripperActual.textContent = `${simulation.getGripperState().opening.toFixed(2)} mm`;
        telemetry.update(simulation.getMotorTelemetry(), simulation.paused);
      }
      controls.forEach((control, i) => {
        const position = robotPose?.positions[i] ?? simulation.data.qpos[simulation.joints[i].q];
        control.actual.textContent = `${position.toFixed(2)} rad`;
      });
      const robotState = robotConnection.dataset.state;
      document.querySelector("#status").textContent = robotPose
        ? (robotState === "homing" ? "回零中 · 真机姿态同步"
          : robotState === "connected" ? "真机姿态同步中" : "保留最后一次真机姿态")
        : simulation.paused
        ? "仿真已暂停"
        : simulation.getSelfContacts().length
          ? "自身接触 · 物理阻挡"
          : "仿真运行中";
      lastUI = now;
    }
  }
  previous = now;
  viewer?.render(ready ? (robotPose ? simulation.robotData : simulation.data) : null);
}
requestAnimationFrame(frame);
