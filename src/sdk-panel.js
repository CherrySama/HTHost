import {
  actionPreviewPoints, capturePose, compileActions, MAX_ACTION_SEGMENTS, SimulationActions,
} from "./sdk-actions.js";

export const sdkPanel = `
  <section class="sidebar-panel sdk-panel" id="sdk-panel" role="tabpanel" aria-labelledby="sdk-tab" hidden>
    <h2>动作编排</h2>
    <form id="sdk-editor">
      <fieldset class="sdk-group">
        <legend>预设示例</legend>
        <div class="sdk-buttons">
          <button type="button" id="sdk-example-multi">多关节协调</button>
          <button type="button" id="sdk-example-sine">SDK 正弦运动</button>
        </div>
      </fieldset>
      <button type="button" id="sdk-record">记录姿态</button>
      <details class="sdk-group">
        <summary>末端动作</summary>
        <div class="sdk-fields">
          <label>运动方式<select id="sdk-path-kind">
            <option value="line">直线</option><option value="rotate">绕轴旋转</option>
            <option value="sine">正弦往返</option><option value="smooth">平滑插值</option>
          </select></label>
          <div class="sdk-row">
            <label>基座坐标轴<select id="sdk-axis"><option value="0">X</option><option value="1" selected>Y</option><option value="2">Z</option></select></label>
            <label><span id="sdk-amount-label">位移（mm）</span><input type="number" id="sdk-amount" value="10" step="any"></label>
          </div>
          <label>步骤时间（秒）<input type="number" id="sdk-duration" min="0.1" max="300" step="any" value="2"></label>
          <button type="button" id="sdk-add-path">添加末端动作</button>
        </div>
      </details>
      <details class="sdk-group">
        <summary>辅助步骤</summary>
        <div class="sdk-fields">
          <div class="sdk-row">
            <label>类型<select id="sdk-helper">
              <option value="hold">停顿</option><option value="transition">过渡</option><option value="zero">回零</option>
            </select></label>
            <label>时间（秒）<input type="number" id="sdk-helper-duration" min="0.1" max="300" step="any" value="2"></label>
          </div>
          <button type="button" id="sdk-add-helper">添加辅助步骤</button>
        </div>
      </details>
      <div class="sdk-step-header">
        <span id="sdk-steps-label">动作步骤</span>
        <button type="button" id="sdk-clear" aria-controls="sdk-steps" disabled>清空</button>
      </div>
      <ol id="sdk-steps" aria-labelledby="sdk-steps-label"></ol>
      <div class="sdk-settings">
        <label>速度（rad/s）<input id="sdk-speed" type="number" min="0.01" max="1" step="any" value="0.2"></label>
        <label>到位停留（秒）<input id="sdk-dwell" type="number" min="0" max="300" step="any" value="0"></label>
        <label>循环次数<input id="sdk-loops" type="number" min="1" max="100" value="1" step="1"></label>
      </div>
      <label><span><input id="sdk-return" type="checkbox"> 每轮返回起点</span></label>
    </form>
    <div class="actions sdk-actions">
      <button type="button" id="sdk-run">仿真运行</button>
      <button type="button" id="sdk-preview">预览</button>
      <button type="button" id="sdk-stop" disabled>停止</button>
    </div>
    <label id="sdk-scrub-label" hidden>预览位置<input type="range" id="sdk-scrub" min="0" value="0"></label>
    <p id="sdk-status" role="status"></p>
  </section>`;

const phaseLabels = {
  approach: "前往起点", playing: "执行路径", finishing: "确认到位", dwell: "停留",
  stopping: "正在停止", stopped: "已停止，保持当前位置", done: "动作完成",
};

export function createSdkPanel(callbacks) {
  const root = document.querySelector("#sdk-panel");
  const get = (name) => root.querySelector("#sdk-" + name);
  const editor = get("editor");
  const list = get("steps");
  const clearButton = get("clear");
  const pathKind = get("path-kind");
  const statusLine = get("status");
  const items = [];
  let simulationRun;
  let hardware;
  let busy = false;
  let preview;
  let pendingStop = false;
  let statusTimer;
  let stopError;

  function tell(text, transient = false) {
    clearTimeout(statusTimer);
    if (statusLine.textContent !== text) statusLine.textContent = text;
    statusLine.classList.toggle("is-visible", Boolean(text));
    if (transient) {
      statusTimer = setTimeout(() => statusLine.classList.remove("is-visible"), 2000);
    }
  }
  const active = () => Boolean(simulationRun || ["approach", "playing", "finishing", "dwell", "stopping"].includes(hardware?.phase));
  const locked = () => busy || active();
  function refresh() {
    const context = callbacks.context();
    const disabled = !context || locked();
    editor.querySelectorAll("button, input, select").forEach((element) => {
      element.disabled = disabled || element.dataset.edge === "true";
    });
    get("run").textContent = context?.real ? "真机运行" : "仿真运行";
    get("speed").max = context?.maxSpeed ?? 1;
    get("run").disabled = disabled || !items.length || !context.canRun;
    get("preview").disabled = disabled || !items.length;
    clearButton.disabled = disabled || !items.length;
    get("stop").disabled = !active() || pendingStop || hardware?.phase === "stopping";
  }

  function clearPreview() {
    if (!preview) return;
    preview = undefined;
    get("scrub-label").hidden = true;
    callbacks.preview(null);
  }

  function updateList() {
    clearPreview();
    list.replaceChildren();
    items.forEach((item, index) => {
      const row = document.createElement("li");
      const title = document.createElement("span");
      title.textContent = item.label;
      row.append(title);
      for (const [label, description, move] of [["↑", "上移", -1], ["↓", "下移", 1], ["×", "删除", 0]]) {
        const button = document.createElement("button");
        button.type = "button";
        button.textContent = label;
        button.setAttribute("aria-label", description + " " + item.label);
        button.dataset.edge = String(move !== 0 && !items[index + move]);
        button.onclick = () => {
          const [removed] = items.splice(index, 1);
          if (move) items.splice(index + move, 0, removed);
          updateList();
        };
        row.append(button);
      }
      list.append(row);
    });
    refresh();
  }

  function updatePathUnits() {
    get("amount-label").textContent = pathKind.value === "rotate" ? "转角（°）" : "位移（mm）";
  }

  function capture() {
    const context = callbacks.context();
    return capturePose(context.pose, context.limits, context.gripperLimits);
  }
  function append(newItems) {
    if (items.length + newItems.length > MAX_ACTION_SEGMENTS) throw new Error("最多 64 个动作步骤");
    items.push(...newItems);
    updateList();
  }
  const poseItem = (label, pose) => ({ kind: "pose", label, pose });
  let nextPoint = 1;
  clearButton.onclick = () => {
    items.length = 0;
    nextPoint = 1;
    updateList();
    tell("动作步骤已清空", true);
  };

  function coordinatedExample() {
    const context = callbacks.context();
    const start = capture();
    const middle = { ...start, positions: start.positions.slice() };
    const excursion = 25 * Math.PI / 180;
    for (const [i, value] of start.positions.entries()) {
      const [min, max] = context.limits[i];
      const positiveRoom = max - value;
      const negativeRoom = value - min;
      middle.positions[i] = positiveRoom >= excursion || positiveRoom >= negativeRoom
        ? value + Math.min(excursion, positiveRoom)
        : value - Math.min(excursion, negativeRoom);
    }
    return [poseItem("起始姿态", start), poseItem("演示姿态", middle), poseItem("返回起始姿态", start)];
  }

  function addSteps(build) {
    return () => {
      try {
        append(build());
        tell("步骤已加入", true);
      } catch (error) {
        tell(error.message);
      }
    };
  }

  function stepDuration(input) {
    const duration = input.valueAsNumber;
    if (!Number.isFinite(duration) || duration < 0.1 || duration > 300) {
      throw new Error("步骤时间须在 0.1–300 秒之间");
    }
    return duration;
  }

  get("record").onclick = addSteps(() => [poseItem("路点 " + nextPoint++, capture())]);
  get("example-multi").onclick = addSteps(coordinatedExample);
  get("example-sine").onclick = addSteps(() => [{ kind: "joint-sine", label: "SDK 正弦运动" }]);
  get("add-path").onclick = addSteps(() => {
    const rotation = pathKind.value === "rotate";
    const amount = get("amount").valueAsNumber;
    const limit = rotation ? 180 : 300;
    const unit = rotation ? "°" : " mm";
    if (!Number.isFinite(amount) || !amount || Math.abs(amount) > limit) {
      throw new Error("数值须非零，且在 ±" + limit + unit + " 内");
    }
    const axis = get("axis");
    const label = pathKind.selectedOptions[0].textContent + " · "
      + axis.selectedOptions[0].textContent + " " + amount + unit;
    return [{ kind: pathKind.value, duration: stepDuration(get("duration")), axis: Number(axis.value), amount, label }];
  });
  get("add-helper").onclick = addSteps(() => {
    const helper = get("helper");
    const duration = stepDuration(get("helper-duration"));
    return [{ kind: helper.value, duration, label: helper.selectedOptions[0].textContent + " · " + duration + " s" }];
  });
  pathKind.onchange = updatePathUnits;
  editor.oninput = clearPreview;
  editor.onsubmit = (event) => event.preventDefault();

  function settings(context) {
    const speed = get("speed").valueAsNumber;
    const dwell = get("dwell").valueAsNumber;
    const loops = get("loops").valueAsNumber;
    if (!Number.isFinite(speed) || speed < 0.01 || speed > context.maxSpeed) throw new Error("速度超出当前允许范围");
    if (!Number.isFinite(dwell) || dwell < 0 || dwell > 300) throw new Error("停留时间须在 0–300 秒之间");
    if (!Number.isInteger(loops) || loops < 1 || loops > 100) throw new Error("循环次数须为 1–100 的整数");
    return { speed, dwell, loops, returnToStart: get("return").checked };
  }

  async function prepare(run) {
    busy = true;
    stopError = undefined;
    tell("");
    clearPreview();
    callbacks.changed();
    try {
      const context = callbacks.context();
      const start = capturePose(context.pose, context.limits, context.gripperLimits);
      const plan = compileActions(items, start, settings(context), context);
      const current = callbacks.context();
      if (!current || current.session !== context.session || current.real !== context.real) throw new Error("连接状态已变化，请重新操作");
      if (!run) {
        preview = { points: actionPreviewPoints(plan, start), start, context };
        const slider = get("scrub");
        slider.max = preview.points.length - 1;
        slider.value = slider.max;
        get("scrub-label").hidden = false;
        showPreview();
        tell("预览就绪 · 可拖动滑条查看路径", true);
      } else if (!current.canRun) {
        throw new Error("请等待当前控制结束，并选择位置控制");
      } else if (context.real) {
        updateHardware(await callbacks.run({ ...plan, session: context.session }));
      } else {
        simulationRun = new SimulationActions(plan, start, context.simulation.data.time);
        tell("仿真动作已开始");
      }
    } catch (error) { tell(error.message); }
    finally { busy = false; callbacks.changed(); }
  }

  function showPreview() {
    if (!preview) return;
    callbacks.preview(preview.points[Number(get("scrub").value)], preview.start, preview.points, preview.context);
  }
  get("scrub").oninput = showPreview;
  get("preview").onclick = () => void prepare(false);
  get("run").onclick = () => void prepare(true);
  get("stop").onclick = async () => {
    if (simulationRun) {
      stopSimulation("已停止，保持当前位置");
      return;
    }
    pendingStop = true;
    stopError = undefined;
    refresh();
    try {
      updateHardware(await callbacks.stop(hardware.id));
    } catch (error) {
      stopError = error.message;
      if (hardware) display(hardware);
    }
    finally { pendingStop = false; refresh(); }
  };

  function stopSimulation(message, transient = true) {
    const simulation = callbacks.context().simulation;
    const pose = simulation.getActionPose();
    pose.positions.forEach((angle, i) => simulation.setTarget(i, angle));
    simulation.setGripperTarget(pose.gripper);
    simulationRun = undefined;
    callbacks.syncSimulation();
    tell(message, transient);
    callbacks.changed();
  }

  function display(status) {
    const text = phaseLabels[status.phase] ?? status.message ?? "";
    const progress = "第 " + status.round + "/" + status.total + " 轮 · " + status.step + "/" + status.steps + " " + status.label;
    const finished = status.phase === "done" || status.phase === "stopped";
    tell(status.phase === "error" ? status.message
      : [text, status.message, stopError, progress].filter(Boolean).join(" · "), finished && !stopError);
  }

  function updateHardware(status, disconnected = false) {
    if (!status && hardware && active() && !disconnected) return;
    if (!status && hardware && active()) tell("真机动作连接已结束");
    const changed = status?.phase !== hardware?.phase || status?.id !== hardware?.id;
    hardware = status;
    if (status && (changed || active())) display(status);
    if (changed) callbacks.changed();
  }

  updatePathUnits();
  updateList();
  return {
    active, locked, refresh, clearPreview, updateHardware,
    tick(simulation) {
      if (!simulationRun) return;
      try {
        if (simulationRun.tick(simulation)) display(simulationRun.status);
        else {
          simulationRun = undefined;
          callbacks.syncSimulation();
          tell("仿真动作完成，保持最后目标", true);
          callbacks.changed();
        }
      } catch (error) { stopSimulation(error.message, false); }
    },
  };
}
