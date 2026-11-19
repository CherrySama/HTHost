import {
  actionPreviewPoints, capturePose, compileActions, MAX_ACTION_SEGMENTS,
  MAX_RECORDING_FRAMES, normalizeRecording, SimulationActions,
} from "./sdk-actions.js";

const playbackSettings = (prefix = "") => `
  <div class="sdk-settings">
    <label>速度（rad/s）<input id="sdk-${prefix}speed" type="number" min="0.01" max="1" step="any" value="0.2"></label>
    <label>到位停留（秒）<input id="sdk-${prefix}dwell" type="number" min="0" max="300" step="any" value="0"></label>
    <label>循环次数<input id="sdk-${prefix}loops" type="number" min="1" max="100" value="1" step="1"></label>
  </div>
  <label><span><input id="sdk-${prefix}return" type="checkbox"> 每轮返回起点</span></label>`;

export const sdkPanel = `
  <section class="sidebar-panel sdk-panel" id="sdk-panel" role="tabpanel" aria-labelledby="sdk-tab" hidden>
    <div class="sdk-tabs" role="tablist" aria-label="SDK 脚本功能">
      <button type="button" id="sdk-compose-tab" role="tab" aria-selected="true" aria-controls="sdk-compose">动作编排</button>
      <button type="button" id="sdk-trajectory-tab" role="tab" aria-selected="false" aria-controls="sdk-trajectory" tabindex="-1">轨迹录制与回放</button>
    </div>
    <section id="sdk-compose" role="tabpanel" aria-labelledby="sdk-compose-tab">
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
        ${playbackSettings()}
      </form>
      <div class="actions sdk-actions">
        <button type="button" id="sdk-run">仿真运行</button>
        <button type="button" id="sdk-preview">预览</button>
        <button type="button" id="sdk-stop" class="sdk-stop" disabled>停止</button>
      </div>
    </section>
    <section id="sdk-trajectory" role="tabpanel" aria-labelledby="sdk-trajectory-tab" hidden>
      <h2>轨迹录制与回放</h2>
      <div class="sdk-fields">
        <div class="sdk-buttons">
          <button type="button" id="sdk-capture">开始录制</button>
          <button type="button" id="sdk-import">导入</button>
          <button type="button" id="sdk-export">导出</button>
        </div>
        <input type="file" id="sdk-file" accept=".jsonl,application/jsonl" hidden>
        <span id="sdk-track-info"></span>
        <span id="sdk-track-hint" hidden></span>
        ${playbackSettings("track-")}
        <span id="sdk-track-estimate" hidden></span>
        <div class="actions sdk-actions">
          <button type="button" id="sdk-replay">仿真回放</button>
          <button type="button" id="sdk-track-preview">预览</button>
          <button type="button" id="sdk-track-stop" class="sdk-stop" disabled>停止</button>
        </div>
      </div>
    </section>
    <label id="sdk-scrub-label" hidden>预览位置<input type="range" id="sdk-scrub" min="0" value="0"></label>
    <p id="sdk-status" role="status"></p>
  </section>`;

const phaseLabels = {
  approach: "前往起点", playing: "执行路径", finishing: "确认到位", dwell: "停留",
  stopping: "正在停止", stopped: "动作已停止，已发送阻尼指令", done: "动作完成",
  cancelled: "动作已取消，正在回零",
};

export function createSdkPanel(callbacks) {
  const root = document.querySelector("#sdk-panel");
  const get = (name) => root.querySelector("#sdk-" + name);
  const editor = get("editor");
  const list = get("steps");
  const clearButton = get("clear");
  const pathKind = get("path-kind");
  const statusLine = get("status");
  const trajectory = get("trajectory");
  const tabs = [get("compose-tab"), get("trajectory-tab")];
  const stopButtons = [get("stop"), get("track-stop")];
  const items = [];
  let simulationRun;
  let hardware;
  let busy = false;
  let preview;
  let pendingStop = false;
  let statusTimer;
  let stopError;
  let recording;
  let recordingBusy = false;
  let collectedId;
  let track;
  let trackName;
  let estimateTimer;
  let estimateKey;
  let estimatedTrack;

  function selectPage(tab) {
    if (tab.getAttribute("aria-selected") === "true") return;
    clearPreview();
    for (const item of tabs) {
      const selected = item === tab;
      item.setAttribute("aria-selected", String(selected));
      item.tabIndex = selected ? 0 : -1;
      root.querySelector("#" + item.getAttribute("aria-controls")).hidden = !selected;
    }
  }

  tabs.forEach((tab, index) => {
    tab.onclick = () => selectPage(tab);
    tab.onkeydown = (event) => {
      if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
      event.preventDefault();
      const next = event.key === "Home" ? tabs[0] : event.key === "End" ? tabs[1] : tabs[1 - index];
      selectPage(next);
      next.focus();
    };
  });

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
  const recordingNow = () => Boolean(recording || recordingBusy);
  function refresh() {
    const context = callbacks.context();
    const disabled = !context || locked();
    editor.querySelectorAll("button, input, select").forEach((element) => {
      element.disabled = disabled || element.dataset.edge === "true";
    });
    get("run").textContent = context?.real ? "真机运行" : "仿真运行";
    for (const prefix of ["", "track-"]) get(prefix + "speed").max = context?.maxSpeed ?? 1;
    for (const name of ["speed", "dwell", "loops", "return"]) {
      get("track-" + name).disabled = disabled || recordingNow();
    }
    get("run").disabled = disabled || recordingNow() || !items.length || !context.canRun;
    get("preview").disabled = disabled || recordingNow() || !items.length;
    clearButton.disabled = disabled || !items.length;
    for (const button of stopButtons) {
      button.disabled = !active() || pendingStop || hardware?.phase === "stopping";
    }
    get("capture").textContent = recording ? "结束录制" : "开始录制";
    get("capture").disabled = recordingBusy || (recording?.real && !context?.real)
      || (!recording && (disabled || !context.canRecord));
    get("import").disabled = locked() || recordingNow();
    get("export").disabled = !track || recordingNow();
    get("replay").textContent = context?.real ? "真机回放" : "仿真回放";
    get("replay").disabled = disabled || recordingNow() || !track || !context.canRun;
    get("track-preview").disabled = disabled || recordingNow() || !track;
    const info = recording ? recordingCount()
      : track ? trackName + " · " + track.at(-1).t.toFixed(1) + " 秒 · " + track.length + " 帧" : "暂无轨迹";
    if (get("track-info").textContent !== info) get("track-info").textContent = info;
    const hint = get("track-hint");
    const message = context?.real && !recordingNow() && !locked()
      ? context.recordingHint || [
        !context.canRecord && "真机示教请先选择重力补偿",
        track && !context.canRun && "真机回放请先选择位置控制",
      ].filter(Boolean).join(" · ") : "";
    hint.hidden = !message;
    if (hint.textContent !== message) hint.textContent = message;
    refreshEstimate(context);
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
  trajectory.oninput = (event) => {
    if (event.target !== get("file")) {
      clearPreview();
      refreshEstimate(callbacks.context());
    }
  };

  function settings(context, prefix) {
    const speed = get(prefix + "speed").valueAsNumber;
    const dwell = get(prefix + "dwell").valueAsNumber;
    const loops = get(prefix + "loops").valueAsNumber;
    if (!Number.isFinite(speed) || speed < 0.01 || speed > context.maxSpeed) throw new Error("速度超出当前允许范围");
    if (!Number.isFinite(dwell) || dwell < 0 || dwell > 300) throw new Error("停留时间须在 0–300 秒之间");
    if (!Number.isInteger(loops) || loops < 1 || loops > 100) throw new Error("循环次数须为 1–100 的整数");
    return { speed, dwell, loops, returnToStart: get(prefix + "return").checked };
  }

  function refreshEstimate(context) {
    const line = get("track-estimate");
    const key = track && context?.real && !recordingNow()
      ? JSON.stringify([context.session, ...["speed", "dwell", "loops"].map((name) => get("track-" + name).value),
        get("track-return").checked])
      : null;
    if (estimateKey === key && estimatedTrack === track) return;
    estimateKey = key;
    estimatedTrack = track;
    const currentTrack = track;
    clearTimeout(estimateTimer);
    line.hidden = !key;
    if (!key) return;
    line.textContent = "正在计算回放时长…";
    estimateTimer = setTimeout(async () => {
      try {
        const start = capturePose(context.pose, context.limits, context.gripperLimits);
        const plan = compileActions(trackItem(), start, settings(context, "track-"), context);
        const { seconds } = await callbacks.estimate({ ...plan, session: context.session });
        if (estimateKey !== key || estimatedTrack !== currentTrack) return;
        line.textContent = "预计 " + seconds.toFixed(1) + " 秒（不含前往起点和到位确认）";
      } catch (error) {
        if (estimateKey === key && estimatedTrack === currentTrack) line.textContent = error.message;
      }
    }, 180);
  }

  async function prepare(run, source = items, prefix = "") {
    busy = true;
    stopError = undefined;
    tell("");
    clearPreview();
    callbacks.changed();
    try {
      const context = callbacks.context();
      const start = capturePose(context.pose, context.limits, context.gripperLimits);
      const plan = compileActions(source, start, settings(context, prefix), context);
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
  const trackItem = () => [{ kind: "recording", label: "录制轨迹", frames: track }];
  get("track-preview").onclick = () => void prepare(false, trackItem(), "track-");
  get("replay").onclick = () => void prepare(true, trackItem(), "track-");

  function setTrack(frames, name) {
    track = normalizeRecording(frames);
    trackName = name;
    clearPreview();
    refresh();
  }

  async function finishRecording(current, automatic = false) {
    recordingBusy = true;
    callbacks.changed();
    try {
      const frames = current.real
        ? (await callbacks.stopRecording(current.session, current.id)).frames
        : current.frames;
      if (current.real) collectedId = current.id;
      if (recording === current) recording = undefined;
      setTrack(frames, current.real ? "真机示教" : "仿真录制");
      tell(automatic ? "达到录制帧数上限，轨迹已保留" : "轨迹录制完成", true);
    } catch (error) {
      tell(error.message);
    } finally {
      recordingBusy = false;
      callbacks.changed();
    }
  }

  function sampleSimulation(simulation) {
    const t = simulation.data.time - recording.origin;
    const frames = recording.frames;
    if (frames.length && t < frames.at(-1).t + 0.05) return;
    const { positions, gripper } = simulation.getActionPose();
    frames.push({ t, pos: positions, gripper_pos: gripper });
    if (frames.length >= MAX_RECORDING_FRAMES) void finishRecording(recording, true);
  }

  function recordingCount() {
    return "录制中 · " + (recording.real ? recording.count : recording.frames.length) + " 帧";
  }

  get("capture").onclick = async () => {
    if (recording) {
      await finishRecording(recording);
      return;
    }
    const context = callbacks.context();
    if (!context?.canRecord || locked() || recordingBusy) return;
    clearPreview();
    tell("");
    if (!context.real) {
      recording = { real: false, origin: context.simulation.data.time, frames: [] };
      sampleSimulation(context.simulation);
      callbacks.changed();
      return;
    }
    recordingBusy = true;
    callbacks.changed();
    try {
      const status = await callbacks.startRecording(context.session);
      if (callbacks.context()?.session !== context.session) throw new Error("真机连接已结束");
      recording = { ...status, real: true, session: context.session };
    } catch (error) {
      tell(error.message);
    } finally {
      recordingBusy = false;
      callbacks.changed();
    }
  };

  get("import").onclick = () => get("file").click();
  get("file").onchange = async (event) => {
    const file = event.target.files[0];
    event.target.value = "";
    if (!file || recordingNow() || locked()) return;
    recordingBusy = true;
    callbacks.changed();
    try {
      if (file.size > 3_000_000) throw new Error("轨迹文件须小于 3 MB");
      const lines = (await file.text()).split(/\r?\n/);
      const frames = [];
      for (const [index, line] of lines.entries()) {
        if (!line.trim()) continue;
        try { frames.push(JSON.parse(line)); }
        catch { throw new Error("第 " + (index + 1) + " 行 JSON 格式无效"); }
      }
      setTrack(frames, file.name);
      tell("轨迹已导入", true);
    } catch (error) {
      tell(error.message);
    } finally {
      recordingBusy = false;
      callbacks.changed();
    }
  };
  get("export").onclick = () => {
    const blob = new Blob([track.map((frame) => JSON.stringify(frame)).join("\n") + "\n"],
      { type: "application/jsonl" });
    const url = URL.createObjectURL(blob);
    const download = document.createElement("a");
    download.href = url;
    download.download = "panthera-trajectory-" + Date.now() + ".jsonl";
    download.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };

  function updateRecording(status, session, disconnected = false) {
    if (disconnected && recording?.real) {
      recording = undefined;
      tell("真机连接中断，录制已结束");
      callbacks.changed();
    }
    if (!status || status.id === collectedId || recordingBusy || recording?.real === false) return;
    const wasRecording = recording?.id === status.id;
    if (!wasRecording && !status.active) return;
    if (!wasRecording) {
      recording = { ...status, real: true, session };
      callbacks.changed();
    } else {
      Object.assign(recording, status);
      refresh();
    }
    if (!status.active) void finishRecording(recording, true);
  }

  async function stop() {
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
  }
  for (const button of stopButtons) button.onclick = stop;

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
    active, locked, recordingNow, refresh, clearPreview, updateHardware, updateRecording,
    tick(simulation) {
      if (recording?.real === false) {
        sampleSimulation(simulation);
        if (recording && !trajectory.hidden) {
          const info = recordingCount();
          if (get("track-info").textContent !== info) get("track-info").textContent = info;
        }
      }
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
