const USB_FILTER = { usbVendorId: 0xcaf1, usbProductId: 0xffff };
const BOARD_SETUP = 0x05;
const MOTOR_STATE = 0x06;
const EXTENDED_STATE = 0x0a;
const FUNCTION_VERSION = 0x0c;
const FDCAN_RESET = 0x0e;
const TORQUE_ADJUSTMENT = 0x10;
const MOTOR_COUNT = 7;
const FEEDBACK_TIMEOUT_MS = 2000;
const CONTROL_POLL_MS = 45;
const WATCHDOG_TIMEOUT_MS = 250;
const WATCHDOG_CHECK_MS = 1400;
const WATCHDOG_STILL_SAMPLES = 4;
const WATCHDOG_STILL_DRIFT = 0.005;
const CONTROL_FEEDBACK_MS = 140;
const MAX_SPEED = 1;
const RETURN_SPEED = 0.2;
const GRIPPER_MAX_SPEED = 0.3;
const POSITION_MODE = 10;
const TIMEOUT_MODE = 11;
const POSITION_TOLERANCE = 0.01;
const POSITION_RESPONSE_MS = 650;
const SETTLE_SPEED = 0.03;
const SETTLE_MS = 150;
// Firmware telemetry has no verification that an interrupted position target was cleared.
const USB_POSITION_CONTROL_VERIFIED = false;
const RECOVERY_MESSAGE = "USB 只读 · 尚未确认控制板旧目标已清除";
const GRIPPER_LIMITS = [0, 2];
const ANGLE_UNIT = 2 * Math.PI / 10000;
const VELOCITY_UNIT = 2 * Math.PI / 4000;
// Panthera-HT Follower SDK limits, with the bridge's gripper torque cap.
const TORQUE_LIMITS = [21, 36, 36, 21, 10, 10, 0.15];
const TORQUE_UNITS = [0.4938, 0.6770, 0.6770, 0.4938, 0.5256, 0.5256, 0.4581]
  .map((value) => value * 0.01);
const JOINT_LIMITS = [[-2.7, 2.7], [-0.1, 3.14], [-0.1, 4.2], [-2.1, 1.6], [-1.7, 1.7], [-2.7, 2.7]];
const LIMITS = [...JOINT_LIMITS, GRIPPER_LIMITS];

function motorName(index) {
  return index === MOTOR_COUNT - 1 ? "夹爪" : `J${index + 1}`;
}

function movingMotors(samples, reference) {
  return LIMITS.flatMap((_, i) => {
    const positions = samples.map((motors) => motors[i].position);
    if (reference) positions.push(reference[i]);
    const drift = Math.max(...positions) - Math.min(...positions);
    const fast = samples.slice(-3).every((motors) => Math.abs(motors[i].velocity) > SETTLE_SPEED);
    return drift > WATCHDOG_STILL_DRIFT || fast ? [motorName(i)] : [];
  });
}

function crc(bytes, initial, polynomial) {
  let value = initial;
  for (const byte of bytes) {
    value ^= byte;
    for (let bit = 0; bit < 8; bit++) {
      value = (value >>> 1) ^ (value & 1 ? polynomial : 0);
    }
  }
  return value;
}

export function encodePantheraPacket(command, payload) {
  const header = [command, payload.length, 0];
  const checksum = crc(payload, 0xffff, 0x8408);
  return Uint8Array.of(0xf7, ...header, crc(header, 0xff, 0x8c), checksum & 255, checksum >>> 8, ...payload);
}

const configureBoard = encodePantheraPacket(BOARD_SETUP, [MOTOR_COUNT, 0]);
// The SDK sends the reset using the same buffer as SET_NUM, retaining byte 0.
const resetCanChannel = encodePantheraPacket(FDCAN_RESET, [MOTOR_COUNT]);
const requestState = encodePantheraPacket(MOTOR_STATE, [0x7f]);
const requestExtendedState = encodePantheraPacket(EXTENDED_STATE, [0x7f]);
const configureControl = encodePantheraPacket(FUNCTION_VERSION, [5]);
const requestTorqueAdjustment = encodePantheraPacket(TORQUE_ADJUSTMENT, [0x7f]);
const configureWatchdog = encodePantheraPacket(0x85, Array.from({ length: MOTOR_COUNT },
  () => [WATCHDOG_TIMEOUT_MS & 255, WATCHDOG_TIMEOUT_MS >>> 8]).flat());

function positionPacket(goals, speeds, torqueUnits, probe = false) {
  const payload = new Uint8Array(MOTOR_COUNT * 6);
  const view = new DataView(payload.buffer);
  for (let i = 0; i < MOTOR_COUNT; i++) {
    const offset = i * 6;
    if (goals[i] === null) {
      // SDK sentinel leaves an uncommanded motor untouched.
      for (let field = offset; field < offset + 6; field += 2) view.setInt16(field, -32768, true);
      continue;
    }
    const [min, max] = LIMITS[i];
    const position = probe ? Math.round(goals[i] / ANGLE_UNIT)
      : Math.max(Math.ceil(min / ANGLE_UNIT), Math.min(Math.floor(max / ANGLE_UNIT), Math.round(goals[i] / ANGLE_UNIT)));
    view.setInt16(offset, position, true);
    view.setInt16(offset + 2, probe ? 0 : Math.floor(speeds[i] / VELOCITY_UNIT), true);
    view.setInt16(offset + 4, probe ? 0 : Math.floor(TORQUE_LIMITS[i] / torqueUnits[i]), true);
  }
  return encodePantheraPacket(0x90, payload);
}

function isPantheraPort(port) {
  const { usbVendorId, usbProductId } = port.getInfo();
  return usbVendorId === USB_FILTER.usbVendorId && usbProductId === USB_FILTER.usbProductId;
}

export class PantheraSerialDecoder {
  buffer = new Uint8Array();

  push(chunk, receive) {
    const bytes = new Uint8Array(this.buffer.length + chunk.length);
    bytes.set(this.buffer);
    bytes.set(chunk, this.buffer.length);
    let offset = 0;
    while (offset + 7 <= bytes.length) {
      const command = bytes[offset + 1];
      const length = bytes[offset + 2] + 256 * bytes[offset + 3];
      if (bytes[offset] !== 0xf7 || length > 1024 ||
          crc(bytes.subarray(offset + 1, offset + 4), 0xff, 0x8c) !== bytes[offset + 4]) {
        offset++;
        continue;
      }
      const end = offset + 7 + length;
      if (end > bytes.length) break;
      const payload = bytes.subarray(offset + 7, end);
      const checksum = bytes[offset + 5] + 256 * bytes[offset + 6];
      if (crc(payload, 0xffff, 0x8408) !== checksum) {
        offset++;
        continue;
      }
      receive(command, payload);
      offset = end;
    }
    this.buffer = bytes.slice(offset);
  }
}

export function decodePantheraMotors(payload, extended = false, torqueUnits = TORQUE_UNITS) {
  const motors = new Map();
  const stride = extended ? 9 : 7;
  if (!payload.length || payload.length % stride) return motors;
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  for (let offset = 0; offset < payload.length; offset += stride) {
    const id = payload[offset];
    if (id < 1 || id > MOTOR_COUNT || motors.has(id)) continue;
    const data = offset + (extended ? 3 : 1);
    motors.set(id, {
      position: view.getInt16(data, true) * ANGLE_UNIT,
      velocity: view.getInt16(data + 2, true) * VELOCITY_UNIT,
      torque: view.getInt16(data + 4, true) * torqueUnits[id - 1],
      target: null,
      ...(extended ? { driveMode: payload[offset + 1], fault: payload[offset + 2] } : {}),
    });
  }
  return motors;
}

class PantheraSerialPort {
  constructor(port, onStatus) {
    this.port = port;
    this.onStatus = onStatus;
    this.torqueFlags = Array(MOTOR_COUNT).fill(null);
    this.commandId = 0;
    this.canResetsPending = 0;
  }

  async connect() {
    try {
      await this.port.open({ baudRate: 4000000 });
      this.open = true;
      this.reader = this.port.readable.getReader();
      this.writer = this.port.writable.getWriter();
      this.session = crypto.randomUUID();
      this.started = this.lastFeedback = performance.now();
      this.decoder = new PantheraSerialDecoder();
      this.configured = false;
      this.connected = false;
      this.port.addEventListener("disconnect", this.handleDisconnect);
      void this.read();
      this.timer = setInterval(() => this.poll(), CONTROL_POLL_MS);
      this.poll();
    } catch {
      await this.close("无法打开 Panthera 串口，请检查权限和串口占用");
    }
  }

  handleDisconnect = () => void this.close("USB 已断开，保留最后一次反馈");

  get canControl() {
    return USB_POSITION_CONTROL_VERIFIED && this.extended && !this.legacy && !!this.torqueUnits;
  }

  freshMotors(now = performance.now()) {
    return this.motors && now - this.lastFeedback < CONTROL_FEEDBACK_MS &&
      this.motors.every((motor) => motor.fault === 0);
  }

  requireControl(session) {
    if (!USB_POSITION_CONTROL_VERIFIED) throw new Error(RECOVERY_MESSAGE);
    if (this.closing || session !== this.session || !this.canControl ||
        document.hidden || !this.freshMotors()) {
      throw new Error("真机控制不可用，请检查电机反馈和 USB 连接");
    }
  }

  setPosition({ session, positions, gripper, speed }) {
    this.requireControl(session);
    if (this.motion) throw new Error("当前运动尚未结束");
    if (!Number.isFinite(speed) || speed < 0.01 || speed > MAX_SPEED) {
      throw new Error("速度超出真机允许范围");
    }
    if (positions === undefined && gripper === undefined) throw new Error("缺少关节或夹爪目标");
    if (positions !== undefined && (!Array.isArray(positions) || positions.length !== MOTOR_COUNT - 1)) {
      throw new Error("需要六个有效的关节角度");
    }
    const goals = this.goals?.slice() ?? Array(MOTOR_COUNT).fill(null);
    const speeds = this.speeds?.slice() ?? Array(MOTOR_COUNT).fill(0);
    LIMITS.forEach(([min, max], i) => {
      const isGripper = i === MOTOR_COUNT - 1;
      if ((isGripper ? gripper : positions) === undefined) return;
      const target = isGripper ? gripper : positions[i];
      if (!Number.isFinite(target) || target < min || target > max) {
        throw new Error(`${isGripper ? "夹爪" : `J${i + 1}`}目标超出限位`);
      }
      goals[i] = target;
      speeds[i] = isGripper ? Math.min(speed, GRIPPER_MAX_SPEED) : speed;
    });
    this.startMotion("moving", goals, speeds);
    return { commandId: this.commandId, message: this.positionStatus.message };
  }

  pausePosition({ session, commandId }) {
    this.requireControl(session);
    if (this.motion?.phase !== "moving" || commandId !== this.commandId) {
      throw new Error("该位置指令已结束或正在停稳");
    }
    if (!this.watchdogReady) {
      // No position torque has been sent yet.
      this.motion = this.watchdogCheck = this.goals = this.speeds = undefined;
      this.positionStatus = { commandId: ++this.commandId, completed: true, message: "已取消位置指令" };
      return;
    }
    const goals = this.motors.map(({ position }, i) =>
      Math.max(LIMITS[i][0], Math.min(LIMITS[i][1], position)));
    this.startMotion("settling", goals, Array(MOTOR_COUNT).fill(RETURN_SPEED));
  }

  returnAndDisconnect() {
    if (!this.canControl) return this.close();
    try {
      this.requireControl(this.session);
      this.startMotion("homing", Array(MOTOR_COUNT).fill(0), Array(MOTOR_COUNT).fill(RETURN_SPEED));
    } catch (error) {
      return this.close(`回零未完成：${error.message}`);
    }
  }

  startMotion(phase, goals, speeds) {
    const now = performance.now();
    if (!this.watchdogReady && !this.watchdogCheck) {
      this.watchdogCheck = { started: now, writes: 0, samples: [], sawPosition: new Set() };
    }
    const travelMs = Math.max(...goals.map((goal, i) =>
      goal === null ? 0 : 1000 * Math.abs(goal - this.motors[i].position) /
        (Math.floor(speeds[i] / VELOCITY_UNIT) * VELOCITY_UNIT)));
    this.goals = goals;
    this.speeds = speeds;
    this.confirmedMotors = new Set();
    this.motion = { phase, deadline: now + Math.max(8000, travelMs + 5000) };
    this.positionStatus = { commandId: ++this.commandId, completed: false,
      message: this.watchdogReady ? "等待电机到位" : "正在确认设备端超时保护…" };
  }

  checkWatchdog(now) {
    const check = this.watchdogCheck;
    const enoughSamples = check.samples.length >= WATCHDOG_STILL_SAMPLES;
    const moving = enoughSamples ? movingMotors(check.samples, check.probePositions) : [];
    if (check.probedAt === undefined) {
      const inPositionMode = this.motors.flatMap(({ driveMode }, i) =>
        driveMode === POSITION_MODE ? [motorName(i)] : []);
      if (now - check.started > WATCHDOG_CHECK_MS) {
        const reason = !enoughSamples ? "稳定反馈帧不足"
          : moving.length ? `${moving.join("、")}未停稳`
          : inPositionMode.length ? `${inPositionMode.join("、")}仍处于位置模式`
          : "未能完成超时配置";
        void this.close(`设备端超时保护确认失败（探测前）：${reason}`);
        return;
      }
      if (!enoughSamples || moving.length || inPositionMode.length) return;
      if (check.writes < 5) {
        check.writes++;
        return configureWatchdog;
      }
      check.probePositions = this.motors.map(({ position }) => position);
      check.probedAt = now;
      return positionPacket(check.probePositions, null, null, true);
    }
    if (moving.length) {
      void this.close(`设备端超时保护确认失败（探测中）：检测到${moving.join("、")}移动`);
      return;
    }
    const missing = this.motors.flatMap((_, i) => check.sawPosition.has(i) ? [] : [motorName(i)]);
    const notTimedOut = this.motors.flatMap(({ driveMode }, i) =>
      check.sawPosition.has(i) && driveMode !== TIMEOUT_MODE ? [motorName(i)] : []);
    if (now - check.probedAt > WATCHDOG_CHECK_MS) {
      const reasons = [];
      if (missing.length) reasons.push(`${missing.join("、")}未反馈进入位置模式`);
      if (notTimedOut.length) reasons.push(`${notTimedOut.join("、")}未反馈进入超时模式`);
      void this.close(`设备端超时保护确认失败（探测后）：${reasons.join("；") || "未收到超时后的有效反馈"}`);
      return;
    }
    if (this.lastFeedback - check.probedAt >= WATCHDOG_TIMEOUT_MS && !missing.length && !notTimedOut.length) {
      this.watchdogReady = true;
      this.watchdogCheck = undefined;
      this.positionStatus.message = "等待电机到位";
    }
  }

  controlPacket(now) {
    if (!this.canControl || (!this.motion && !this.holding)) return;
    if (document.hidden || !this.freshMotors(now) ||
        (this.lastPositionWrite !== undefined && now - this.lastPositionWrite >= CONTROL_FEEDBACK_MS)) {
      void this.close("页面离开前台或电机反馈中断，已停止发送控制指令");
      return;
    }
    if (!this.watchdogReady) return this.checkWatchdog(now);

    if ([...this.confirmedMotors].some((i) => this.motors[i].driveMode !== POSITION_MODE)) {
      void this.close("电机已退出位置模式，已停止发送控制指令");
      return;
    }
    const motion = this.motion;
    if (motion) {
      motion.sentAt ??= now;
      if (now - motion.sentAt > POSITION_RESPONSE_MS &&
          this.goals.some((goal, i) => goal !== null && !this.confirmedMotors.has(i))) {
        void this.close("电机未确认进入位置模式，已停止发送控制指令");
        return;
      }
      if (now > motion.deadline) {
        void this.close("未能确认电机到位，已停止发送控制指令");
        return;
      }
      const reached = this.motors.every(({ position, velocity }, i) => this.goals[i] === null ||
        (this.confirmedMotors.has(i) && Math.abs(position - this.goals[i]) <= POSITION_TOLERANCE &&
          Math.abs(velocity) <= SETTLE_SPEED));
      if (!reached) motion.settledAt = undefined;
      else if (motion.settledAt === undefined) motion.settledAt = now;
      else if (now - motion.settledAt >= SETTLE_MS) {
        if (motion.phase === "homing") {
          void this.close();
          return;
        }
        this.positionStatus = { commandId: this.commandId, completed: true,
          message: motion.phase === "settling" ? "已停止，保持当前位置" : "已到位，保持目标位置" };
        this.motion = undefined;
        this.holding = true;
      }
    }
    this.lastPositionWrite = now;
    return positionPacket(this.goals, this.speeds, this.torqueUnits);
  }

  leavePage() {
    if (this.motion || this.holding) void this.close("页面已离开前台，已停止真机控制");
  }

  poll() {
    const now = performance.now();
    if (now - this.lastFeedback > FEEDBACK_TIMEOUT_MS) {
      const message = this.connected ? "电机反馈已中断"
        : this.configured ? "未收到全部七个电机的反馈，请检查机械臂主电源"
        : "控制板未响应，请检查 USB 连接和串口选择";
      void this.close(message);
      return;
    }
    if (this.writing) return;
    this.writing = true;
    let request;
    if (this.canResetsPending) {
      this.canResetsPending--;
      this.configuredAt = now;
      request = resetCanChannel;
    } else {
      if (this.configured && !this.connected && now - this.configuredAt > 650) this.legacy = true;
      request = !this.configured ? configureBoard : this.legacy ? requestState
        : this.extended ? requestExtendedState : configureControl;
    }
    if (request === requestExtendedState && !this.torqueUnits) {
      // Give each board query its own poll while detecting the firmware's torque scale.
      if (this.requestTorqueNext) request = requestTorqueAdjustment;
      this.requestTorqueNext = !this.requestTorqueNext;
    }
    const command = this.controlPacket(now);
    if (this.closing) return;
    const bytes = command ? Uint8Array.of(...command, ...request) : request;
    this.writer.write(bytes)
      .catch(() => this.close("USB 串口写入中断"))
      .finally(() => { this.writing = false; });
  }

  async read() {
    try {
      while (!this.closing) {
        const { value, done } = await this.reader.read();
        if (done) break;
        this.decoder.push(value, (command, payload) => this.receive(command, payload));
      }
    } catch {
      // Report both an interrupted stream and a disconnected port as lost feedback.
    }
    if (!this.closing) void this.close("USB 串口反馈已中断");
  }

  receive(command, payload) {
    if (this.closing) return;
    if (command === BOARD_SETUP && payload.length >= 4 && payload[2] >= 3 && !this.configured) {
      this.configuredAt = this.lastFeedback = performance.now();
      this.configured = true;
      // The SDK initializes the CAN channel on control-board firmware 4.1.0 and newer.
      if (payload[2] > 4 || (payload[2] === 4 && payload[3] >= 1)) this.canResetsPending = 3;
    }
    if (!this.configured || this.canResetsPending) return;
    if (command === FUNCTION_VERSION && payload[0] === 5) this.extended = true;
    if (command === TORQUE_ADJUSTMENT && payload.length % 2 === 0) {
      // SDK response is a sequence of (motor ID, adjustment flag) pairs.
      for (let i = 0; i < payload.length; i += 2) {
        const id = payload[i];
        const flag = payload[i + 1];
        if (id >= 1 && id <= MOTOR_COUNT && (flag === 0 || flag === 1)) this.torqueFlags[id - 1] = flag;
      }
      if (this.torqueFlags.every((flag) => flag !== null)) {
        this.torqueUnits = this.torqueFlags.map((flag, i) => flag === 1 ? 0.01 : TORQUE_UNITS[i]);
      }
    }
    if ((!this.legacy && !this.extended) || command !== (this.legacy ? MOTOR_STATE : EXTENDED_STATE)) return;
    const states = decodePantheraMotors(payload, !this.legacy, this.torqueUnits);
    if (states.size !== MOTOR_COUNT) return;
    const motors = Array.from({ length: MOTOR_COUNT }, (_, i) => {
      const motor = states.get(i + 1);
      if (this.lastPositionWrite !== undefined && this.goals?.[i] != null) {
        if (motor.driveMode === POSITION_MODE) this.confirmedMotors.add(i);
        motor.target = this.goals[i];
      }
      return motor;
    });
    this.connected = true;
    this.lastFeedback = performance.now();
    this.motors = motors;
    const check = this.watchdogCheck;
    if (check) {
      check.samples.push(motors);
      if (check.samples.length > WATCHDOG_STILL_SAMPLES) check.samples.shift();
      if (check.probedAt !== undefined && this.lastFeedback - check.probedAt < WATCHDOG_TIMEOUT_MS) {
        motors.forEach(({ driveMode }, i) => {
          if (driveMode === POSITION_MODE) check.sawPosition.add(i);
        });
      }
    }
    const homing = this.motion?.phase === "homing";
    this.onStatus({
      state: homing ? "homing" : "connected",
      message: homing ? "正在回零并断开…" : this.canControl ? "已连接 · USB 直连" : "已连接 · USB 只读",
      readOnly: !this.canControl, usbDirect: true, mode: "position",
      controlUnavailable: !USB_POSITION_CONTROL_VERIFIED ? RECOVERY_MESSAGE : undefined,
      positionPhase: this.canControl ? (homing ? "moving" : this.motion?.phase ?? null) : undefined,
      positionStatus: this.positionStatus,
      session: this.session, sampleTime: (this.lastFeedback - this.started) / 1000,
      motors, positions: motors.slice(0, -1).map((motor) => motor.position),
      gripper: motors[MOTOR_COUNT - 1].position, limits: JOINT_LIMITS, gripperLimits: GRIPPER_LIMITS, maxSpeed: MAX_SPEED,
    });
  }

  close(message) {
    if (message && this.motion?.phase === "homing" && !message.startsWith("回零未完成")) {
      message = `回零未完成：${message}`;
    }
    if (!this.closing) {
      this.closing = this.release(message);
      if (this.connected) this.onStatus({ state: "disconnecting", message: "正在关闭 USB 串口…" });
    }
    return this.closing;
  }

  async release(message) {
    clearInterval(this.timer);
    this.port?.removeEventListener("disconnect", this.handleDisconnect);
    this.goals = this.speeds = this.motion = this.positionStatus = this.watchdogCheck = undefined;
    this.holding = this.watchdogReady = false;
    this.lastPositionWrite = undefined;
    this.confirmedMotors?.clear();
    // Abort the stream before unlocking it so queued serial writes are discarded.
    await Promise.all([this.reader?.cancel().catch(() => {}), this.writer?.abort().catch(() => {})]);
    this.writer?.releaseLock();
    this.reader?.releaseLock();
    if (this.open) await this.port.close().catch(() => {});
    this.onStatus(message ? { state: "error", message } : { state: "disconnected" });
  }
}

export class PantheraSerial {
  constructor(onStatus) {
    this.onStatus = onStatus;
  }

  async connect() {
    const serial = navigator.serial;
    if (!serial) {
      this.onStatus({ state: "error", message: "USB 直连需要支持 Web Serial 的桌面浏览器" });
      return;
    }
    let ports;
    try {
      ports = (await serial.getPorts()).filter(isPantheraPort);
      if (!ports.length) {
        this.onStatus({
          state: "connecting",
          message: "请选择 Livelybot Communication Board，网页会自动识别控制接口",
        });
        const selected = await serial.requestPort({ filters: [USB_FILTER] });
        // Chrome may authorize several interfaces together. Probe the selected one first.
        const authorized = await serial.getPorts().catch(() => []);
        ports = [selected, ...authorized.filter((port) => port !== selected && isPantheraPort(port))];
      }
    } catch (error) {
      this.onStatus(error.name === "NotFoundError" ? { state: "disconnected" }
        : { state: "error", message: "无法获取串口授权，请检查浏览器的串口权限" });
      return;
    }

    let failure;
    for (const [index, port] of ports.entries()) {
      const progress = ports.length > 1 ? `（${index + 1}/${ports.length}）` : "";
      this.onStatus({ state: "connecting", message: `正在识别控制接口${progress}…` });
      const result = await this.probe(port);
      if (result.state === "connected") return;
      failure ??= result;
    }
    this.onStatus(ports.length === 1 ? failure : {
      state: "error", message: "已授权串口中未找到完整电机反馈，请检查供电、USB 和串口占用后重试",
    });
  }

  probe(port) {
    return new Promise((resolve) => {
      let connected = false;
      this.link = new PantheraSerialPort(port, (status) => {
        if (connected) {
          this.onStatus(status);
        } else {
          connected = status.state === "connected";
          if (connected) this.onStatus(status);
          resolve(status);
        }
      });
      void this.link.connect();
    });
  }

  disconnect() { return this.link.returnAndDisconnect(); }

  setPosition(payload) { return this.link.setPosition(payload); }

  pausePosition(payload) { return this.link.pausePosition(payload); }

  leavePage() { this.link?.leavePage(); }
}
