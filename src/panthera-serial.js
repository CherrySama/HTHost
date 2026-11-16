const USB_FILTER = { usbVendorId: 0xcaf1, usbProductId: 0xffff };
const BOARD_SETUP = 0x05;
const MOTOR_STATE = 0x06;
const MOTOR_COUNT = 7;
const POLL_MS = 100;
const FEEDBACK_TIMEOUT_MS = 2000;
const ANGLE_UNIT = 2 * Math.PI / 10000;
const VELOCITY_UNIT = 2 * Math.PI / 4000;
// Follower motor types, in ID order, using the SDK's torque coefficients.
const TORQUE_UNITS = [0.4938, 0.6770, 0.6770, 0.4938, 0.5256, 0.5256, 0.4581]
  .map((value) => value * 0.01);
const JOINT_LIMITS = [[-2.7, 2.7], [-0.1, 3.14], [-0.1, 4.2], [-2.1, 1.6], [-1.7, 1.7], [-2.7, 2.7]];

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

function packet(command, payload) {
  const header = [command, payload.length, 0];
  const checksum = crc(payload, 0xffff, 0x8408);
  return Uint8Array.of(0xf7, ...header, crc(header, 0xff, 0x8c), checksum & 255, checksum >>> 8, ...payload);
}

const configureBoard = packet(BOARD_SETUP, [MOTOR_COUNT, 0]);
const requestState = packet(MOTOR_STATE, [0x7f]);

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

function decodeMotors(payload) {
  const motors = new Map();
  if (!payload.length || payload.length % 7) return motors;
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  for (let offset = 0; offset < payload.length; offset += 7) {
    const id = payload[offset];
    if (id < 1 || id > MOTOR_COUNT || motors.has(id)) continue;
    motors.set(id, {
      position: view.getInt16(offset + 1, true) * ANGLE_UNIT,
      velocity: view.getInt16(offset + 3, true) * VELOCITY_UNIT,
      torque: view.getInt16(offset + 5, true) * TORQUE_UNITS[id - 1],
      target: null,
    });
  }
  return motors;
}

class PantheraSerialPort {
  constructor(port, onStatus) {
    this.port = port;
    this.onStatus = onStatus;
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
      this.timer = setInterval(() => this.poll(), POLL_MS);
      this.poll();
    } catch {
      await this.close("无法打开 Panthera 串口，请检查权限和串口占用");
    }
  }

  handleDisconnect = () => void this.close("USB 已断开，保留最后一次反馈");

  poll() {
    if (performance.now() - this.lastFeedback > FEEDBACK_TIMEOUT_MS) {
      const message = this.connected ? "电机反馈已中断"
        : this.configured ? "未收到全部七个电机的反馈，请检查机械臂主电源"
        : "控制板未响应，请检查 USB 连接和串口选择";
      void this.close(message);
      return;
    }
    if (this.writing) return;
    this.writing = true;
    this.writer.write(this.configured ? requestState : configureBoard)
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
    if (command === BOARD_SETUP && payload.length >= 4 && payload[2] >= 3) {
      if (!this.configured) this.lastFeedback = performance.now();
      this.configured = true;
    }
    if (!this.configured || command !== MOTOR_STATE) return;
    const states = decodeMotors(payload);
    if (states.size !== MOTOR_COUNT) return;
    const motors = Array.from({ length: MOTOR_COUNT }, (_, index) => states.get(index + 1));
    this.connected = true;
    this.lastFeedback = performance.now();
    this.onStatus({
      state: "connected", message: "已连接 · USB 只读", readOnly: true,
      session: this.session, sampleTime: (this.lastFeedback - this.started) / 1000,
      motors, positions: motors.slice(0, 6).map((motor) => motor.position),
      gripper: motors[6].position, limits: JOINT_LIMITS, gripperLimits: [0, 2], maxSpeed: 1,
    });
  }

  close(message) {
    this.closing ??= this.release(message);
    return this.closing;
  }

  async release(message) {
    clearInterval(this.timer);
    this.port?.removeEventListener("disconnect", this.handleDisconnect);
    const readerClosed = this.reader?.cancel().catch(() => {});
    // Releasing the writer rejects pending observations without waiting on a stalled write.
    this.writer?.releaseLock();
    await readerClosed;
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
    for (const port of ports) {
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

  close() {
    return this.link?.close();
  }
}
