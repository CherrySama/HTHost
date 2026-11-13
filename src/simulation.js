import loadMujoco from "@mujoco/mujoco";
import wasmUrl from "@mujoco/mujoco/mujoco.wasm?url";
import sceneUrl from "../models/panthera.xml?url";
import { GRIPPER_CALIBRATION, gripperOpening } from "./gripper-calibration.js";
import { solveTcpPose, solveTcpPosition } from "./tcp-ik.js";
import { Quaternion } from "three";

const assets = import.meta.glob(
  "../Panthera-HT_SDK/panthera_python/Panthera-HT_description_V1.1.0/mujoco/meshes/obj/*.obj",
  {
    query: "?url",
    import: "default",
    eager: true,
  },
);

const collisionAssets = import.meta.glob("../models/collision/*.obj", {
  query: "?url",
  import: "default",
  eager: true,
});
const contactAssets = import.meta.glob("../models/contact/*.obj", {
  query: "?url",
  import: "default",
  eager: true,
});

export class Simulation {
  async init(onProgress) {
    onProgress("正在启动 MuJoCo…");
    this.mj = await loadMujoco({ locateFile: () => wasmUrl });
    const fs = this.mj.FS;
    fs.mkdirTree("/panthera/meshes/obj");
    fs.mkdirTree("/panthera/collision");
    fs.mkdirTree("/panthera/contact");
    const response = await fetch(sceneUrl);
    if (!response.ok) throw new Error("场景文件加载失败");
    const xml = await response.text();
    fs.writeFile("/panthera/scene.xml", xml);
    const urls = new Map(
      Object.entries({ ...assets, ...collisionAssets, ...contactAssets }).map(
        ([path, url]) => {
          const directory = path.includes("/models/contact/")
            ? "contact"
            : path.includes("/models/collision/")
              ? "collision"
              : "meshes/obj";
          return [`${directory}/${path.split("/").pop()}`, url];
        },
      ),
    );
    const required = [
      ...new Set(
        Array.from(
          new DOMParser()
            .parseFromString(xml, "application/xml")
            .querySelectorAll("asset mesh[file]"),
          (mesh) => mesh.getAttribute("file"),
        ),
      ),
    ];
    let loaded = 0;
    await Promise.all(
      required.map(async (path) => {
        const url = urls.get(path);
        if (!url) throw new Error(`缺少模型资源：${path}`);
        const res = await fetch(url);
        if (!res.ok) throw new Error(`网格加载失败：${path}`);
        fs.writeFile(
          `/panthera/${path}`,
          new Uint8Array(await res.arrayBuffer()),
        );
        onProgress(`正在加载模型 · ${++loaded} / ${required.length}`);
      }),
    );
    onProgress("正在编译机械臂模型…");
    this.model = this.mj.MjModel.from_xml_path("/panthera/scene.xml");
    this.data = new this.mj.MjData(this.model);
    this.joints = Array.from({ length: 6 }, (_, i) => {
      const id = this.mj.mj_name2id(
        this.model,
        this.mj.mjtObj.mjOBJ_JOINT.value,
        `joint${i + 1}`,
      );
      if (id < 0) throw new Error(`模型缺少 joint${i + 1}`);
      return {
        id,
        q: this.model.jnt_qposadr[id],
        v: this.model.jnt_dofadr[id],
        min: this.model.jnt_range[2 * id],
        max: this.model.jnt_range[2 * id + 1],
        actuator: this.mj.mj_name2id(
          this.model,
          this.mj.mjtObj.mjOBJ_ACTUATOR.value,
          `joint${i + 1}_torque`,
        ),
      };
    });
    this.targets = new Float64Array(6);
    const bodyType = this.mj.mjtObj.mjOBJ_BODY.value;
    this.tcpBodies = ["link6", "gripper_center_link"].map((name) =>
      this.mj.mj_name2id(this.model, bodyType, name),
    );
    this.gripperJoints = ["L", "R"].map((side) => {
      const id = this.mj.mj_name2id(
        this.model,
        this.mj.mjtObj.mjOBJ_JOINT.value,
        `gripper_${side}_joint`,
      );
      return {
        id,
        q: this.model.jnt_qposadr[id],
        v: this.model.jnt_dofadr[id],
        actuator: this.mj.mj_name2id(
          this.model,
          this.mj.mjtObj.mjOBJ_ACTUATOR.value,
          `gripper_${side}_joint_force`,
        ),
      };
    });
    this.highlightBodies = this.joints.map((joint) => [
      this.model.jnt_bodyid[joint.id],
    ]);
    this.highlightBodies[5].push(
      this.mj.mj_name2id(
        this.model,
        this.mj.mjtObj.mjOBJ_BODY.value,
        "gripper_center_link",
      ),
    );
    this.gripperHighlightJointId = this.gripperJoints[0].id;
    this.highlightBodies[this.gripperHighlightJointId] = this.gripperJoints.map(
      (joint) => this.model.jnt_bodyid[joint.id],
    );
    this.paused = false;
    this.reset();
  }

  reset() {
    this.accumulator = 0;
    this.mj.mj_resetData(this.model, this.data);
    this.targets.fill(0);
    this.gripperTarget = 0;
    this.writeGripperPose(this.data, this.gripperTarget);
    this.mj.mj_forward(this.model, this.data);
  }

  setTarget(i, radians) {
    if (!Number.isFinite(radians)) return;
    this.targets[i] = Math.max(
      this.joints[i].min,
      Math.min(this.joints[i].max, radians),
    );
  }

  setGripperTarget(angle) {
    if (Number.isFinite(angle))
      this.gripperTarget = Math.max(0, Math.min(2, angle));
  }

  setRobotPose(positions, gripper) {
    this.robotData ??= new this.mj.MjData(this.model);
    this.writePose(this.robotData, positions, gripper);
    this.mj.mj_forward(this.model, this.robotData);
  }

  setTargetPreview(positions, gripper) {
    this.previewData ??= new this.mj.MjData(this.model);
    this.writePose(this.previewData, positions, gripper);
    this.mj.mj_kinematics(this.model, this.previewData);
    return this.previewData;
  }

  setActionPreviewStart(pose) {
    this.actionPreviewStart ??= new this.mj.MjData(this.model);
    this.writePose(this.actionPreviewStart, pose.positions, pose.gripper);
    this.mj.mj_kinematics(this.model, this.actionPreviewStart);
    return this.actionPreviewStart;
  }

  getTcpPosition(data, hasGripper = true) {
    const offset = this.tcpBodies[Number(hasGripper)] * 3;
    return Array.from(data.xpos.subarray(offset, offset + 3));
  }

  getActionPose() {
    const opening = this.getGripperState().opening;
    const index = GRIPPER_CALIBRATION.findIndex(([, width]) => width >= opening);
    let gripper = index === -1 ? GRIPPER_CALIBRATION.at(-1)[0] : 0;
    if (index > 0) {
      const [angle, width] = GRIPPER_CALIBRATION[index];
      const [lower, lowerWidth] = GRIPPER_CALIBRATION[index - 1];
      gripper = lower + (angle - lower) * (opening - lowerWidth) / (width - lowerWidth);
    }
    return { positions: this.joints.map(({ q }) => this.data.qpos[q]), gripper };
  }

  actionPoseSettled(goal) {
    const current = this.getActionPose();
    return this.joints.every(({ v }, i) => Math.abs(current.positions[i] - goal.positions[i]) <= 0.015
      && Math.abs(this.data.qvel[v]) <= 0.035)
      && (goal.gripper == null || (Math.abs(gripperOpening(current.gripper) - gripperOpening(goal.gripper)) <= 1
        && this.gripperJoints.every(({ v }) => Math.abs(this.data.qvel[v]) <= 0.005)));
  }

  actionTcp(angles, hasGripper) {
    this.ikData ??= new this.mj.MjData(this.model);
    const data = this.ikData;
    this.writePose(data, angles);
    this.mj.mj_kinematics(this.model, data);
    const index = this.tcpBodies[Number(hasGripper)] * 4;
    const [w, x, y, z] = data.xquat.subarray(index, index + 4);
    return { position: this.getTcpPosition(data, hasGripper), rotation: new Quaternion(x, y, z, w) };
  }

  solveActionTcp(goal, seed, limits, hasGripper) {
    const rotationWeight = 0.25;
    return solveTcpPose((angles) => {
      const { position, rotation } = this.actionTcp(angles, hasGripper);
      const data = this.ikData;
      const delta = goal.rotation.clone().multiply(rotation.invert());
      if (delta.w < 0) delta.set(-delta.x, -delta.y, -delta.z, -delta.w);
      const length = Math.hypot(delta.x, delta.y, delta.z);
      const scale = rotationWeight * (length < 1e-9 ? 2 : 2 * Math.atan2(length, delta.w) / length);
      const error = [...goal.position.map((value, i) => value - position[i]),
        delta.x * scale, delta.y * scale, delta.z * scale];
      const columns = this.joints.map(({ id }) => {
        const offset = id * 3;
        const [x, y, z] = position.map((value, i) => value - data.xanchor[offset + i]);
        const [ax, ay, az] = data.xaxis.subarray(offset, offset + 3);
        return [ay * z - az * y, az * x - ax * z, ax * y - ay * x,
          ax * rotationWeight, ay * rotationWeight, az * rotationWeight];
      });
      return { error, columns };
    }, seed, limits);
  }

  solveTcp(target, initial, limits, hasGripper) {
    this.ikData ??= new this.mj.MjData(this.model);
    const data = this.ikData;
    return solveTcpPosition((angles) => {
      this.writePose(data, angles);
      this.mj.mj_kinematics(this.model, data);
      const position = this.getTcpPosition(data, hasGripper);
      const anchors = data.xanchor;
      const axes = data.xaxis;
      const columns = this.joints.map(({ id }) => {
        const offset = id * 3;
        const x = position[0] - anchors[offset];
        const y = position[1] - anchors[offset + 1];
        const z = position[2] - anchors[offset + 2];
        const [ax, ay, az] = axes.subarray(offset, offset + 3);
        return [ay * z - az * y, az * x - ax * z, ax * y - ay * x];
      });
      return { position, columns };
    }, initial, limits, target);
  }

  writePose(data, positions, gripper) {
    this.joints.forEach((joint, index) => {
      data.qpos[joint.q] = positions[index];
    });
    if (Number.isFinite(gripper)) {
      this.writeGripperPose(data, gripper);
    }
  }

  writeGripperPose(data, gripper) {
    const displacement = (gripperOpening(gripper) / 1000 - 0.082) / 2;
    data.qpos[this.gripperJoints[0].q] = displacement;
    data.qpos[this.gripperJoints[1].q] = -displacement;
  }

  getGripperState() {
    const left = this.data.qpos[this.gripperJoints[0].q];
    const right = this.data.qpos[this.gripperJoints[1].q];
    return {
      target: this.gripperTarget,
      targetOpening: gripperOpening(this.gripperTarget),
      opening: (0.082 + left - right) * 1000,
      left,
      right,
    };
  }

  getMotorTelemetry() {
    return {
      time: this.data.time,
      source: "simulation",
      joints: this.joints.map((joint, index) => ({
        name: `J${index + 1}`,
        target: this.targets[index],
        position: this.data.qpos[joint.q],
        velocity: this.data.qvel[joint.v],
        torque: this.data.qfrc_actuator[joint.v],
        error: this.targets[index] - this.data.qpos[joint.q],
        torqueLimit: Math.max(
          Math.abs(this.model.actuator_ctrlrange[joint.actuator * 2]),
          Math.abs(this.model.actuator_ctrlrange[joint.actuator * 2 + 1]),
        ),
      })),
    };
  }

  getSelfContacts() {
    const contacts = this.data.contact;
    const result = [];
    try {
      for (let i = 0; i < this.data.ncon; i++) {
        const contact = contacts.get(i);
        try {
          const a = this.model.geom_bodyid[contact.geom1];
          const b = this.model.geom_bodyid[contact.geom2];
          if (a > 0 && b > 0 && contact.efc_address >= 0) {
            result.push({ body1: a, body2: b, distance: contact.dist });
          }
        } finally {
          contact.delete();
        }
      }
    } finally {
      contacts.delete();
    }
    return result;
  }

  advance(seconds) {
    if (this.paused) return;
    const dt = this.model.opt.timestep;
    this.accumulator =
      (this.accumulator || 0) + Math.max(0, Math.min(seconds, 0.05));
    while (this.accumulator >= dt) {
      this.joints.forEach((joint, i) => {
        const kp = [110, 160, 130, 55, 35, 25][i];
        const kd = [12, 16, 13, 6, 4, 3][i];
        const torque =
          kp * (this.targets[i] - this.data.qpos[joint.q]) -
          kd * this.data.qvel[joint.v] +
          this.data.qfrc_bias[joint.v];
        const a = joint.actuator;
        this.data.ctrl[a] = Math.max(
          this.model.actuator_ctrlrange[a * 2],
          Math.min(this.model.actuator_ctrlrange[a * 2 + 1], torque),
        );
      });
      const displacement =
        (gripperOpening(this.gripperTarget) / 1000 - 0.082) / 2;
      this.gripperJoints.forEach((joint, index) => {
        const target = index === 0 ? displacement : -displacement;
        const force =
          800 * (target - this.data.qpos[joint.q]) -
          8 * this.data.qvel[joint.v] +
          this.data.qfrc_bias[joint.v];
        this.data.ctrl[joint.actuator] = Math.max(-5, Math.min(5, force));
      });
      this.mj.mj_step(this.model, this.data);
      this.accumulator -= dt;
    }
    this.mj.mj_forward(this.model, this.data);
  }
}
