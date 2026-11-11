import loadMujoco from "@mujoco/mujoco";
import wasmUrl from "@mujoco/mujoco/mujoco.wasm?url";
import sceneUrl from "../models/panthera.xml?url";
import { gripperOpening } from "./gripper-calibration.js";

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
    this.gripperTarget = 2;
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
