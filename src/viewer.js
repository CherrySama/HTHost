import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js";

const UP = new THREE.Vector3(0, 1, 0);
const markerMaterial = { transparent: true, depthTest: false, depthWrite: false };

function meshGeometry(model, id) {
  const start = model.mesh_vertadr[id];
  const count = model.mesh_vertnum[id];
  const face = model.mesh_faceadr[id];
  const faces = model.mesh_facenum[id];
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(
      model.mesh_vert.subarray(start * 3, (start + count) * 3),
      3,
    ),
  );
  geometry.setIndex(
    Array.from(model.mesh_face.subarray(face * 3, (face + faces) * 3)),
  );
  return geometry;
}

function readQuaternion(target, values, id) {
  const n = id * 4;
  // MuJoCo stores quaternions as w, x, y, z.
  target.set(values[n + 1], values[n + 2], values[n + 3], values[n]);
}

function centerOfMassGeometry() {
  const sphere = new THREE.SphereGeometry(0.015, 24, 16);
  const geometry = sphere.toNonIndexed();
  sphere.dispose();
  const positions = geometry.getAttribute("position");
  const colors = new Float32Array(positions.count * 3);
  const dark = new THREE.Color(0x171717).toArray();
  const light = new THREE.Color(0xffffff).toArray();
  for (let i = 0; i < positions.count; i += 3) {
    // Color whole triangles by octant so the black and white patches stay crisp.
    const x = positions.getX(i) + positions.getX(i + 1) + positions.getX(i + 2);
    const y = positions.getY(i) + positions.getY(i + 1) + positions.getY(i + 2);
    const z = positions.getZ(i) + positions.getZ(i + 1) + positions.getZ(i + 2);
    const octant = Number(x < 0) + Number(y < 0) + Number(z < 0);
    const color = octant % 2 === 0 ? dark : light;
    colors.set(color, i * 3);
    colors.set(color, (i + 1) * 3);
    colors.set(color, (i + 2) * 3);
  }
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  return geometry;
}

export class Viewer {
  constructor(container) {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color("#f5f5f7");
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.appendChild(this.renderer.domElement);
    this.camera = new THREE.PerspectiveCamera(36, 1, 0.01, 30);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.dirty = true;
    this.controls.addEventListener("change", () => {
      this.dirty = true;
    });
    this.controls.enableDamping = true;
    this.controls.minDistance = 0.35;
    this.controls.maxDistance = 4;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.home();
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xa2a7b5, 2.5));
    const light = new THREE.DirectionalLight(0xffffff, 3.2);
    light.position.set(1, -1, 2);
    light.castShadow = true;
    light.shadow.mapSize.set(2048, 2048);
    Object.assign(light.shadow.camera, {
      left: -1,
      right: 1,
      top: 1,
      bottom: -1,
      near: 0.1,
      far: 5,
    });
    light.shadow.bias = -0.0001;
    light.shadow.normalBias = 0.002;
    this.scene.add(light);
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(200, 200),
      new THREE.MeshStandardMaterial({ color: "#f5f5f7", roughness: 0.95 }),
    );
    ground.position.z = -0.002;
    ground.receiveShadow = true;
    this.scene.add(ground);
    this.grid = new THREE.GridHelper(3, 30, "#ccd0d8", "#e1e3e8");
    this.grid.rotation.x = Math.PI / 2;
    this.grid.position.z = -0.001;
    this.grid.material.transparent = true;
    this.grid.material.opacity = 0.55;
    this.scene.add(this.grid);
    this.items = [];
    this.overlays = new Map();
    this.modelTranslucent = false;
    this.matrix = new THREE.Matrix4();
    this.jointAxis = new THREE.Vector3();
    this.highlightColor = new THREE.Color("#7db5ef");
    this.reducedMotion = matchMedia("(prefers-reduced-motion: reduce)").matches;
    new ResizeObserver(() => {
      const { width, height } = container.getBoundingClientRect();
      this.renderer.setSize(width, height);
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
      this.dirty = true;
    }).observe(container);
  }

  home() {
    this.dirty = true;
    this.camera.position.set(0.85, -1.0, 0.7);
    this.controls.target.set(0.12, 0, 0.22);
    this.controls.update();
  }

  load(model, highlightBodies = [], hingeJointType) {
    this.model = model;
    this.highlightBodies = highlightBodies;
    this.hingeJointType = hingeJointType;
    this.dirty = true;
    const geometries = new Map();
    for (let i = 0; i < model.ngeom; i++) {
      // Group 1 contains the original colored visual meshes; group 3 is collision-only.
      if (model.geom_group[i] !== 1 || model.geom_dataid[i] < 0) continue;
      const id = model.geom_dataid[i];
      if (!geometries.has(id)) {
        const geometry = meshGeometry(model, id);
        geometry.computeVertexNormals();
        geometries.set(id, geometry);
      }
      const color = new THREE.Color().setRGB(
        ...model.geom_rgba.slice(i * 4, i * 4 + 3),
        THREE.LinearSRGBColorSpace,
      );
      const mesh = new THREE.Mesh(
        geometries.get(id),
        new THREE.MeshStandardMaterial({
          color,
          metalness: 0.35,
          roughness: 0.38,
        }),
      );
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.scene.add(mesh);
      this.items.push({
        id: i,
        bodyId: model.geom_bodyid[i],
        mesh,
        baseColor: mesh.material.color.clone(),
        baseEmissive: mesh.material.emissive.clone(),
        baseIntensity: mesh.material.emissiveIntensity,
      });
    }
    this.dirty = true;
  }

  setTargetPreview(data, actual) {
    if (!data) {
      if (this.targetPreview?.visible) {
        this.targetPreview.visible = false;
        this.dirty = true;
      }
      return;
    }

    if (!this.targetPreview) {
      const material = new THREE.MeshBasicMaterial({
        color: "#367fde",
        transparent: true,
        opacity: 0.38,
        depthWrite: false,
      });
      this.targetPreview = new THREE.Group();
      this.targetPreview.name = "target-preview";
      for (const { mesh } of this.items) {
        this.targetPreview.add(new THREE.Mesh(mesh.geometry, material));
      }
      this.scene.add(this.targetPreview);
    }

    const positions = data.geom_xpos;
    const rotations = data.geom_xmat;
    const actualPositions = actual.geom_xpos;
    const actualRotations = actual.geom_xmat;
    this.items.forEach(({ id }, index) => {
      const mesh = this.targetPreview.children[index];
      const moved = positions.subarray(id * 3, id * 3 + 3)
        .some((value, offset) => Math.abs(value - actualPositions[id * 3 + offset]) > 1e-5)
        || rotations.subarray(id * 9, id * 9 + 9)
          .some((value, offset) => Math.abs(value - actualRotations[id * 9 + offset]) > 1e-5);
      mesh.visible = moved;
      if (moved) {
        mesh.position.fromArray(positions, id * 3);
        this.setRotation(mesh, rotations, id);
      }
    });
    this.targetPreview.visible = true;
    this.dirty = true;
  }

  setOverlayVisible(name, visible) {
    let group = this.overlays.get(name);
    if (!group && visible) {
      group = new THREE.Group();
      group.name = name;
      switch (name) {
        case "collision":
          this.addCollisionOverlay(group);
          break;
        case "jointAxes":
          this.addJointAxes(group);
          break;
        default:
          this.addBodyOverlay(group, name);
      }
      this.overlays.set(name, group);
      this.scene.add(group);
    }
    if (group) {
      group.visible = visible;
      this.updateModelTransparency();
      this.dirty = true;
    }
  }

  updateModelTransparency() {
    const translucent = ["bodyAxes", "jointAxes", "centerOfMass"].some(
      (name) => this.overlays.get(name)?.visible,
    );
    if (translucent === this.modelTranslucent) return;
    this.modelTranslucent = translucent;
    for (const { mesh } of this.items) {
      mesh.material.transparent = translucent;
      mesh.material.opacity = translucent ? 0.4 : 1;
      mesh.material.depthWrite = !translucent;
      mesh.material.needsUpdate = true;
      mesh.castShadow = !translucent;
    }
  }

  addCollisionOverlay(group) {
    const model = this.model;
    const meshIds = model.geom_dataid;
    const bodyIds = model.geom_bodyid;
    const groups = model.geom_group;
    const positions = model.geom_pos;
    const rotations = model.geom_quat;
    const batches = new Map();
    const position = new THREE.Vector3();
    const rotation = new THREE.Quaternion();
    const transform = new THREE.Matrix4();
    const scale = new THREE.Vector3(1, 1, 1);

    for (let id = 0; id < model.ngeom; id++) {
      if (groups[id] !== 3 || meshIds[id] < 0) continue;
      const bodyId = bodyIds[id];
      const mesh = meshGeometry(model, meshIds[id]);
      // Suppress triangulation edges inside nearly coplanar surfaces.
      const wireframe = new THREE.EdgesGeometry(mesh, 12);
      mesh.dispose();
      // Bake the geometry's local pose so each body needs only one transform.
      position.fromArray(positions, id * 3);
      readQuaternion(rotation, rotations, id);
      wireframe.applyMatrix4(transform.compose(position, rotation, scale));
      if (!batches.has(bodyId)) batches.set(bodyId, []);
      batches.get(bodyId).push(wireframe);
    }

    const material = new THREE.LineBasicMaterial({
      color: 0x007b86,
      transparent: true,
      opacity: 0.28,
      depthTest: false,
      depthWrite: false,
    });
    for (const [bodyId, wireframes] of batches) {
      const lines = new THREE.LineSegments(mergeGeometries(wireframes), material);
      wireframes.forEach((geometry) => geometry.dispose());
      lines.userData.bodyId = bodyId;
      lines.renderOrder = 1;
      group.add(lines);
    }
  }

  addBodyOverlay(group, name) {
    const { nbody, body_mass: masses, body_inertia: moments } = this.model;
    let createMarker;
    switch (name) {
      case "bodyAxes": {
        group.renderOrder = 3;
        const length = 0.18;
        const shaft = new THREE.CylinderGeometry(0.004, 0.004, length, 10);
        shaft.translate(0, length / 2, 0);
        const directions = [new THREE.Vector3(1, 0, 0), UP, new THREE.Vector3(0, 0, 1)];
        const materials = [0xf02830, 0x00ae39, 0x1758e6].map(
          (color) => new THREE.MeshBasicMaterial({ ...markerMaterial, color }),
        );
        createMarker = () => {
          const axes = new THREE.Group();
          directions.forEach((direction, index) => {
            const axis = new THREE.Mesh(shaft, materials[index]);
            axis.quaternion.setFromUnitVectors(UP, direction);
            axes.add(axis);
          });
          return axes;
        };
        break;
      }
      case "centerOfMass": {
        group.renderOrder = 5;
        const geometry = centerOfMassGeometry();
        const material = new THREE.MeshBasicMaterial({
          ...markerMaterial,
          vertexColors: true,
        });
        createMarker = () => new THREE.Mesh(geometry, material);
        break;
      }
      case "inertia": {
        const sphere = new THREE.SphereGeometry(1, 16, 10);
        const geometry = new THREE.WireframeGeometry(sphere);
        sphere.dispose();
        const material = new THREE.LineBasicMaterial({
          color: 0x6334b3,
          transparent: true,
          opacity: 0.55,
          depthTest: false,
          depthWrite: false,
        });
        createMarker = (id) => {
          const x = moments[id * 3];
          const y = moments[id * 3 + 1];
          const z = moments[id * 3 + 2];
          // Invert the principal moments of a uniform solid ellipsoid.
          const factor = 5 / (2 * masses[id]);
          const ellipsoid = new THREE.LineSegments(geometry, material);
          ellipsoid.scale.set(
            Math.sqrt(Math.max(0, factor * (y + z - x))),
            Math.sqrt(Math.max(0, factor * (z + x - y))),
            Math.sqrt(Math.max(0, factor * (x + y - z))),
          );
          return ellipsoid;
        };
        break;
      }
    }

    // Body 0 is the world and does not represent a robot link.
    for (let id = 1; id < nbody; id++) {
      if (name !== "bodyAxes" && masses[id] <= 0) continue;
      const marker = createMarker(id);
      marker.userData.bodyId = id;
      marker.renderOrder = group.renderOrder || 2;
      group.add(marker);
    }
  }

  addJointAxes(group) {
    const shaftLength = 0.125;
    const headLength = 0.028;
    const shaft = new THREE.CylinderGeometry(0.005, 0.005, shaftLength, 10);
    shaft.translate(0, shaftLength / 2, 0);
    const head = new THREE.ConeGeometry(0.011, headLength, 12);
    head.translate(0, shaftLength + headLength / 2, 0);
    const red = new THREE.MeshBasicMaterial({ ...markerMaterial, color: 0xf02830 });
    const green = new THREE.MeshBasicMaterial({ ...markerMaterial, color: 0x00ae39 });

    const radius = 0.065;
    const sweep = Math.PI * 1.55;
    const arc = new THREE.TorusGeometry(radius, 0.003, 8, 48, sweep);
    // Rotate the XY arc to follow the right-hand rule around local +Y.
    arc.rotateX(-Math.PI / 2);
    const arcHeadLength = 0.021;
    const arcHead = new THREE.ConeGeometry(0.008, arcHeadLength, 12);
    const arcEnd = new THREE.Vector3(radius * Math.cos(sweep), 0, -radius * Math.sin(sweep));
    const tangent = new THREE.Vector3(-Math.sin(sweep), 0, -Math.cos(sweep));
    const types = this.model.jnt_type;

    for (let id = 0; id < this.model.njnt; id++) {
      if (types[id] !== this.hingeJointType) continue;
      const arrow = new THREE.Group();
      arrow.userData.jointId = id;
      arrow.renderOrder = 4;
      arrow.add(new THREE.Mesh(shaft, red), new THREE.Mesh(head, red));
      const tip = new THREE.Mesh(arcHead, green);
      tip.quaternion.setFromUnitVectors(UP, tangent);
      tip.position.copy(arcEnd).addScaledVector(tangent, arcHeadLength / 3);
      arrow.add(new THREE.Mesh(arc, green), tip);
      group.add(arrow);
    }
  }

  updateOverlays(data, bodyPositions, bodyRotations) {
    for (const [name, group] of this.overlays) {
      if (!group.visible) continue;
      if (name === "jointAxes") {
        const anchors = data.xanchor;
        const axes = data.xaxis;
        for (const arrow of group.children) {
          const id = arrow.userData.jointId;
          arrow.position.fromArray(anchors, id * 3);
          arrow.quaternion.setFromUnitVectors(UP, this.jointAxis.fromArray(axes, id * 3));
        }
        continue;
      }

      const inertial = name === "inertia" || name === "centerOfMass";
      const positions = inertial ? data.xipos : bodyPositions;
      const inertialRotations = name === "inertia" ? data.ximat : null;
      for (const marker of group.children) {
        const id = marker.userData.bodyId;
        marker.position.fromArray(positions, id * 3);
        if (!inertial) readQuaternion(marker.quaternion, bodyRotations, id);
        if (inertialRotations) this.setRotation(marker, inertialRotations, id);
      }
    }
  }

  setRotation(object, values, id) {
    const n = id * 9;
    this.matrix.set(
      values[n], values[n + 1], values[n + 2], 0,
      values[n + 3], values[n + 4], values[n + 5], 0,
      values[n + 6], values[n + 7], values[n + 8], 0,
      0, 0, 0, 1,
    );
    object.quaternion.setFromRotationMatrix(this.matrix);
  }

  highlightJoint(jointId, held = false) {
    if (!this.model || jointId < 0 || jointId >= this.model.njnt) return;
    if (this.highlight?.jointId !== jointId) {
      this.clearJointHighlight();
      const bodyId = this.model.jnt_bodyid[jointId];
      this.highlight = {
        jointId,
        bodyId,
        held: false,
        items: this.items.filter((item) =>
          (this.highlightBodies[jointId] || [bodyId]).includes(item.bodyId),
        ),
      };
    }
    this.highlight.held ||= held;
    this.highlight.expiresAt = performance.now() + 800;
    this.dirty = true;
  }

  endJointInteraction() {
    if (!this.highlight?.held) return;
    this.highlight.held = false;
    this.highlight.expiresAt = performance.now() + 800;
  }

  clearJointHighlight() {
    if (this.highlight) {
      for (const item of this.highlight.items) {
        item.mesh.material.color.copy(item.baseColor);
        item.mesh.material.emissive.copy(item.baseEmissive);
        item.mesh.material.emissiveIntensity = item.baseIntensity;
      }
      this.highlight = null;
    }
    this.dirty = true;
  }

  updateJointHighlight(now) {
    const active = this.highlight;
    if (!active) return;
    if (!active.held && now >= active.expiresAt) {
      this.clearJointHighlight();
      return;
    }
    const remaining = Math.min(1, (active.expiresAt - now) / 650);
    const strength =
      active.held || this.reducedMotion
        ? 1
        : remaining * remaining * (3 - 2 * remaining);
    for (const item of active.items) {
      item.mesh.material.color
        .copy(item.baseColor)
        .lerp(this.highlightColor, 0.35 * strength);
      item.mesh.material.emissive.copy(this.highlightColor);
      item.mesh.material.emissiveIntensity = 0.12 * strength;
    }
    this.dirty = true;
  }

  render(data) {
    const now = performance.now();
    this.updateJointHighlight(now);
    this.controls.update();
    const bodyPositions = data?.xpos;
    const bodyRotations = data?.xquat;
    if (data) {
      if (
        !this.lastBodyPositions ||
        bodyPositions.some((value, i) => Math.abs(value - this.lastBodyPositions[i]) > 1e-7) ||
        bodyRotations.some((value, i) => Math.abs(value - this.lastBodyRotations[i]) > 1e-7)
      ) {
        this.lastBodyPositions = bodyPositions.slice();
        this.lastBodyRotations = bodyRotations.slice();
        this.dirty = true;
      }
    }
    // Keep a low-rate refresh for browsers that discard an idle WebGL drawing buffer.
    if (!this.dirty && now - (this.lastRender || 0) < 250) return;
    if (data) {
      const positions = data.geom_xpos;
      const rotations = data.geom_xmat;
      this.items.forEach(({ id, mesh }) => {
        mesh.position.fromArray(positions, id * 3);
        this.setRotation(mesh, rotations, id);
      });
      this.updateOverlays(data, bodyPositions, bodyRotations);
    }
    this.renderer.render(this.scene, this.camera);
    this.lastRender = now;
    this.dirty = this.renderer.info.render.calls === 0;
  }
}
