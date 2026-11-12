import scriptUrl from "../vendor/filament/filament.js?url";
import wasmUrl from "../vendor/filament/filament.wasm?url";
import surfaceUrl from "./filament-materials/surface.filamat?url";
import fadeUrl from "./filament-materials/surface-fade.filamat?url";
import overlayUrl from "./filament-materials/overlay.filamat?url";

let runtime;

function loadFilament() {
  runtime ??= new Promise((resolve, reject) => {
    const script = document.createElement("script");
    script.src = scriptUrl;
    script.onload = resolve;
    script.onerror = () => reject(new Error("Filament 脚本加载失败"));
    document.head.append(script);
  }).then(async () => {
    // The pinned browser build exposes its Emscripten factory as a global.
    // Pass Vite's asset URL to the factory and install the package's JS extensions.
    const factory = window.Filament;
    const filament = await factory({ locateFile: () => wasmUrl });
    window.Filament = Object.assign(filament, factory);
    filament.loadClassExtensions();
    return filament;
  });
  return runtime;
}

async function loadMaterial(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error("Filament 材质加载失败");
  return new Uint8Array(await response.arrayBuffer());
}

export class FilamentRenderer {
  static async create() {
    const [filament, ...materials] = await Promise.all([
      loadFilament(), loadMaterial(surfaceUrl), loadMaterial(fadeUrl), loadMaterial(overlayUrl),
    ]);
    return new FilamentRenderer(filament, materials);
  }

  constructor(filament, materials) {
    this.filament = filament;
    this.domElement = document.createElement("canvas");
    this.engine = filament.Engine.create(this.domElement);
    this.renderer = this.engine.createRenderer();
    this.renderer.setClearOptions({ clear: true, clearColor: [0.91, 0.91, 0.93, 1] });
    this.scene = this.engine.createScene();
    this.view = this.engine.createView();
    this.camera = this.engine.createCamera(filament.EntityManager.get().create());
    this.camera.setExposure(13, 1 / 250, 100);
    this.view.setCamera(this.camera);
    this.view.setScene(this.scene);
    this.view.setAntiAliasing(filament.View$AntiAliasing.FXAA);
    this.view.setColorGrading(filament.ColorGrading.Builder()
      .toneMapping(filament.ColorGrading$ToneMapping.LINEAR)
      .build(this.engine));
    this.swapChain = this.engine.createSwapChain();
    [this.surface, this.fade, this.overlay] = materials.map((bytes) => this.engine.createMaterial(bytes));
    this.transforms = this.engine.getTransformManager();
    this.renderables = this.engine.getRenderableManager();
    this.geometry = new WeakMap();
    this.objects = new WeakMap();
    this.visible = new Set();
    this.frameCount = 0;
    this.width = 1;
    this.height = 1;

    const ambient = filament.IndirectLight.Builder()
      .irradianceSh(1, new Float32Array([1, 1, 1]))
      .intensity(20000)
      .build(this.engine);
    this.scene.setIndirectLight(ambient);
    this.lights = [2048, 1024].map((mapSize) => {
      const light = filament.EntityManager.get().create();
      filament.LightManager.Builder(filament.LightManager$Type.DIRECTIONAL)
        .direction([-0.408, 0.408, -0.816])
        .intensity(95000)
        .shadowOptions({ mapSize, shadowFar: 4 })
        .castShadows(true)
        .build(this.engine, light);
      return light;
    });
    this.setRobotView(false);
  }

  setRobotView(robot) {
    if (robot === this.robotView) return false;
    if (this.robotView !== undefined) this.scene.remove(this.lights[Number(this.robotView)]);
    this.robotView = robot;
    this.scene.addEntity(this.lights[Number(robot)]);
    this.view.setMultiSampleAntiAliasingOptions({ enabled: !robot, sampleCount: 4 });
    this.setSize(this.width, this.height);
    return true;
  }

  setSize(width, height) {
    this.width = width;
    this.height = height;
    const ratio = Math.min(devicePixelRatio, this.robotView ? 1.25 : 2);
    const pixelWidth = Math.max(1, Math.round(width * ratio));
    const pixelHeight = Math.max(1, Math.round(height * ratio));
    if (this.domElement.width !== pixelWidth) this.domElement.width = pixelWidth;
    if (this.domElement.height !== pixelHeight) this.domElement.height = pixelHeight;
    this.view.setViewport([0, 0, pixelWidth, pixelHeight]);
  }

  uploadGeometry(geometry) {
    const cached = this.geometry.get(geometry);
    const position = geometry.getAttribute("position");
    if (cached) {
      if (cached.positionVersion !== position.version) {
        cached.vertices.setBufferAt(this.engine, 0, position.array);
        cached.positionVersion = position.version;
      }
      return cached;
    }

    const f = this.filament;
    const attributes = f.VertexAttribute;
    const types = f.VertexBuffer$AttributeType;
    const normal = geometry.getAttribute("normal");
    const color = geometry.getAttribute("color");
    const indexed = geometry.getIndex();
    const IndexArray = position.count <= 65536 ? Uint16Array : Uint32Array;
    const indices = indexed
      ? IndexArray.from(indexed.array)
      : IndexArray.from({ length: position.count }, (_, index) => index);
    const indexType = IndexArray === Uint16Array ? "USHORT" : "UINT";
    const vertexBuilder = f.VertexBuffer.Builder().vertexCount(position.count)
      .bufferCount(normal ? 3 : 2)
      .attribute(attributes.POSITION, 0, types.FLOAT3, 0, 12)
      .attribute(attributes.COLOR, 1, color ? types.FLOAT3 : types.UBYTE4, 0, color ? 12 : 4);
    if (!color) vertexBuilder.normalized(attributes.COLOR);
    if (normal) vertexBuilder.attribute(attributes.TANGENTS, 2, types.SHORT4, 0, 8).normalized(attributes.TANGENTS);
    const vertices = vertexBuilder.build(this.engine);
    vertices.setBufferAt(this.engine, 0, position.array);
    vertices.setBufferAt(this.engine, 1, color?.array ?? new Uint8Array(position.count * 4).fill(255));
    if (normal) {
      const builder = new f.SurfaceOrientation$Builder();
      builder.vertexCount(position.count);
      builder.normals(normal.array);
      const orientation = builder.build();
      vertices.setBufferAt(this.engine, 2, orientation.getQuats(position.count));
      orientation.delete();
    }

    const indexBuffer = f.IndexBuffer.Builder()
      .indexCount(indices.length).bufferType(f.IndexBuffer$IndexType[indexType]).build(this.engine);
    indexBuffer.setBuffer(this.engine, indices);

    const entry = { vertices, indexBuffer, positionVersion: position.version };
    this.geometry.set(geometry, entry);
    return entry;
  }

  createObject(object, geometry, material) {
    const f = this.filament;
    const entity = f.EntityManager.get().create();
    const bounds = object.geometry.boundingBox;
    const center = bounds.min.toArray().map((min, i) => (min + bounds.max.getComponent(i)) / 2);
    const halfExtent = bounds.max.toArray().map((max, i) => Math.max(0.0001, max - center[i]));
    const primitives = f.RenderableManager$PrimitiveType;
    const primitive = object.isLineSegments ? primitives.LINES
      : object.isLine ? primitives.LINE_STRIP : primitives.TRIANGLES;
    const depthTest = object.material.depthTest;
    const instance = material.createInstance();
    instance.setDepthCulling(depthTest);
    if (object.material.side === 2) instance.setDoubleSided(true);

    f.RenderableManager.Builder(1)
      .geometry(0, primitive, geometry.vertices, geometry.indexBuffer)
      .material(0, instance)
      .boundingBox({ center, halfExtent })
      .culling(depthTest)
      .priority(depthTest ? 4 : Math.min(7, 5 + Math.floor(object.renderOrder / 4)))
      .castShadows(object.castShadow)
      .receiveShadows(object.receiveShadow)
      .build(this.engine, entity);

    const entry = {
      entity,
      transform: this.transforms.getInstance(entity),
      renderable: this.renderables.getInstance(entity),
      matrix: object.matrixWorld.clone(),
      material,
      instance,
      instances: new Map([[material, instance]]),
      castShadow: object.castShadow,
    };
    this.transforms.setTransform(entry.transform, entry.matrix.elements);
    this.objects.set(object, entry);
    return entry;
  }

  syncObject(object) {
    const source = object.material;
    const lit = source.isMeshStandardMaterial;
    const material = lit ? (source.transparent ? this.fade : this.surface) : this.overlay;
    if (!object.geometry.boundingBox) object.geometry.computeBoundingBox();
    const geometry = this.uploadGeometry(object.geometry);
    const entry = this.objects.get(object) ?? this.createObject(object, geometry, material);

    if (entry.material !== material) {
      if (!entry.instances.has(material)) entry.instances.set(material, material.createInstance());
      entry.material = material;
      entry.instance = entry.instances.get(material);
      this.renderables.setMaterialInstanceAt(entry.renderable, 0, entry.instance);
      entry.parameters = undefined;
    }
    if (entry.castShadow !== object.castShadow) {
      entry.castShadow = object.castShadow;
      this.renderables.setCastShadows(entry.renderable, object.castShadow);
    }
    if (!entry.matrix.equals(object.matrixWorld)) {
      entry.matrix.copy(object.matrixWorld);
      this.transforms.setTransform(entry.transform, entry.matrix.elements);
    }

    const color = source.color;
    const parameters = [color?.r ?? 1, color?.g ?? 1, color?.b ?? 1, source.opacity];
    if (lit) {
      const glow = source.emissive;
      const intensity = source.emissiveIntensity;
      parameters.push(source.roughness, source.metalness, glow.r * intensity, glow.g * intensity, glow.b * intensity);
    }
    if (parameters.some((value, index) => value !== entry.parameters?.[index])) {
      entry.parameters = parameters;
      entry.instance.setFloat4Parameter("tint", parameters.slice(0, 4));
      if (lit) {
        entry.instance.setFloatParameter("roughness", parameters[4]);
        entry.instance.setFloatParameter("metallic", parameters[5]);
        entry.instance.setFloat4Parameter("glow", [...parameters.slice(6), 0]);
      }
    }
    return entry;
  }

  render(scene, camera) {
    scene.updateMatrixWorld();
    camera.updateMatrixWorld();
    this.camera.setProjectionFov(
      camera.fov, camera.aspect, camera.near, camera.far, this.filament.Camera$Fov.VERTICAL,
    );
    this.camera.setModelMatrix(camera.matrixWorld.elements);
    const visible = new Set();
    scene.traverseVisible((object) => {
      if (!object.isMesh && !object.isLine) return;
      const { entity } = this.syncObject(object);
      visible.add(entity);
      if (!this.visible.has(entity)) this.scene.addEntity(entity);
    });
    for (const entity of this.visible) {
      if (!visible.has(entity)) this.scene.remove(entity);
    }
    this.visible = visible;

    const drawn = this.renderer.beginFrame(this.swapChain);
    if (drawn) {
      this.renderer.renderView(this.view);
      this.renderer.endFrame();
      this.frameCount++;
    }
    this.engine.execute();
    return drawn;
  }
}
