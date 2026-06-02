import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const JOINT_LABELS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"];
const FORCE_HIDE_N = 0.05;
const FORCE_FULL_SCALE_N = 18.0;

const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));
const fmt = (value, digits = 1) => Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "--";

function vec3(values, fallback = 0) {
  const arr = Array.isArray(values) ? values : [];
  return new THREE.Vector3(
    Number.isFinite(Number(arr[0])) ? Number(arr[0]) : fallback,
    Number.isFinite(Number(arr[1])) ? Number(arr[1]) : fallback,
    Number.isFinite(Number(arr[2])) ? Number(arr[2]) : fallback,
  );
}

function applyOrigin(object, origin) {
  const xyz = vec3(origin?.xyz || [0, 0, 0]);
  const rpy = origin?.rpy || [0, 0, 0];
  object.position.copy(xyz);
  object.rotation.set(Number(rpy[0]) || 0, Number(rpy[1]) || 0, Number(rpy[2]) || 0, "XYZ");
}

function rgbaToColor(material) {
  const rgba = material?.rgba || [0.78, 0.78, 0.78, 1];
  return new THREE.Color(clamp(Number(rgba[0]) || 0, 0, 1), clamp(Number(rgba[1]) || 0, 0, 1), clamp(Number(rgba[2]) || 0, 0, 1));
}

function makeMaterial(material) {
  const rgba = material?.rgba || [0.78, 0.78, 0.78, 1];
  const color = rgbaToColor(material);
  const alpha = clamp(Number(rgba[3]) || 1, 0, 1);
  return new THREE.MeshStandardMaterial({
    color,
    roughness: 0.72,
    metalness: material?.name === "Silver" ? 0.18 : 0.04,
    transparent: alpha < 1,
    opacity: alpha,
  });
}

function makeGeometry(geometry) {
  if (!geometry) return null;
  if (geometry.type === "box") {
    const size = geometry.size || [0.04, 0.04, 0.04];
    return { geometry: new THREE.BoxGeometry(Number(size[0]) || 0.04, Number(size[1]) || 0.04, Number(size[2]) || 0.04), alignCylinder: false };
  }
  if (geometry.type === "cylinder") {
    const radius = Number(geometry.radius) || 0.02;
    const length = Number(geometry.length) || 0.04;
    return { geometry: new THREE.CylinderGeometry(radius, radius, length, 36, 1), alignCylinder: true };
  }
  if (geometry.type === "sphere") {
    return { geometry: new THREE.SphereGeometry(Number(geometry.radius) || 0.02, 32, 16), alignCylinder: false };
  }
  return { geometry: new THREE.BoxGeometry(0.045, 0.045, 0.045), alignCylinder: false };
}

function makePrimitiveMesh(visual) {
  const payload = makeGeometry(visual.geometry);
  if (!payload) return null;
  const mesh = new THREE.Mesh(payload.geometry, makeMaterial(visual.material));
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  if (payload.alignCylinder) {
    mesh.rotation.x = Math.PI / 2;
  }

  const edge = new THREE.LineSegments(
    new THREE.EdgesGeometry(payload.geometry),
    new THREE.LineBasicMaterial({ color: 0x181512, transparent: true, opacity: 0.62 }),
  );
  mesh.add(edge);
  return mesh;
}

function forceColor(magnitude) {
  const color = new THREE.Color();
  const t = clamp(magnitude / FORCE_FULL_SCALE_N, 0, 1);
  color.setHSL(0.34 * (1 - t), 0.92, 0.56);
  return color;
}

export class RobotView {
  constructor(canvas, options = {}) {
    this.canvas = canvas;
    this.forceHud = options.forceHud || null;
    this.model = null;
    this.linkObjects = new Map();
    this.jointObjects = new Map();
    this.eefObject = null;
    this.latestSnapshot = null;
    this.forceVector = new THREE.Vector3();
    this.eefPosition = new THREE.Vector3();
    this.tmpDirection = new THREE.Vector3(1, 0, 0);

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false, preserveDrawingBuffer: true });
    this.renderer.setClearColor(0x050403, 1);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x050403);
    this.camera = new THREE.PerspectiveCamera(42, 1, 0.01, 12);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(0.78, -1.18, 0.72);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.12, 0.0, 0.28);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.minDistance = 0.35;
    this.controls.maxDistance = 3.2;
    this.controls.update();

    this.robotRoot = new THREE.Group();
    this.robotRoot.name = "xarm6-live-root";
    this.scene.add(this.robotRoot);

    this.forceArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.001, 0x79d88f, 0.001, 0.001);
    this.forceArrow.name = "estimated-end-effector-force";
    this.forceArrow.visible = false;
    this.scene.add(this.forceArrow);

    this._addEnvironment();
    this.resize();
    this.renderer.setAnimationLoop(() => this.renderFrame());
  }

  _addEnvironment() {
    const hemi = new THREE.HemisphereLight(0xf5ead8, 0x19120a, 2.1);
    this.scene.add(hemi);

    const key = new THREE.DirectionalLight(0xffe0a3, 2.6);
    key.position.set(-0.75, -0.8, 1.35);
    this.scene.add(key);

    const fill = new THREE.DirectionalLight(0x76a6ff, 0.82);
    fill.position.set(0.8, 0.65, 0.6);
    this.scene.add(fill);

    const grid = new THREE.GridHelper(1.35, 27, 0xd7a735, 0x3b2a17);
    grid.name = "mujoco-style-floor-grid";
    grid.rotation.x = Math.PI / 2;
    grid.material.transparent = true;
    grid.material.opacity = 0.38;
    this.scene.add(grid);

    const axes = new THREE.AxesHelper(0.16);
    axes.position.set(-0.54, -0.48, 0.02);
    this.scene.add(axes);
  }

  setRobotModel(model) {
    this.model = model;
    this._clearRobot();
    if (!model?.ok || !model.links || !Array.isArray(model.joints)) {
      return;
    }

    Object.values(model.links).forEach((link) => {
      const group = new THREE.Group();
      group.name = link.name;
      (link.visuals || []).forEach((visual) => {
        const pose = new THREE.Group();
        pose.name = `${visual.name || link.name}_pose`;
        applyOrigin(pose, visual.origin);
        const mesh = makePrimitiveMesh(visual);
        if (mesh) {
          pose.add(mesh);
          group.add(pose);
        }
      });
      this.linkObjects.set(link.name, group);
    });

    const childNames = new Set(model.joints.map((joint) => joint.child).filter(Boolean));
    Object.keys(model.links).forEach((name) => {
      if (!childNames.has(name)) {
        this.robotRoot.add(this.linkObjects.get(name));
      }
    });

    model.joints.forEach((joint) => {
      const parent = this.linkObjects.get(joint.parent);
      const child = this.linkObjects.get(joint.child);
      if (!parent || !child) return;

      const origin = new THREE.Group();
      origin.name = `${joint.name}_origin`;
      applyOrigin(origin, joint.origin);
      parent.add(origin);

      const motion = new THREE.Group();
      motion.name = joint.name;
      origin.add(motion);
      motion.add(child);

      if (joint.type !== "fixed") {
        const axis = vec3(joint.axis || [0, 0, 1]);
        if (axis.lengthSq() < 1e-8) axis.set(0, 0, 1);
        axis.normalize();
        this.jointObjects.set(joint.name, { motion, axis, type: joint.type });
      }
    });

    this.eefObject = this.linkObjects.get("link_eef") || this.linkObjects.get("link6") || null;
    this.setJointAngles(this.latestSnapshot?.state?.joints_deg || []);
    this.updateForce(this.latestSnapshot?.state?.ee_force || []);
  }

  _clearRobot() {
    while (this.robotRoot.children.length) {
      this.robotRoot.remove(this.robotRoot.children[0]);
    }
    this.linkObjects.clear();
    this.jointObjects.clear();
    this.eefObject = null;
  }

  setJointAngles(jointsDeg) {
    const jointNames = Array.isArray(this.model?.movable_joints) && this.model.movable_joints.length >= 6
      ? this.model.movable_joints.slice(0, 6)
      : JOINT_LABELS;
    jointNames.forEach((name, index) => {
      const joint = this.jointObjects.get(name);
      if (!joint) return;
      const deg = Number(jointsDeg?.[index]) || 0;
      joint.motion.quaternion.setFromAxisAngle(joint.axis, THREE.MathUtils.degToRad(deg));
    });
    this.robotRoot.updateMatrixWorld(true);
  }

  update(snapshot) {
    this.latestSnapshot = snapshot;
    this.setJointAngles(snapshot?.state?.joints_deg || []);
    this.updateForce(snapshot?.state?.ee_force || []);
  }

  updateForce(forceValues) {
    const force = vec3(forceValues || [0, 0, 0]);
    const magnitude = force.length();
    this.forceVector.copy(force);

    if (this.forceHud) {
      this.forceHud.innerHTML = `
        <span>|F| <strong>${fmt(magnitude, 1)} N</strong></span>
        <span>Fx ${fmt(force.x, 1)}</span>
        <span>Fy ${fmt(force.y, 1)}</span>
        <span>Fz ${fmt(force.z, 1)}</span>
      `;
    }

    if (!this.eefObject || magnitude < FORCE_HIDE_N) {
      this.forceArrow.visible = false;
      return;
    }

    this.eefObject.getWorldPosition(this.eefPosition);
    this.tmpDirection.copy(force).normalize();
    const length = clamp(magnitude * 0.018, 0.035, 0.42);
    const headLength = clamp(length * 0.24, 0.018, 0.08);
    const headWidth = clamp(headLength * 0.55, 0.012, 0.05);
    const color = forceColor(magnitude);

    this.forceArrow.visible = true;
    this.forceArrow.position.copy(this.eefPosition);
    this.forceArrow.setDirection(this.tmpDirection);
    this.forceArrow.setLength(length, headLength, headWidth);
    this.forceArrow.setColor(color);
  }

  resize() {
    const rect = this.canvas.parentElement?.getBoundingClientRect() || this.canvas.getBoundingClientRect();
    const width = Math.max(320, Math.floor(rect.width || 320));
    const height = Math.max(220, Math.floor(rect.height || 220));
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  renderFrame() {
    this.resize();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}

export function createRobotView(canvas, options = {}) {
  return new RobotView(canvas, options);
}
