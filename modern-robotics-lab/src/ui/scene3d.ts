/**
 * Three.js scene wrapper with a z-up robotics convention, orbit camera, themed background,
 * and helpers for frames, links, arrows and screw axes. Matrices follow the lab convention:
 * number[][] row-major 4×4 homogeneous transforms.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { el, cssVar } from './dom';

export type Vec3Like = ArrayLike<number> | THREE.Vector3;

export interface Scene3DOptions {
  height?: number;
  aspect?: number;
  /** Grid size in world units (default 4); false disables. */
  grid?: number | false;
  /** World axes triad size (default 0.5); false disables. */
  axes?: number | false;
  cameraPos?: [number, number, number];
  target?: [number, number, number];
  /** Start the render loop immediately (default true). */
  autoStart?: boolean;
  fov?: number;
}

export function vec3(a: Vec3Like): THREE.Vector3 {
  if (a instanceof THREE.Vector3) return a.clone();
  return new THREE.Vector3(a[0], a[1], a[2]);
}

/** Row-major 4×4 number[][] → THREE.Matrix4. */
export function mat4FromT(T: number[][]): THREE.Matrix4 {
  const m = new THREE.Matrix4();
  m.set(
    T[0][0], T[0][1], T[0][2], T[0][3],
    T[1][0], T[1][1], T[1][2], T[1][3],
    T[2][0], T[2][1], T[2][2], T[2][3],
    T[3]?.[0] ?? 0, T[3]?.[1] ?? 0, T[3]?.[2] ?? 0, T[3]?.[3] ?? 1,
  );
  return m;
}

/** Apply a 4×4 transform (or 3×3 rotation) to an object. */
export function setMatrix(obj: THREE.Object3D, T: number[][]): void {
  obj.matrixAutoUpdate = false;
  if (T.length === 3) {
    obj.matrix.set(T[0][0], T[0][1], T[0][2], 0, T[1][0], T[1][1], T[1][2], 0, T[2][0], T[2][1], T[2][2], 0, 0, 0, 0, 1);
  } else {
    obj.matrix.copy(mat4FromT(T));
  }
  obj.matrixWorldNeedsUpdate = true;
}

export function axisColors(): { x: string; y: string; z: string } {
  return { x: cssVar('--axis-x', '#e11d48'), y: cssVar('--axis-y', '#16a34a'), z: cssVar('--axis-z', '#2563eb') };
}

/**
 * Theme binding: a colour given as a CSS custom-property name (a string starting with "--",
 * e.g. '--axis-x', '--fg', '--accent') is resolved now and re-resolved by Scene3D whenever the
 * theme changes. Any other THREE colour (hex number, '#rrggbb', a cssVar(...) result) stays fixed.
 */
export function resolveColor(c: THREE.ColorRepresentation, fallback = '#888888'): THREE.ColorRepresentation {
  return typeof c === 'string' && c.startsWith('--') ? cssVar(c, fallback) : c;
}

type ColoredMaterial = THREE.Material & { color: THREE.Color };

/** Set a material colour and remember its CSS variable (if any) for theme refreshes. */
export function setThemedColor(mat: ColoredMaterial, c: THREE.ColorRepresentation): void {
  if (typeof c === 'string' && c.startsWith('--')) mat.userData.themeVar = c;
  else delete mat.userData.themeVar;
  mat.color.set(resolveColor(c));
}

/** MeshStandardMaterial whose colour may be a CSS variable name (see resolveColor). */
export function themedStandardMaterial(c: THREE.ColorRepresentation, params: THREE.MeshStandardMaterialParameters = {}): THREE.MeshStandardMaterial {
  const m = new THREE.MeshStandardMaterial(params);
  setThemedColor(m, c);
  return m;
}

/** Re-resolve every theme-bound material colour and text sprite under `root` (Scene3D calls this on theme change). */
export function refreshThemeColors(root: THREE.Object3D): void {
  root.traverse((o) => {
    const mat = (o as THREE.Mesh).material as THREE.Material | THREE.Material[] | undefined;
    const mats = Array.isArray(mat) ? mat : mat ? [mat] : [];
    for (const m of mats) {
      const v = m.userData?.themeVar as string | undefined;
      if (v && (m as ColoredMaterial).color) (m as ColoredMaterial).color.set(cssVar(v, '#888888'));
    }
    if ((o as THREE.Sprite).isSprite && o.userData.themeText) redrawTextSprite(o as THREE.Sprite);
  });
}

/** An arrow mesh (shaft + head) pointing along local +Y with unit length; use setLength/setFromTo. */
export class Arrow3D {
  readonly group = new THREE.Group();
  private shaft: THREE.Mesh;
  private head: THREE.Mesh;
  private headLen: number;
  private radius: number;
  constructor(color: THREE.ColorRepresentation, radius = 0.012, headLen = 0.08, opacity = 1) {
    this.radius = radius; this.headLen = headLen;
    const mat = themedStandardMaterial(color, { transparent: opacity < 1, opacity, roughness: 0.6 });
    this.shaft = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 12), mat);
    this.head = new THREE.Mesh(new THREE.ConeGeometry(radius * 2.6, headLen, 16), mat);
    this.group.add(this.shaft, this.head);
    this.setLength(1);
  }
  setColor(c: THREE.ColorRepresentation): void { setThemedColor(this.shaft.material as THREE.MeshStandardMaterial, c); }
  setLength(L: number): void {
    const hl = Math.min(this.headLen, L * 0.5);
    const sl = Math.max(1e-4, L - hl);
    this.shaft.scale.set(1, sl, 1);
    this.shaft.position.set(0, sl / 2, 0);
    this.head.scale.set(1, hl / this.headLen, 1);
    this.head.position.set(0, sl + hl / 2, 0);
  }
  /** Place the arrow from a to b (world/parent coordinates). */
  setFromTo(a: Vec3Like, b: Vec3Like): void {
    const from = vec3(a), to = vec3(b);
    const d = to.clone().sub(from); const L = d.length();
    this.group.position.copy(from);
    this.group.visible = L > 1e-6;
    if (L > 1e-6) {
      this.group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
      this.setLength(L);
    }
  }
  /** Place the arrow at `origin` pointing along `dir` (unit) with length L. */
  setDir(origin: Vec3Like, dir: Vec3Like, L: number): void {
    const o = vec3(origin), d = vec3(dir);
    this.setFromTo(o, o.clone().add(d.multiplyScalar(L)));
  }
  set visible(v: boolean) { this.group.visible = v; }
  get radiusPx(): number { return this.radius; }
}

/** A coordinate frame: three arrows (x red, y green, z blue) plus an optional label sprite. */
export class Frame3D {
  readonly group = new THREE.Group();
  readonly axes: { x: Arrow3D; y: Arrow3D; z: Arrow3D };
  private label?: THREE.Sprite;
  constructor(opts: { size?: number; label?: string; opacity?: number; thickness?: number } = {}) {
    const size = opts.size ?? 0.3;
    const c = { x: '--axis-x', y: '--axis-y', z: '--axis-z' };
    const r = opts.thickness ?? size * 0.04;
    const head = size * 0.22;
    const op = opts.opacity ?? 1;
    this.axes = { x: new Arrow3D(c.x, r, head, op), y: new Arrow3D(c.y, r, head, op), z: new Arrow3D(c.z, r, head, op) };
    this.axes.x.group.rotation.z = -Math.PI / 2; // +Y → +X
    this.axes.z.group.rotation.x = Math.PI / 2;  // +Y → +Z
    for (const a of Object.values(this.axes)) { a.setLength(size); this.group.add(a.group); }
    if (opts.label) {
      this.label = makeTextSprite(opts.label, '--fg');
      this.label.position.set(-size * 0.25, -size * 0.25, size * 0.25);
      this.group.add(this.label);
    }
  }
  setTransform(T: number[][]): void { setMatrix(this.group, T); }
  setVisible(v: boolean): void { this.group.visible = v; }
  setLabel(text: string): void {
    if (this.label) this.group.remove(this.label);
    this.label = makeTextSprite(text, '--fg');
    this.label.position.set(-0.08, -0.08, 0.08);
    this.group.add(this.label);
  }
  setOpacity(op: number): void {
    this.group.traverse((o) => {
      if ((o as THREE.Mesh).isMesh) {
        const m = (o as THREE.Mesh).material as THREE.MeshStandardMaterial;
        m.transparent = op < 1; m.opacity = op;
      }
    });
  }
}

/** Cylinder between two points (robot link). */
export class Link3D {
  readonly mesh: THREE.Mesh;
  constructor(radius = 0.03, color: THREE.ColorRepresentation = 0x888888, opacity = 1) {
    this.mesh = new THREE.Mesh(
      new THREE.CylinderGeometry(radius, radius, 1, 16),
      themedStandardMaterial(color, { roughness: 0.5, metalness: 0.1, transparent: opacity < 1, opacity }),
    );
  }
  setEndpoints(a: Vec3Like, b: Vec3Like): void {
    const from = vec3(a), to = vec3(b);
    const d = to.clone().sub(from); const L = d.length();
    this.mesh.visible = L > 1e-6;
    if (L <= 1e-6) return;
    this.mesh.position.copy(from.clone().add(to).multiplyScalar(0.5));
    this.mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
    this.mesh.scale.set(1, L, 1);
  }
  setColor(c: THREE.ColorRepresentation): void { setThemedColor(this.mesh.material as THREE.MeshStandardMaterial, c); }
}

/** Visualization of a screw axis S = (ω, v): a line through a point on the axis, direction arrow, pitch marker. */
export class ScrewAxis3D {
  readonly group = new THREE.Group();
  private line: THREE.Line;
  private arrow: Arrow3D;
  private dot: THREE.Mesh;
  private ring: THREE.Mesh;
  private length: number;
  constructor(opts: { color?: THREE.ColorRepresentation; length?: number } = {}) {
    const color = opts.color ?? 0xd97706;
    this.length = opts.length ?? 1.5;
    const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 0, 1)]);
    const lineMat = new THREE.LineDashedMaterial({ dashSize: 0.06, gapSize: 0.04 });
    setThemedColor(lineMat, color);
    this.line = new THREE.Line(geo, lineMat);
    this.arrow = new Arrow3D(color, 0.014, 0.1);
    this.dot = new THREE.Mesh(new THREE.SphereGeometry(0.03, 12, 12), themedStandardMaterial(color));
    this.ring = new THREE.Mesh(new THREE.TorusGeometry(0.12, 0.008, 8, 40), themedStandardMaterial(color));
    this.group.add(this.line, this.arrow.group, this.dot, this.ring);
  }
  /** Update from S = [ω; v] (6-vector). Pure translation (ω = 0) is drawn as an arrow along v. */
  setScrew(S: ArrayLike<number>): void {
    const w = new THREE.Vector3(S[0], S[1], S[2]);
    const v = new THREE.Vector3(S[3], S[4], S[5]);
    const wn = w.length();
    if (wn > 1e-9) {
      const wh = w.clone().divideScalar(wn);
      const q = wh.clone().cross(v).divideScalar(wn); // point on axis: ω̂ × v / |ω| (for |ω|=1, q = ω × v)
      const a = q.clone().sub(wh.clone().multiplyScalar(this.length));
      const b = q.clone().add(wh.clone().multiplyScalar(this.length));
      this.line.geometry.setFromPoints([a, b]);
      this.line.computeLineDistances();
      this.line.visible = true;
      this.arrow.setDir(q, wh, 0.45);
      this.dot.position.copy(q); this.dot.visible = true;
      this.ring.position.copy(q); this.ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), wh); this.ring.visible = true;
    } else {
      const vn = v.length();
      this.line.visible = false; this.dot.visible = false; this.ring.visible = false;
      if (vn > 1e-9) this.arrow.setDir([0, 0, 0], v.clone().divideScalar(vn), 0.6); else this.arrow.visible = false;
    }
  }
  setVisible(v: boolean): void { this.group.visible = v; }
}

function drawTextCanvas(text: string, color: string, fontPx: number): HTMLCanvasElement {
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d')!;
  const font = `${fontPx}px ${cssVar('--font') || 'sans-serif'}`;
  ctx.font = font;
  const w = Math.ceil(ctx.measureText(text).width) + 16;
  canvas.width = Math.max(2, w); canvas.height = fontPx + 16;
  ctx.font = font;
  ctx.textBaseline = 'middle';
  ctx.fillStyle = color;
  ctx.fillText(text, 8, canvas.height / 2);
  return canvas;
}

/**
 * Text label rendered to a sprite (always faces the camera). `color` defaults to the theme
 * foreground; pass a CSS variable name ('--accent') to keep it theme-aware, or a fixed colour.
 */
export function makeTextSprite(text: string, color: string = '--fg', fontPx = 48): THREE.Sprite {
  const canvas = drawTextCanvas(text, String(resolveColor(color, '#222222')), fontPx);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const mat = new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true });
  const sprite = new THREE.Sprite(mat);
  const scale = 0.0028;
  sprite.scale.set(canvas.width * scale, canvas.height * scale, 1);
  sprite.renderOrder = 999;
  if (color.startsWith('--')) sprite.userData.themeText = { text, color, fontPx };
  return sprite;
}

/** Redraw a theme-bound text sprite with the current theme colour (keeps its scale). */
function redrawTextSprite(sprite: THREE.Sprite): void {
  const { text, color, fontPx } = sprite.userData.themeText as { text: string; color: string; fontPx: number };
  const mat = sprite.material as THREE.SpriteMaterial;
  const old = mat.map;
  const tex = new THREE.CanvasTexture(drawTextCanvas(text, cssVar(color, '#222222'), fontPx));
  tex.colorSpace = THREE.SRGBColorSpace;
  mat.map = tex;
  mat.needsUpdate = true;
  old?.dispose();
}

export class Scene3D {
  readonly el: HTMLDivElement;
  readonly canvas: HTMLCanvasElement;
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  renderer: THREE.WebGLRenderer | null = null;
  controls: OrbitControls | null = null;
  readonly ok: boolean;
  private grid?: THREE.GridHelper;
  private ro: ResizeObserver;
  private opts: Scene3DOptions;
  private tickers = new Set<(dt: number, t: number) => void>();
  private raf = 0;
  private running = false;
  private last = 0;
  private t = 0;
  private onTheme = () => { this.applyTheme(); this.requestRender(); };

  constructor(parent: HTMLElement, opts: Scene3DOptions = {}) {
    this.opts = opts;
    this.canvas = el('canvas');
    this.el = el('div', { class: 'mr-canvas-wrap mr-scene3d' }, this.canvas);
    parent.appendChild(this.el);
    this.camera = new THREE.PerspectiveCamera(opts.fov ?? 42, 4 / 3, 0.01, 200);
    this.camera.up.set(0, 0, 1);
    const cp = opts.cameraPos ?? [2.6, -2.6, 1.8];
    this.camera.position.set(cp[0], cp[1], cp[2]);
    try {
      this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, alpha: false, powerPreference: 'low-power' });
      this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
      this.ok = true;
    } catch (e) {
      this.ok = false;
      this.el.appendChild(el('div', { class: 'mr-note', style: { padding: '1rem' } }, '此瀏覽器無法建立 WebGL 場景（3D 顯示停用）。'));
    }
    if (this.renderer) {
      this.controls = new OrbitControls(this.camera, this.canvas);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.12;
      const tg = opts.target ?? [0, 0, 0.3];
      this.controls.target.set(tg[0], tg[1], tg[2]);
      this.controls.addEventListener('change', () => this.requestRender());
    }
    // Lights
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x556677, 1.1));
    const dir = new THREE.DirectionalLight(0xffffff, 1.4);
    dir.position.set(3, -2, 5);
    this.scene.add(dir);
    const dir2 = new THREE.DirectionalLight(0xffffff, 0.5);
    dir2.position.set(-3, 3, 2);
    this.scene.add(dir2);
    // Grid on the XY plane (GridHelper is XZ by default)
    if (opts.grid !== false) {
      const size = opts.grid ?? 4;
      this.grid = new THREE.GridHelper(size, size * 2);
      this.grid.rotation.x = Math.PI / 2;
      this.scene.add(this.grid);
    }
    if (opts.axes !== false) {
      const f = new Frame3D({ size: opts.axes ?? 0.5, thickness: 0.006, label: '{s}' });
      this.scene.add(f.group);
    }
    this.applyTheme();
    window.addEventListener('mr-theme', this.onTheme);
    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(this.el);
    this.resize();
    if (opts.autoStart !== false) this.start();
  }

  private applyTheme(): void {
    this.renderer?.setClearColor(new THREE.Color(cssVar('--sim-bg', '#ffffff')));
    refreshThemeColors(this.scene);
    if (this.grid) {
      const mat = this.grid.material as THREE.LineBasicMaterial;
      mat.color.set(cssVar('--grid-strong', '#cccccc'));
      mat.transparent = true; mat.opacity = 0.8;
    }
  }

  private resize(): void {
    const W = this.el.clientWidth;
    if (W <= 0) return;
    const H = this.opts.height ?? Math.round(W / (this.opts.aspect ?? 4 / 3));
    this.el.style.height = `${H}px`;
    this.camera.aspect = W / H;
    this.camera.updateProjectionMatrix();
    this.renderer?.setSize(W, H, false);
    this.requestRender();
  }

  /** Register a per-frame callback (dt in seconds, clamped). Returns an unsubscribe function. */
  onTick(cb: (dt: number, t: number) => void): () => void {
    this.tickers.add(cb);
    return () => { this.tickers.delete(cb); };
  }

  private renderQueued = false;

  /** Render once on the next animation frame (coalesced). No-op while the loop is running. */
  requestRender(): void {
    if (this.running || this.renderQueued || !this.renderer) return;
    this.renderQueued = true;
    requestAnimationFrame(() => {
      this.renderQueued = false;
      if (this.running || !this.renderer) return;
      // controls.update() may dispatch 'change' (damping); that only queues another frame.
      this.controls?.update();
      this.renderer.render(this.scene, this.camera);
    });
  }

  start(): void {
    if (this.running || !this.renderer) return;
    this.running = true;
    this.last = performance.now();
    const loop = (now: number) => {
      if (!this.running) return;
      const dt = Math.min(0.05, (now - this.last) / 1000);
      this.last = now; this.t += dt;
      for (const cb of this.tickers) cb(dt, this.t);
      this.controls?.update();
      this.renderer!.render(this.scene, this.camera);
      this.raf = requestAnimationFrame(loop);
    };
    this.raf = requestAnimationFrame(loop);
  }

  stop(): void {
    this.running = false;
    if (this.raf) cancelAnimationFrame(this.raf);
    this.raf = 0;
  }

  /** Point the camera at a target from a given distance (keeps the current direction). */
  lookAt(target: Vec3Like, distance?: number): void {
    if (!this.controls) return;
    const tg = vec3(target);
    if (distance != null) {
      const dir = this.camera.position.clone().sub(this.controls.target).normalize();
      this.camera.position.copy(tg.clone().add(dir.multiplyScalar(distance)));
    }
    this.controls.target.copy(tg);
    this.controls.update();
    this.requestRender();
  }

  // ---- factories ------------------------------------------------------
  addFrame(opts: { size?: number; label?: string; opacity?: number; T?: number[][] } = {}): Frame3D {
    const f = new Frame3D(opts);
    if (opts.T) f.setTransform(opts.T);
    this.scene.add(f.group);
    return f;
  }
  addLink(opts: { radius?: number; color?: THREE.ColorRepresentation; opacity?: number } = {}): Link3D {
    const l = new Link3D(opts.radius, opts.color ?? '--fg-muted', opts.opacity);
    this.scene.add(l.mesh);
    return l;
  }
  addArrow(opts: { color?: THREE.ColorRepresentation; radius?: number; head?: number } = {}): Arrow3D {
    const a = new Arrow3D(opts.color ?? '--accent', opts.radius, opts.head);
    this.scene.add(a.group);
    return a;
  }
  addSphere(radius = 0.05, color: THREE.ColorRepresentation = 0x2563eb, opacity = 1): THREE.Mesh {
    const m = new THREE.Mesh(new THREE.SphereGeometry(radius, 20, 16), themedStandardMaterial(color, { transparent: opacity < 1, opacity, roughness: 0.5 }));
    this.scene.add(m);
    return m;
  }
  addBox(size: [number, number, number], color: THREE.ColorRepresentation = 0x94a3b8, opacity = 1): THREE.Mesh {
    const m = new THREE.Mesh(new THREE.BoxGeometry(size[0], size[1], size[2]), themedStandardMaterial(color, { transparent: opacity < 1, opacity, roughness: 0.6 }));
    this.scene.add(m);
    return m;
  }
  addScrewAxis(opts: { color?: THREE.ColorRepresentation; length?: number } = {}): ScrewAxis3D {
    const s = new ScrewAxis3D(opts);
    this.scene.add(s.group);
    return s;
  }
  /** Polyline (trail / path). Call `update(points)` to change it. */
  addPolyline(points: Vec3Like[], color: THREE.ColorRepresentation = 0xd97706, opts: { dashed?: boolean } = {}): { line: THREE.Line; update(pts: Vec3Like[]): void } {
    const geo = new THREE.BufferGeometry().setFromPoints(points.map(vec3));
    const mat = opts.dashed ? new THREE.LineDashedMaterial({ dashSize: 0.05, gapSize: 0.03 }) : new THREE.LineBasicMaterial();
    setThemedColor(mat, color);
    const line = new THREE.Line(geo, mat);
    if (opts.dashed) line.computeLineDistances();
    this.scene.add(line);
    return {
      line,
      update: (pts) => { line.geometry.setFromPoints(pts.map(vec3)); if (opts.dashed) line.computeLineDistances(); this.requestRender(); },
    };
  }
  addLabel(text: string, pos: Vec3Like, color?: string): THREE.Sprite {
    const s = makeTextSprite(text, color ?? '--fg');
    s.position.copy(vec3(pos));
    this.scene.add(s);
    return s;
  }

  destroy(): void {
    this.stop();
    this.ro.disconnect();
    window.removeEventListener('mr-theme', this.onTheme);
    this.controls?.dispose();
    this.scene.traverse((o) => {
      const m = o as THREE.Mesh;
      if (m.geometry) m.geometry.dispose();
      const mat = (m as THREE.Mesh).material as THREE.Material | THREE.Material[] | undefined;
      if (Array.isArray(mat)) mat.forEach((x) => x.dispose()); else mat?.dispose();
    });
    this.renderer?.dispose();
    this.renderer?.forceContextLoss();
    this.el.remove();
  }
}
