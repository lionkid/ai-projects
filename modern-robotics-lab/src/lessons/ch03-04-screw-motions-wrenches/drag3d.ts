/**
 * Direct manipulation of constrained handles inside a Scene3D (the toolkit only offers 2D handles).
 * Each handle reports its world position (null = hidden) and receives the pointer ray while it is
 * dragged; the handle decides the constraint itself (a horizontal plane, a line, ...).
 * Listeners sit on the scene wrapper in the capture phase, so a drag that we claim never reaches the
 * OrbitControls on the canvas; every other drag still orbits the camera.
 */
import * as THREE from 'three';
import type { Scene3D } from '@/ui';

export interface DragHandle3D {
  /** Current world position, or null when the handle is hidden / disabled. */
  pos(): THREE.Vector3 | null;
  /** Called with the pointer ray (world space) while the handle is dragged. */
  drag(ray: THREE.Ray): void;
  onStart?(): void;
  onEnd?(): void;
}

export interface DragController { destroy(): void; readonly active: DragHandle3D | null }

export function attachDragHandles(scene: Scene3D, handles: DragHandle3D[], opts: { hitPx?: number } = {}): DragController {
  const wrap = scene.el;
  const canvas = scene.canvas;
  const raycaster = new THREE.Raycaster();
  const hitPx = opts.hitPx ?? 20;
  let active: DragHandle3D | null = null;
  let pointerId = -1;

  const local = (ev: PointerEvent): [number, number, number, number] => {
    const r = canvas.getBoundingClientRect();
    return [ev.clientX - r.left, ev.clientY - r.top, Math.max(1, r.width), Math.max(1, r.height)];
  };

  const pick = (ev: PointerEvent): DragHandle3D | null => {
    const [x, y, w, h] = local(ev);
    let best: DragHandle3D | null = null;
    let bestD = hitPx;
    for (const hd of handles) {
      const p = hd.pos();
      if (!p) continue;
      const s = p.clone().project(scene.camera);
      if (s.z > 1) continue; // behind the camera
      const d = Math.hypot(((s.x + 1) / 2) * w - x, ((1 - s.y) / 2) * h - y);
      if (d <= bestD) { best = hd; bestD = d; }
    }
    return best;
  };

  const rayOf = (ev: PointerEvent): THREE.Ray => {
    const [x, y, w, h] = local(ev);
    raycaster.setFromCamera(new THREE.Vector2((x / w) * 2 - 1, -(y / h) * 2 + 1), scene.camera);
    return raycaster.ray.clone();
  };

  const setControls = (on: boolean) => { if (scene.controls) scene.controls.enabled = on; };

  const onDown = (ev: PointerEvent) => {
    if (ev.button !== 0 || active) return;
    const hd = pick(ev);
    if (!hd) return;
    ev.stopPropagation();
    ev.preventDefault();
    active = hd;
    pointerId = ev.pointerId;
    try { canvas.setPointerCapture(ev.pointerId); } catch { /* pointer already gone */ }
    setControls(false);
    wrap.style.cursor = 'grabbing';
    hd.onStart?.();
  };

  const onMove = (ev: PointerEvent) => {
    if (!active) {
      wrap.style.cursor = pick(ev) ? 'grab' : '';
      return;
    }
    if (ev.pointerId !== pointerId) return;
    ev.stopPropagation();
    active.drag(rayOf(ev));
  };

  const onUp = (ev: PointerEvent) => {
    if (!active || ev.pointerId !== pointerId) return;
    ev.stopPropagation();
    try { canvas.releasePointerCapture(ev.pointerId); } catch { /* already released */ }
    const hd = active;
    active = null;
    pointerId = -1;
    setControls(true);
    wrap.style.cursor = '';
    hd.onEnd?.();
  };

  const cap = { capture: true } as const;
  wrap.addEventListener('pointerdown', onDown, cap);
  wrap.addEventListener('pointermove', onMove, cap);
  wrap.addEventListener('pointerup', onUp, cap);
  wrap.addEventListener('pointercancel', onUp, cap);
  return {
    get active() { return active; },
    destroy() {
      wrap.removeEventListener('pointerdown', onDown, cap);
      wrap.removeEventListener('pointermove', onMove, cap);
      wrap.removeEventListener('pointerup', onUp, cap);
      wrap.removeEventListener('pointercancel', onUp, cap);
      setControls(true);
    },
  };
}

/** Intersection of a ray with the horizontal plane z = z0; null when (nearly) parallel or behind the eye. */
export function rayPlaneZ(ray: THREE.Ray, z0: number): THREE.Vector3 | null {
  const dz = ray.direction.z;
  if (Math.abs(dz) < 1e-6) return null;
  const t = (z0 - ray.origin.z) / dz;
  if (t <= 0) return null;
  return ray.origin.clone().addScaledVector(ray.direction, t);
}

/**
 * Parameter s of the point a + s·w (w a unit vector) on a line that is closest to the pointer ray;
 * null when the ray is (nearly) parallel to the line.
 */
export function rayLineParam(ray: THREE.Ray, a: THREE.Vector3, w: THREE.Vector3): number | null {
  const u = ray.direction;
  const r = ray.origin.clone().sub(a);
  const b = u.dot(w);
  const den = 1 - b * b;
  if (den < 1e-8) return null;
  // minimise |o + t u − a − s w|² over (t, s)
  return (w.dot(r) - b * u.dot(r)) / den;
}
