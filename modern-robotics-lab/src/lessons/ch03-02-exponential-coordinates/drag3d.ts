/**
 * Direct manipulation inside a Scene3D:
 *  - drag the axis handle over a sphere centred at the origin (sets a unit direction);
 *  - optionally, drag empty space to rotate the OBJECT with an arcball (instead of orbiting the camera).
 * Listeners sit on the scene wrapper in the capture phase, so a drag that we claim never reaches the
 * OrbitControls (which listen on the canvas); everything else falls through to the camera controls.
 */
import * as THREE from 'three';
import type { Scene3D } from '@/ui';

export type DragKind = 'axis' | 'object';

export interface Drag3DOpts {
  /** World position of the draggable handle, or null when it is hidden. */
  handlePos(): THREE.Vector3 | null;
  /** Radius of the sphere the handle lives on. */
  radius: number;
  /** New unit direction while the handle is dragged. */
  onAxis(dir: [number, number, number]): void;
  /** When true, dragging empty space rotates the object (arcball) instead of the camera. */
  objectMode(): boolean;
  /** Incremental rotation of the object, as a unit axis (world) and an angle. */
  onRotate(axis: [number, number, number], angle: number): void;
  onDragState?(kind: DragKind | null): void;
  /** Pick radius around the handle in CSS px (default 18). */
  hitPx?: number;
}

export function attachDrag3D(scene: Scene3D, o: Drag3DOpts): { destroy(): void; readonly dragging: DragKind | null } {
  const wrap = scene.el;
  const canvas = scene.canvas;
  const cam = scene.camera;
  const ray = new THREE.Raycaster();
  let kind: DragKind | null = null;
  let pointerId = -1;
  let last: [number, number] = [0, 0];

  const local = (ev: PointerEvent): [number, number] => {
    const r = canvas.getBoundingClientRect();
    return [ev.clientX - r.left, ev.clientY - r.top];
  };
  const size = (): [number, number] => {
    const r = canvas.getBoundingClientRect();
    return [Math.max(1, r.width), Math.max(1, r.height)];
  };

  const nearHandle = (px: [number, number]): boolean => {
    const h = o.handlePos();
    if (!h) return false;
    const [w, hgt] = size();
    const p = h.clone().project(cam);
    if (p.z > 1) return false;
    const sx = ((p.x + 1) / 2) * w, sy = ((1 - p.y) / 2) * hgt;
    return Math.hypot(sx - px[0], sy - px[1]) <= (o.hitPx ?? 18);
  };

  /** Point on the handle sphere under the pointer (continuous across the silhouette). */
  const spherePoint = (px: [number, number]): THREE.Vector3 => {
    const [w, hgt] = size();
    ray.setFromCamera(new THREE.Vector2((px[0] / w) * 2 - 1, -(px[1] / hgt) * 2 + 1), cam);
    const org = ray.ray.origin, dir = ray.ray.direction;
    const b = org.dot(dir);
    const c = org.lengthSq() - o.radius * o.radius;
    const disc = b * b - c;
    if (disc >= 0) {
      const s = Math.sqrt(disc);
      const p1 = org.clone().addScaledVector(dir, -b - s);
      const p2 = org.clone().addScaledVector(dir, -b + s);
      const cur = o.handlePos();
      if (cur && p2.distanceTo(cur) < p1.distanceTo(cur)) return p2;
      return p1;
    }
    // the ray misses the sphere: use the closest point of the ray to the centre (on the silhouette)
    return org.clone().addScaledVector(dir, -b).setLength(o.radius);
  };

  /** Arcball vector (world) for a pointer position. */
  const arcball = (px: [number, number]): THREE.Vector3 => {
    const [w, hgt] = size();
    const m = Math.min(w, hgt);
    const x = (2 * px[0] - w) / m, y = (hgt - 2 * px[1]) / m;
    const r2 = x * x + y * y;
    const v = r2 <= 1 ? new THREE.Vector3(x, y, Math.sqrt(1 - r2)) : new THREE.Vector3(x, y, 0).divideScalar(Math.sqrt(r2));
    return v.applyQuaternion(cam.quaternion);
  };

  const setControls = (on: boolean) => { if (scene.controls) scene.controls.enabled = on; };

  const onDown = (ev: PointerEvent) => {
    if (ev.button !== 0 || kind) return;
    const px = local(ev);
    if (nearHandle(px)) kind = 'axis';
    else if (o.objectMode()) kind = 'object';
    else return;
    ev.stopPropagation();
    ev.preventDefault();
    pointerId = ev.pointerId;
    last = px;
    try { canvas.setPointerCapture(ev.pointerId); } catch { /* pointer already released */ }
    setControls(false);
    wrap.style.cursor = 'grabbing';
    o.onDragState?.(kind);
  };

  const onMove = (ev: PointerEvent) => {
    const px = local(ev);
    if (!kind) {
      wrap.style.cursor = nearHandle(px) ? 'grab' : (o.objectMode() ? 'move' : '');
      return;
    }
    if (ev.pointerId !== pointerId) return;
    ev.stopPropagation();
    if (kind === 'axis') {
      const p = spherePoint(px).normalize();
      o.onAxis([p.x, p.y, p.z]);
    } else {
      const v0 = arcball(last), v1 = arcball(px);
      const axis = v0.clone().cross(v1);
      const s = axis.length();
      const angle = Math.atan2(s, Math.min(1, Math.max(-1, v0.dot(v1))));
      if (s > 1e-9 && angle > 1e-6) {
        axis.divideScalar(s);
        o.onRotate([axis.x, axis.y, axis.z], angle);
      }
    }
    last = px;
  };

  const onUp = (ev: PointerEvent) => {
    if (!kind || ev.pointerId !== pointerId) return;
    ev.stopPropagation();
    try { canvas.releasePointerCapture(ev.pointerId); } catch { /* already released */ }
    kind = null;
    pointerId = -1;
    setControls(true);
    wrap.style.cursor = '';
    o.onDragState?.(null);
  };

  const opts = { capture: true } as const;
  wrap.addEventListener('pointerdown', onDown, opts);
  wrap.addEventListener('pointermove', onMove, opts);
  wrap.addEventListener('pointerup', onUp, opts);
  wrap.addEventListener('pointercancel', onUp, opts);
  return {
    get dragging() { return kind; },
    destroy() {
      wrap.removeEventListener('pointerdown', onDown, opts);
      wrap.removeEventListener('pointermove', onMove, opts);
      wrap.removeEventListener('pointerup', onUp, opts);
      wrap.removeEventListener('pointercancel', onUp, opts);
      setControls(true);
    },
  };
}
