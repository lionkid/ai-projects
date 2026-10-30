/** Draggable SE(2) pose: a position handle plus a heading knob. */
import type { Canvas2D, Handle, Pt } from './canvas2d';
import { cssVar } from './dom';

export interface Pose2 { x: number; y: number; theta: number }

export interface PoseHandleOpts {
  onChange?: (p: Pose2) => void;
  color?: string;
  label?: string;
  /** Distance (world units) from the centre to the heading knob. Default 0.5. */
  knob?: number;
  /** Draw a frame triad at the pose (default true). */
  frame?: boolean;
  frameSize?: number;
  enabled?: boolean;
}

export interface PoseHandleHandle {
  pose: Pose2;
  set(p: Partial<Pose2>, o?: { silent?: boolean }): void;
  /** Call inside the Canvas2D draw callback. */
  draw(): void;
  setEnabled(v: boolean): void;
  remove(): void;
}

export function poseHandle(cv: Canvas2D, init: Pose2, opts: PoseHandleOpts = {}): PoseHandleHandle {
  const pose: Pose2 = { ...init };
  const knob = opts.knob ?? 0.5;
  const color = opts.color ?? cssVar('--warn', '#d97706');
  const knobPos = (): Pt => [pose.x + knob * Math.cos(pose.theta), pose.y + knob * Math.sin(pose.theta)];
  const center: Handle = {
    get: () => [pose.x, pose.y],
    set: (p) => { pose.x = p[0]; pose.y = p[1]; opts.onChange?.(pose); },
    color, label: opts.label, radius: 9, enabled: opts.enabled,
  };
  const heading: Handle = {
    get: knobPos,
    set: (p) => { pose.theta = Math.atan2(p[1] - pose.y, p[0] - pose.x); opts.onChange?.(pose); },
    color, radius: 6, enabled: opts.enabled,
  };
  cv.addHandle(center); cv.addHandle(heading);
  return {
    pose,
    set(p, o) { Object.assign(pose, p); if (!o?.silent) opts.onChange?.(pose); cv.requestRender(); },
    draw() {
      cv.line([pose.x, pose.y], knobPos(), { color, width: 1.5, dash: [4, 3] });
      if (opts.frame !== false) cv.frame(pose, opts.frameSize ?? 0.35, { width: 2 });
    },
    setEnabled(v) { center.enabled = v; heading.enabled = v; cv.requestRender(); },
    remove() { cv.removeHandle(center); cv.removeHandle(heading); },
  };
}
