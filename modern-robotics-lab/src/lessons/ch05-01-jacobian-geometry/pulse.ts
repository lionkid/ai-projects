/**
 * Single-joint "pulse" used by the lesson: joint i swings around its start angle while the others
 * stay locked, θ_i(t) = θ_i0 + dir·A·sin(Ωt), so the true joint rate is θ̇_i(t) = dir·A·Ω·cos(Ωt).
 * With A·Ω = 1 rad/s the rate shown on screen (slider, θ̇ vector, V = J θ̇, arrows) is exactly the
 * rate of the motion, starting at ±1 rad/s.
 *
 * A full period swings to both sides of θ_i0; near a joint limit only half a period on the free side
 * is used (sin ≥ 0 on [0, T/2]), so the joint never leaves its range. Full-turn joints are wrapped.
 */
import { wrapToPi } from '@/mr';

/** Period of a full swing (s). */
export const PULSE_T = 2;
export const PULSE_W = (2 * Math.PI) / PULSE_T;
/** Amplitude (rad): A·Ω = 1 rad/s, i.e. about 18°. */
export const PULSE_A = 1 / PULSE_W;

export interface PulsePlan {
  /** +1: first swing towards larger angles; −1: towards smaller angles. */
  dir: number;
  /** PULSE_T (both sides) or PULSE_T / 2 (one side, next to a joint limit). */
  period: number;
  /** Full-turn joint: angles wrapped to (−π, π]. */
  wrap: boolean;
}

export function planPulse(theta0: number, lo: number, hi: number): PulsePlan {
  const wrap = hi - lo >= 2 * Math.PI - 1e-9;
  const upOk = wrap || theta0 + PULSE_A <= hi;
  const downOk = wrap || theta0 - PULSE_A >= lo;
  return { dir: upOk ? 1 : -1, period: upOk && downOk ? PULSE_T : PULSE_T / 2, wrap };
}

/** Joint angle and true joint rate at time t (clamped to [0, period]). */
export function pulseState(plan: PulsePlan, theta0: number, t: number): { theta: number; rate: number } {
  const tc = Math.min(Math.max(t, 0), plan.period);
  const x = theta0 + plan.dir * PULSE_A * Math.sin(PULSE_W * tc);
  return { theta: plan.wrap ? wrapToPi(x) : x, rate: plan.dir * PULSE_A * PULSE_W * Math.cos(PULSE_W * tc) };
}
