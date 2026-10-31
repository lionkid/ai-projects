/** Fixed-sub-step physics runner with blow-up protection, built on createLoop + substep. */
import { createLoop, type Loop } from './loop';

export interface PhysicsRunnerOpts {
  /** Integration sub-step in seconds (default 1/240). */
  h?: number;
  /** Advance the model by h seconds (called many times per frame). */
  step: (h: number, t: number) => void;
  /** Called once per rendered frame with the simulated time. */
  onFrame?: (t: number, dt: number) => void;
  /** Return false if the state is no longer finite / sane; the runner stops and calls onBlowUp. */
  guard?: () => boolean;
  onBlowUp?: () => void;
  /** Reset callback (also clears time). */
  onReset?: () => void;
  /** Upper bound on simulated seconds per frame (default 0.05). */
  maxDt?: number;
  /**
   * CPU budget per frame in ms (default 8). When the machine cannot keep up, the remaining
   * sub-steps of that frame are dropped, so the simulation runs in slow motion instead of
   * freezing the page.
   */
  budgetMs?: number;
}

export interface PhysicsRunner {
  loop: Loop;
  start(): void;
  stop(): void;
  toggle(): void;
  reset(): void;
  readonly time: number;
  readonly running: boolean;
  readonly blownUp: boolean;
  speed: number;
  /** Advance by a fixed amount while paused (e.g. frame-by-frame). */
  advance(dt: number): void;
}

export function physicsRunner(opts: PhysicsRunnerOpts): PhysicsRunner {
  const h = opts.h ?? 1 / 240;
  let t = 0;
  let blownUp = false;
  const budget = opts.budgetMs ?? 8;
  const advance = (dt: number) => {
    if (blownUp) return;
    const start = performance.now();
    let remaining = dt;
    while (remaining > 1e-12) {
      const hh = Math.min(h, remaining);
      opts.step(hh, t);
      t += hh;
      remaining -= hh;
      if (performance.now() - start > budget) break; // over budget: drop the rest of this frame
    }
    if (opts.guard && !opts.guard()) { blownUp = true; loop.stop(); opts.onBlowUp?.(); }
  };
  const loop = createLoop((dt) => { advance(dt); opts.onFrame?.(t, dt); }, { maxDt: opts.maxDt });
  return {
    loop,
    start: () => { if (!blownUp) loop.start(); },
    stop: () => loop.stop(),
    toggle: () => { if (loop.running) loop.stop(); else if (!blownUp) loop.start(); },
    reset: () => { loop.stop(); loop.reset(); t = 0; blownUp = false; opts.onReset?.(); opts.onFrame?.(0, 0); },
    get time() { return t; },
    get running() { return loop.running; },
    get blownUp() { return blownUp; },
    get speed() { return loop.speed; },
    set speed(s: number) { loop.speed = s; },
    advance: (dt) => { advance(dt); opts.onFrame?.(t, dt); },
  };
}
