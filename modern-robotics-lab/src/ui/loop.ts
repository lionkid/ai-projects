/** requestAnimationFrame loop with clamped dt (robust to tab switching). */
export interface Loop {
  start(): void;
  stop(): void;
  toggle(): void;
  readonly running: boolean;
  /** Simulated time accumulated while running (seconds). */
  readonly time: number;
  reset(): void;
  /** Playback speed multiplier applied to dt (default 1). */
  speed: number;
}

export function createLoop(cb: (dt: number, t: number) => void, opts: { maxDt?: number } = {}): Loop {
  const maxDt = opts.maxDt ?? 0.05;
  let raf = 0;
  let last = 0;
  let running = false;
  let time = 0;
  const loop: Loop = {
    speed: 1,
    get running() { return running; },
    get time() { return time; },
    start() {
      if (running) return;
      running = true;
      last = performance.now();
      const tick = (now: number) => {
        if (!running) return;
        const dt = Math.min(maxDt, (now - last) / 1000) * loop.speed;
        last = now;
        time += dt;
        cb(dt, time);
        raf = requestAnimationFrame(tick);
      };
      raf = requestAnimationFrame(tick);
    },
    stop() {
      running = false;
      if (raf) cancelAnimationFrame(raf);
      raf = 0;
    },
    toggle() { running ? loop.stop() : loop.start(); },
    reset() { time = 0; },
  };
  return loop;
}

/** Fixed-step sub-stepping helper: calls `step(h)` as many times as needed to advance `dt`. */
export function substep(dt: number, h: number, step: (h: number) => void): void {
  let remaining = dt;
  while (remaining > 1e-12) {
    const s = Math.min(h, remaining);
    step(s);
    remaining -= s;
  }
}
