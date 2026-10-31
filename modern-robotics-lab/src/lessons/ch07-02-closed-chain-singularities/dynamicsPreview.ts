/**
 * Preview card for the next chapter (dynamics), shown in the last step of ch07-02: a planar 2R arm
 * (two 1 m, 1 kg rods) swinging freely under gravity. The motion is integrated with RK4 (`rk4Step`)
 * on the closed-form planar 2R dynamics (`planar2RAccel`) from @/mr, and the configuration-dependent
 * mass matrix M(θ) (`planar2RDynamics`) is shown live, which is the point the next chapter starts from.
 */
import { el, fmt, mdElement, readout, matrixView, Canvas2D, createLoop, substep } from '@/ui';
import { planar2RParams, planar2RAccel, planar2RDynamics, planarFK, rk4Step, wrapToPi } from '@/mr';

export interface PreviewCard {
  el: HTMLElement;
  start(): void;
  stop(): void;
  destroy(): void;
}

/** Released from rest, bent and lifted; θ1 is measured from +x and gravity points along −y. */
const START: number[] = [-Math.PI / 2 + 1.25, 1.1, 0, 0];
const H = 1 / 240;
/** The readouts below the canvas are refreshed at most this often (ms). */
const UI_INTERVAL = 100;

export function dynamicsPreview(parent: HTMLElement): PreviewCard {
  const P = planar2RParams([1, 1], [1, 1], 'rod');
  let x = START.slice();
  const trail: [number, number][] = [];

  const card = el('div', {
    class: 'mr-note',
    style: { border: '1px solid var(--border)', borderRadius: 'var(--radius-sm)', padding: '0.4rem 0.6rem', margin: '0.5rem 0' },
  });
  card.appendChild(mdElement('**下一章預告：動力學。** 一隻 2R 手臂（兩根 1 m、1 kg 的桿）在重力下自由擺動。下方讀數是質量矩陣 $M(\\theta)$：它的元素隨 $\\theta_2$ 改變，就像這一課的 $J$ 隨構型改變一樣。'));
  // no overlay: the readouts sit below the canvas so they never cover the arm or its pivot
  const cv = new Canvas2D(card, { height: 150, world: { xmin: -2.25, xmax: 2.25, ymin: -2.15, ymax: 0.35 } });
  const foot = el('div', { style: { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: '0.3rem 1rem', marginTop: '0.3rem' } });
  card.appendChild(foot);
  const reset = el('button', { class: 'btn small', type: 'button' }, '⟲ 重新放開');
  foot.appendChild(reset);
  const th2Read = readout(foot, { label: 'θ₂ =', minWidth: '4.5ch' });
  const mView = matrixView(foot, { label: 'M(θ) (kg·m²) =', digits: 2 });
  parent.appendChild(card);

  const f = (_t: number, s: number[]): number[] => {
    const a = planar2RAccel(P, [s[0], s[1]], [s[2], s[3]], [0, 0]);
    return [s[2], s[3], a[0], a[1]];
  };

  let lastUi = -Infinity;
  const refreshReadouts = (force = false): void => {
    const now = performance.now();
    if (!force && now - lastUi < UI_INTERVAL) return;
    lastUi = now;
    th2Read.set(`${fmt((wrapToPi(x[1]) * 180) / Math.PI, 0)}°`);
    mView.set(planar2RDynamics(P, [x[0], x[1]], [0, 0]).M);
  };

  cv.draw = (c) => {
    c.grid(0.5);
    c.line([-0.35, 0], [0.35, 0], { color: '--fg-muted', width: 2 });
    for (let k = -0.3; k <= 0.3 + 1e-9; k += 0.1) c.line([k, 0], [k - 0.07, 0.07], { color: '--fg-muted', width: 1 });
    if (trail.length > 1) c.polyline(trail, { color: '--series-5', width: 1.5, alpha: 0.55 });
    const fk = planarFK([P.L1, P.L2], [x[0], x[1]]);
    const [p0, p1, p2] = fk.joints as [number, number][];
    c.line(p0, p1, { color: '--series-1', width: 7 });
    c.line(p1, p2, { color: '--series-4', width: 6 });
    for (const p of [p0, p1]) {
      c.point(p, 5, { color: '--bg-elev' });
      c.circle(p, 5 / c.scale, { color: '--fg', width: 2 });
    }
    c.point(p2, 4, { color: '--fg' });
  };

  const loop = createLoop((dt) => {
    substep(dt, H, (h) => { x = rk4Step(f, 0, x, h); });
    if (!x.every(Number.isFinite)) x = START.slice();
    const tip = planarFK([P.L1, P.L2], [x[0], x[1]]).tip as [number, number];
    trail.push([tip[0], tip[1]]);
    if (trail.length > 240) trail.shift();
    refreshReadouts();
    cv.requestRender();
  });

  reset.addEventListener('click', () => {
    x = START.slice();
    trail.length = 0;
    refreshReadouts(true);
    cv.requestRender();
  });
  refreshReadouts(true);
  cv.requestRender();

  return {
    el: card,
    start() { loop.start(); },
    stop() { loop.stop(); },
    destroy() { loop.stop(); cv.destroy(); card.remove(); },
  };
}
