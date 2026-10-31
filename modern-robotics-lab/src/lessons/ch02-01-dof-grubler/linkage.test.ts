import { describe, expect, it } from 'vitest';
import {
  PRESETS, analyze, addLink, addPJoint, addRJoint, arm2RAngles, cloneMechanism, constraintSystem, dragPin,
  grubler, grublerDof, pinMotion, pointGroups, presetByKey, project, removeBody, removeJoint, setArm2R, setGround,
  spatialMobility, worldPt, type Mechanism, type PresetKey, type Pt,
} from './linkage';

/** [Grübler dof, actual (numerical) mobility] for every preset. */
const EXPECTED: Record<PresetKey, [number, number]> = {
  free: [3, 3], twoFree: [6, 6], twoR: [4, 4], twoP: [4, 4], fourbar: [1, 1], slider: [1, 1], fivebar: [2, 2],
  sixbar: [1, 1], parallelogram: [1, 1], stacked: [0, 1], stackedSkew: [0, 0], truss: [0, 0], arm2R: [2, 2],
};

const dist = (a: Pt, b: Pt) => Math.hypot(a[0] - b[0], a[1] - b[1]);
const residual = (m: Mechanism) => constraintSystem(m).C.reduce((s, c) => Math.max(s, Math.abs(c)), 0);

/** Lengths between every pair of pins of every body (must be invariant: bodies are rigid). */
function pinDistances(m: Mechanism): number[] {
  const out: number[] = [];
  for (const b of m.bodies) for (let i = 0; i < b.pins.length; i++) for (let j = i + 1; j < b.pins.length; j++) out.push(dist(worldPt(b, i), worldPt(b, j)));
  return out;
}

describe('presets', () => {
  it('cover every key and are assembled', () => {
    expect(PRESETS.map((p) => p.key).sort()).toEqual(Object.keys(EXPECTED).sort());
    for (const p of PRESETS) expect(residual(p.build()), p.key).toBeLessThan(1e-9);
  });

  it('Grübler count and numerical mobility', () => {
    for (const p of PRESETS) {
      const m = p.build();
      const [g, actual] = EXPECTED[p.key];
      expect(grubler(m).dof, `${p.key} grubler`).toBe(g);
      expect(analyze(m).mobility, `${p.key} mobility`).toBe(actual);
    }
  });

  it('stacked parallelogram has exactly one redundant constraint, the skewed one none', () => {
    const s = analyze(presetByKey('stacked').build());
    expect(s.sumC).toBe(12);
    expect(s.rank).toBe(11);
    expect(s.redundant).toBe(1);
    const k = analyze(presetByKey('stackedSkew').build());
    expect(k.rank).toBe(12);
    expect(k.redundant).toBe(0);
  });

  it('counts the ground in N and k−1 joints at a shared pin', () => {
    const c = grubler(presetByKey('fourbar').build());
    expect([c.N, c.J, c.sumF]).toEqual([4, 4, 4]);
    // Pin a third link onto the crank–coupler joint: one more body, one more joint there.
    const m = presetByKey('fourbar').build();
    const g = pointGroups(m).find((q) => q.rJoints.length === 1 && !q.fixed)!;
    const before = m.joints.length;
    addLink(m, { ref: g.members[0] }, { free: [0, 1.8] }, 'x');
    const sameSpot = pointGroups(m).find((q) => q.members.some((r) => r.body === g.members[0].body && r.pin === g.members[0].pin))!;
    expect(sameSpot.rJoints.length).toBe(2); // three links meet → two joints
    expect(m.joints.length).toBe(before + 1);
    expect(grubler(m).dof).toBe(2);
  });
});

describe('formula values used in the lesson text and quiz', () => {
  it('spatial examples', () => {
    expect(grublerDof(6, 7, 6, 6)).toBe(6); // 6R open chain
    expect(grublerDof(6, 14, 18, 36)).toBe(6); // Stewart–Gough, 6 × UPS
    expect(grublerDof(6, 4, 4, 8)).toBe(2); // RSSR
    expect(grublerDof(6, 4, 4, 4)).toBe(-2); // planar four-bar counted with m = 6
    expect(grublerDof(6, 5, 4, 6)).toBe(6); // RRRS open chain
    expect(grublerDof(6, 4, 4, 6)).toBe(0); // RRRS closed back onto the ground
  });
  it('planar quiz counts', () => {
    expect(grublerDof(3, 6, 7, 7)).toBe(1); // six-bar, ground counted
    expect(grublerDof(3, 5, 7, 7)).toBe(-2); // same, ground forgotten
    expect(grublerDof(3, 6, 7, 7) - grublerDof(3, 5, 5, 5)).toBe(-1); // five-bar + binary link
  });
  it('m = 6 view of planar presets vs the same mechanism in 3-D', () => {
    const four = presetByKey('fourbar').build();
    expect(grubler(four, 6).dof).toBe(-2);
    expect(spatialMobility(four)).toBe(1);
    const free = presetByKey('free').build();
    expect(grubler(free, 6).dof).toBe(6);
    expect(spatialMobility(free)).toBe(6);
    const twoR = presetByKey('twoR').build();
    expect(grubler(twoR, 6).dof).toBe(7);
    expect(spatialMobility(twoR)).toBe(7);
    const arm = presetByKey('arm2R').build();
    expect(grubler(arm, 6).dof).toBe(2);
    expect(spatialMobility(arm)).toBe(2);
  });
});

describe('constraint Jacobian', () => {
  it('matches central finite differences (R and P joints, moving rail)', () => {
    for (const key of ['slider', 'twoP', 'sixbar'] as PresetKey[]) {
      const m = presetByKey(key).build();
      // move away from the build pose so that no term is trivially zero
      m.bodies.forEach((b, i) => { if (!b.fixed) { b.pose.x += 0.03 * i; b.pose.y -= 0.02 * i; b.pose.phi += 0.1 * i; } });
      const sys = constraintSystem(m);
      const h = 1e-6;
      m.bodies.forEach((b, bi) => {
        const k = sys.varBase[bi];
        if (k < 0) return;
        (['x', 'y', 'phi'] as const).forEach((f, c) => {
          const p = cloneMechanism(m), q = cloneMechanism(m);
          p.bodies[bi].pose[f] += h; q.bodies[bi].pose[f] -= h;
          const Cp = constraintSystem(p).C, Cq = constraintSystem(q).C;
          Cp.forEach((v, r) => expect(sys.J[r][k + c], `${key} row ${r} var ${k + c}`).toBeCloseTo((v - Cq[r]) / (2 * h), 6));
        });
      });
    }
  });
});

describe('dragging', () => {
  it('four-bar: crank tip stays on its circle, joints stay assembled, bodies stay rigid', () => {
    const m = presetByKey('fourbar').build();
    const lengths = pinDistances(m);
    const O1 = worldPt(m.bodies[0], 0);
    const tip = { body: 1, pin: 1 };
    const r0 = dist(worldPt(m.bodies[1], 1), O1);
    const start = worldPt(m.bodies[1], 1);
    let target = start;
    for (let k = 0; k < 40; k++) {
      target = [target[0] + 0.02, target[1] - 0.01];
      dragPin(m, tip, target);
      expect(residual(m)).toBeLessThan(1e-8);
    }
    expect(dist(worldPt(m.bodies[1], 1), O1)).toBeCloseTo(r0, 8);
    pinDistances(m).forEach((d, i) => expect(d).toBeCloseTo(lengths[i], 9));
    expect(dist(worldPt(m.bodies[1], 1), start)).toBeGreaterThan(0.3); // it actually moved
  });

  it('crank can be driven through a full turn', () => {
    const m = presetByKey('fourbar').build();
    const O1 = worldPt(m.bodies[0], 0);
    const tip = { body: 1, pin: 1 };
    let ang = Math.atan2(worldPt(m.bodies[1], 1)[1] - O1[1], worldPt(m.bodies[1], 1)[0] - O1[0]);
    for (let k = 0; k < 72; k++) {
      ang += (2 * Math.PI) / 72;
      const t: Pt = [O1[0] + 0.5 * Math.cos(ang), O1[1] + 0.5 * Math.sin(ang)];
      const r = dragPin(m, tip, t);
      expect(r.remaining).toBeLessThan(1e-3);
      expect(residual(m)).toBeLessThan(1e-8);
    }
  });

  it('six-bar and crank–slider cranks also make full turns', () => {
    for (const key of ['sixbar', 'slider'] as PresetKey[]) {
      const m = presetByKey(key).build();
      const O = worldPt(m.bodies[0], 0);
      const tip = { body: 1, pin: 1 };
      const p = worldPt(m.bodies[1], 1);
      const r = dist(p, O);
      let ang = Math.atan2(p[1] - O[1], p[0] - O[0]);
      for (let k = 0; k < 144; k++) {
        ang += (2 * Math.PI) / 144;
        const res = dragPin(m, tip, [O[0] + r * Math.cos(ang), O[1] + r * Math.sin(ang)]);
        expect(res.remaining, `${key} at step ${k}`).toBeLessThan(1e-3);
      }
      expect(residual(m)).toBeLessThan(1e-8);
      expect(analyze(m).mobility).toBe(1);
    }
  });

  it('parallelogram passes its flat change-point without leaving the parallelogram branch', () => {
    const m = presetByKey('parallelogram').build();
    const ref = { body: 2, pin: 1 };
    let p = worldPt(m.bodies[2], 1);
    for (let k = 0; k < 60; k++) {
      p = [p[0] + 0.04, p[1] - 0.04];
      dragPin(m, ref, p);
      expect(m.bodies[2].pose.phi).toBeCloseTo(0, 7); // coupler never rotates
      expect(m.bodies[3].pose.phi).toBeCloseTo(m.bodies[1].pose.phi, 7); // side links stay parallel
    }
    expect(m.bodies[1].pose.phi).toBeLessThan(-0.3); // went through the flat position to the other side
  });

  it('truss: nothing moves and the drag reports locked', () => {
    const m = presetByKey('truss').build();
    const apex = worldPt(m.bodies[1], 1);
    const r = dragPin(m, { body: 1, pin: 1 }, [apex[0] + 0.3, apex[1] + 0.2]);
    expect(r.locked).toBe(true);
    expect(dist(worldPt(m.bodies[1], 1), apex)).toBeLessThan(1e-7);
  });

  it('stacked parallelogram moves although Grübler says 0, and the bars stay parallel', () => {
    const m = presetByKey('stacked').build();
    const g = pointGroups(m).find((q) => q.members.some((r) => r.body === 4 && r.pin === 1))!;
    const ref = g.members.find((r) => !m.bodies[r.body].fixed)!;
    const p0 = worldPt(m.bodies[ref.body], ref.pin);
    let target = p0;
    for (let k = 0; k < 15; k++) { target = [target[0] + 0.03, target[1] - 0.01]; dragPin(m, ref, target); }
    expect(dist(worldPt(m.bodies[ref.body], ref.pin), p0)).toBeGreaterThan(0.2);
    expect(residual(m)).toBeLessThan(1e-8);
    const phis = [1, 3, 4].map((i) => m.bodies[i].pose.phi);
    expect(phis[1]).toBeCloseTo(phis[0], 7);
    expect(phis[2]).toBeCloseTo(phis[0], 7);
    expect(analyze(m).mobility).toBe(1);
  });

  it('a dof-1 point that cannot follow the cursor is still movable (rocker pushed past its limit)', () => {
    // dragPin reports `locked` whenever the pin made no progress; the UI must not read that as dof = 0.
    const m = presetByKey('fourbar').build();
    const ref = { body: 3, pin: 0 }; // coupler–rocker joint B
    const B = worldPt(m.bodies[3], 0);
    const target: Pt = [B[0] + 2, B[1] + 1.5]; // beyond the rocker's swing
    let r = dragPin(m, ref, target);
    for (let k = 0; k < 20; k++) r = dragPin(m, ref, target);
    expect(r.locked).toBe(true);
    expect(pinMotion(m, analyze(m), ref).dof).toBe(1);
    expect(analyze(m).mobility).toBe(1);
    expect(residual(m)).toBeLessThan(1e-8);
  });

  it('skewed version is locked', () => {
    const m = presetByKey('stackedSkew').build();
    const ref = { body: 2, pin: 1 };
    const p0 = worldPt(m.bodies[2], 1);
    const r = dragPin(m, ref, [p0[0] + 0.3, p0[1] - 0.1]);
    expect(r.locked).toBe(true);
    expect(dist(worldPt(m.bodies[2], 1), p0)).toBeLessThan(1e-6);
  });

  it('free rod: a perpendicular pull rotates it about the far end, an axial pull translates it', () => {
    const m = presetByKey('free').build();
    const A = worldPt(m.bodies[1], 0), B = worldPt(m.bodies[1], 1);
    const u: Pt = [(B[0] - A[0]) / dist(A, B), (B[1] - A[1]) / dist(A, B)];
    const n: Pt = [-u[1], u[0]];
    dragPin(m, { body: 1, pin: 0 }, [A[0] + 0.03 * n[0], A[1] + 0.03 * n[1]]);
    expect(dist(worldPt(m.bodies[1], 1), B)).toBeLessThan(2e-3); // far end nearly still
    const m2 = presetByKey('free').build();
    dragPin(m2, { body: 1, pin: 0 }, [A[0] - 0.04 * u[0], A[1] - 0.04 * u[1]]);
    const B2 = worldPt(m2.bodies[1], 1);
    expect(B2[0] - B[0]).toBeCloseTo(-0.04 * u[0], 6);
    expect(B2[1] - B[1]).toBeCloseTo(-0.04 * u[1], 6);
  });

  it('crank–slider: the block stays on its rail and can be pushed out of a dead centre', () => {
    const m = presetByKey('slider').build();
    const block = 3;
    const p0 = worldPt(m.bodies[block], 0);
    dragPin(m, { body: block, pin: 0 }, [p0[0] + 0.25, 0.4]);
    expect(worldPt(m.bodies[block], 0)[1]).toBeCloseTo(0, 9);
    // push to the far dead centre (crank and rod aligned) …
    for (let k = 0; k < 20; k++) dragPin(m, { body: block, pin: 0 }, [2.0, 0]);
    const xMax = worldPt(m.bodies[block], 0)[0];
    expect(xMax).toBeCloseTo(-1.25 + 2.0, 3);
    // … and back again
    for (let k = 0; k < 20; k++) dragPin(m, { body: block, pin: 0 }, [0, 0]);
    expect(worldPt(m.bodies[block], 0)[0]).toBeLessThan(xMax - 0.3);
    expect(residual(m)).toBeLessThan(1e-8);
  });

  it('pin motion: one direction on a dof-1 linkage, two at the five-bar apex, none in a truss', () => {
    const four = presetByKey('fourbar').build();
    const tip = pinMotion(four, analyze(four), { body: 1, pin: 1 });
    expect(tip.dof).toBe(1);
    const O1 = worldPt(four.bodies[0], 0), A = worldPt(four.bodies[1], 1);
    const radial: Pt = [(A[0] - O1[0]) / dist(A, O1), (A[1] - O1[1]) / dist(A, O1)];
    expect(Math.abs(tip.dir![0] * radial[0] + tip.dir![1] * radial[1])).toBeLessThan(1e-6); // tangent to the circle
    const five = presetByKey('fivebar').build();
    expect(pinMotion(five, analyze(five), { body: 2, pin: 1 }).dof).toBe(2);
    expect(pinMotion(five, analyze(five), { body: 1, pin: 1 }).dof).toBe(1);
    const truss = presetByKey('truss').build();
    expect(pinMotion(truss, analyze(truss), { body: 1, pin: 1 }).dof).toBe(0);
  });

  it('five-bar apex can be dragged anywhere nearby (2 dof)', () => {
    const m = presetByKey('fivebar').build();
    const ref = { body: 2, pin: 1 };
    const p0 = worldPt(m.bodies[2], 1);
    for (const t of [[p0[0] + 0.15, p0[1]], [p0[0] + 0.15, p0[1] - 0.2], [p0[0] - 0.1, p0[1] - 0.1]] as Pt[]) {
      for (let k = 0; k < 8; k++) dragPin(m, ref, t);
      expect(dist(worldPt(m.bodies[2], 1), t)).toBeLessThan(1e-4);
    }
  });
});

describe('editing', () => {
  it('a binary link across the five-bar removes one dof', () => {
    const m = presetByKey('fivebar').build();
    const gs = pointGroups(m);
    const ground = gs.find((g) => g.fixed && g.rJoints.length)!.members.find((r) => m.bodies[r.body].fixed)!;
    const apex = gs.find((g) => g.members.some((r) => r.body === 2 && r.pin === 1))!.members[0];
    expect(addLink(m, { ref: ground }, { ref: apex }, 'new')).not.toBeNull();
    expect(project(m)).toBeLessThan(1e-9);
    expect(grubler(m).dof).toBe(1);
    expect(analyze(m).mobility).toBe(1);
  });

  it('an R joint between two free links removes two, a P joint as well', () => {
    const m = presetByKey('twoFree').build();
    expect(addRJoint(m, { body: 1, pin: 1 }, { body: 2, pin: 0 })).toBe(true);
    expect(project(m, 60)).toBeLessThan(1e-9);
    expect(dist(worldPt(m.bodies[1], 1), worldPt(m.bodies[2], 0))).toBeLessThan(1e-9);
    expect(grubler(m).dof).toBe(4);
    expect(analyze(m).mobility).toBe(4);
    expect(addRJoint(m, { body: 1, pin: 1 }, { body: 2, pin: 0 })).toBe(false); // duplicate
    const p = presetByKey('twoFree').build();
    expect(addPJoint(p, { body: 2, pin: 0 }, 1)).toBe(true);
    expect(residual(p)).toBeLessThan(1e-12);
    expect(analyze(p).mobility).toBe(4);
  });

  it('removing a body drops its joints and re-indexes the rest', () => {
    const m = presetByKey('fourbar').build();
    expect(removeBody(m, 0)).toBe(false); // ground
    expect(removeBody(m, 2)).toBe(true); // coupler
    expect(m.bodies.length).toBe(3);
    expect(m.joints.length).toBe(2);
    expect(m.joints.every((j) => j.a < 3 && j.b < 3)).toBe(true);
    expect(grubler(m).dof).toBe(2); // two independent pendulums
    expect(analyze(m).mobility).toBe(2);
    expect(removeJoint(m, 0)).toBe(true);
    expect(grubler(m).dof).toBe(4);
  });

  it('kinematic inversion (another link as ground) keeps the dof', () => {
    for (const key of ['fourbar', 'slider', 'fivebar'] as PresetKey[]) {
      const m = presetByKey(key).build();
      const before = analyze(m).mobility;
      expect(setGround(m, 2)).toBe(true);
      expect(m.bodies.filter((b) => b.fixed).length).toBe(1);
      expect(residual(m)).toBeLessThan(1e-9);
      expect(grubler(m).dof).toBe(grubler(presetByKey(key).build()).dof);
      expect(analyze(m).mobility).toBe(before);
    }
    // a ground without any pin cannot be released (it would float away invisibly)
    expect(setGround(presetByKey('free').build(), 1)).toBe(false);
  });
});

describe('2R arm helpers', () => {
  it('round-trips joint angles and stays assembled', () => {
    const m = presetByKey('arm2R').build();
    for (const th of [[0.3, -1.2], [3.0, 2.5], [-2.9, 0.1]] as [number, number][]) {
      setArm2R(m, th);
      expect(residual(m)).toBeLessThan(1e-12);
      const back = arm2RAngles(m);
      expect(back[0]).toBeCloseTo(th[0], 12);
      expect(back[1]).toBeCloseTo(th[1], 12);
    }
  });
});
