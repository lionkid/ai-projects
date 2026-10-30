import { describe, it, expect } from 'vitest';
import { segPointDistance, segCircleIntersects, segSegIntersects, pointInPolygon, segPolygonIntersects, distPointPolygon, rectPolygon, polylineCircleIntersects, circlePolygonIntersects } from './geometry2d';

describe('geometry2d', () => {
  it('segment-point distance', () => {
    expect(segPointDistance([0, 0], [2, 0], [1, 1])).toBeCloseTo(1);
    expect(segPointDistance([0, 0], [2, 0], [3, 0])).toBeCloseTo(1);
    expect(segPointDistance([1, 1], [1, 1], [1, 3])).toBeCloseTo(2);
  });
  it('segment-circle', () => {
    expect(segCircleIntersects([0, 0], [2, 0], [1, 0.5], 0.5)).toBe(true);
    expect(segCircleIntersects([0, 0], [2, 0], [1, 0.6], 0.5)).toBe(false);
  });
  it('segment-segment', () => {
    expect(segSegIntersects([0, 0], [1, 1], [0, 1], [1, 0])).toBe(true);
    expect(segSegIntersects([0, 0], [1, 1], [2, 2], [3, 3])).toBe(false);
    expect(segSegIntersects([0, 0], [1, 1], [1, 1], [2, 0])).toBe(true); // touching
    expect(segSegIntersects([0, 0], [1, 0], [0, 1], [1, 1])).toBe(false); // parallel
  });
  it('point in polygon', () => {
    const sq = rectPolygon(0, 0, 1, 1);
    expect(pointInPolygon([0.5, 0.5], sq)).toBe(true);
    expect(pointInPolygon([1.5, 0.5], sq)).toBe(false);
    expect(pointInPolygon([1, 0.5], sq)).toBe(true); // boundary
    const tri: [number, number][] = [[0, 0], [2, 0], [0, 2]];
    expect(pointInPolygon([0.5, 0.5], tri)).toBe(true);
    expect(pointInPolygon([1.5, 1.5], tri)).toBe(false);
  });
  it('segment / circle vs polygon', () => {
    const sq = rectPolygon(0, 0, 1, 1);
    expect(segPolygonIntersects([-1, 0.5], [2, 0.5], sq)).toBe(true);
    expect(segPolygonIntersects([-1, 2], [2, 2], sq)).toBe(false);
    expect(segPolygonIntersects([0.2, 0.2], [0.8, 0.8], sq)).toBe(true); // fully inside
    expect(distPointPolygon([2, 0.5], sq)).toBeCloseTo(1);
    expect(circlePolygonIntersects([2, 0.5], 1.01, sq)).toBe(true);
    expect(polylineCircleIntersects([[0, 0], [1, 0], [1, 1]], [1.4, 0.5], 0.5)).toBe(true);
  });
});
