import { describe, expect, it } from 'vitest';
import { cellCount, fmtCount, fmtDuration, powLabel } from './scale';

describe('cell counts', () => {
  it('k^n for the numbers quoted in the lesson', () => {
    expect(cellCount(32, 2)).toBe(1024);
    expect(cellCount(100, 6)).toBe(1e12);
    // doubling every axis multiplies the count by 2^n
    expect(cellCount(32, 6) / cellCount(16, 6)).toBe(64);
  });
});

describe('formatting', () => {
  it('counts', () => {
    expect(fmtCount(1024)).toBe('1,024');
    expect(fmtCount(9_999_999)).toBe('9,999,999');
    expect(fmtCount(1e12)).toBe('10¹²');
    expect(fmtCount(32 ** 6)).toBe('1.1 × 10⁹');
    expect(fmtCount(9.97e8)).toBe('10⁹');
    expect(fmtCount(Infinity)).toBe('∞');
    expect(powLabel(32, 6)).toBe('32⁶');
    expect(powLabel(100, 12)).toBe('100¹²');
  });
  it('durations', () => {
    expect(fmtDuration(1024e-6)).toBe('1.0 ms');
    expect(fmtDuration(5e-7)).toBe('0.5 μs');
    expect(fmtDuration(32 ** 3 * 1e-6)).toBe('32.8 ms');
    expect(fmtDuration(32 ** 6 * 1e-6)).toBe('17.9 分鐘');
    expect(fmtDuration(1e12 * 1e-6)).toBe('11.6 天');
    expect(fmtDuration(128 ** 6 * 1e-6)).toBe('50.9 天');
    expect(fmtDuration(3 * 365.25 * 86400)).toBe('3.0 年');
  });
});
