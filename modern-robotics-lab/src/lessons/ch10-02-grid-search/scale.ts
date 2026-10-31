/**
 * How grid planning scales with dimension: cell counts k^n and readable zh-TW strings for very
 * large counts and durations (used by the "limits of grids" card of the lesson). Pure, unit-tested.
 */

/** Number of cells of a grid with k cells per axis in `dims` dimensions. */
export const cellCount = (k: number, dims: number): number => k ** dims;

const SUP = '⁰¹²³⁴⁵⁶⁷⁸⁹';
const sup = (e: number): string => String(e).split('').map((ch) => (ch === '-' ? '⁻' : SUP[Number(ch)])).join('');

/** 'k' raised to e with a Unicode superscript, e.g. powLabel(32, 6) = '32⁶'. */
export const powLabel = (k: number, e: number): string => `${k}${sup(e)}`;

/** 1234 → '1,234'; ≥ 10⁷ → '1.2 × 10⁹' (exact powers of ten print as '10¹²'). */
export function fmtCount(x: number): string {
  if (!Number.isFinite(x)) return '∞';
  if (Math.abs(x) < 1e7) return Math.round(x).toLocaleString('en-US');
  const e = Math.floor(Math.log10(Math.abs(x)));
  const m = x / 10 ** e;
  if (Math.abs(m - 1) < 1e-9) return `10${sup(e)}`;
  const ms = m.toFixed(1);
  // 9.96 rounds to "10.0": carry into the exponent
  if (ms === '10.0') return `10${sup(e + 1)}`;
  return `${ms} × 10${sup(e)}`;
}

/** Seconds → a short zh-TW duration such as '33 ms', '18 分鐘', '11.6 天', '1.4 × 10⁵ 年'. */
export function fmtDuration(s: number): string {
  if (!Number.isFinite(s)) return '∞';
  const one = (v: number): string => (v >= 100 ? v.toFixed(0) : v >= 10 ? v.toFixed(1).replace(/\.0$/, '') : v.toFixed(1));
  if (s < 1e-3) return `${one(s * 1e6)} μs`;
  if (s < 1) return `${one(s * 1e3)} ms`;
  if (s < 60) return `${one(s)} 秒`;
  if (s < 3600) return `${one(s / 60)} 分鐘`;
  if (s < 86400) return `${one(s / 3600)} 小時`;
  const days = s / 86400;
  if (days < 365) return `${one(days)} 天`;
  const years = days / 365.25;
  return years < 1e4 ? `${one(years)} 年` : `${fmtCount(years)} 年`;
}
