// Seconds is the SI/protocol unit, but UI readouts are friendlier as
// `Xh Ym Zs` — drops the seconds term on minute boundaries and the minutes
// term when the value is whole hours.
export function formatDurationShort(totalSeconds: number): string {
  const h = Math.floor(totalSeconds / 3600);
  const m = Math.floor((totalSeconds % 3600) / 60);
  const s = Math.round(totalSeconds % 60);
  const parts: string[] = [];
  if (h > 0) parts.push(`${h}h`);
  if (m > 0 || (h > 0 && s > 0)) parts.push(`${m}m`);
  if (s > 0 || parts.length === 0) parts.push(`${s}s`);
  return parts.join(' ');
}

// ROE components are dimensionless; values span zero to ~1e-3. Use plain `0`
// for exact zeros (common — most components are zero for V-bar/R-bar perches),
// scientific notation when <1e-3, fixed 4-decimal otherwise.
export function formatRoeComponent(x: number): string {
  if (x === 0) return '0';
  const sign = x < 0 ? '-' : '+';
  const abs = Math.abs(x);
  if (abs < 1e-3) return `${sign}${abs.toExponential(2)}`;
  return `${sign}${abs.toFixed(4)}`;
}
