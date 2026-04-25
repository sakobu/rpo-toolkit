/**
 * Resolve a CSS custom property at module-load time, with an SSR-safe
 * hex fallback.
 *
 * The lookup runs once per call; the returned value is intended to be cached
 * in a module-level `const`. This means a runtime theme switch will NOT
 * propagate to consumers — the app is dark-only by design (see
 * `rpo-app/CLAUDE.md`). If a theme system is ever added, switch consumers to
 * read the token reactively (e.g. via a CSS class on the root) instead of
 * caching the resolved value.
 *
 * Returns `fallback` when running in a non-DOM context (SSR, workers).
 */
export function resolveColor(cssVar: string, fallback: string): string {
  if (typeof window === 'undefined') return fallback;
  const value = getComputedStyle(document.documentElement).getPropertyValue(cssVar).trim();
  return value || fallback;
}
