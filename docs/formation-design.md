# Formation Design

This document covers the formation-design vocabulary implemented in `rpo-core`. For the
architectural positioning (advisory vs enforced, ground-based vs flight software), see the
**Formation Design** section of [the README](../README.md#formation-design).

The toolkit implements the quasi-nonsingular ROE formation-design vocabulary from D'Amico
2010 directly:

- **e/i vector separation** (D'Amico Eq. 2.22) — the passive-safety metric. A formation
  maintains minimum radial/cross-track separation across phasing as long as the eccentricity
  and inclination vectors are properly separated, independent of along-track drift. The
  planner computes it; the analyst decides whether to act on it.
- **Null-space-projected waypoint enrichment** (D'Amico Eq. 2.17, 2.23) — for each
  user-placed waypoint, the toolkit computes a "safe alternative" ROE that preserves the
  chosen RIC position _exactly_ but adjusts the approach velocity along the null space of the
  position-to-ROE map. Improves e/i separation without moving the waypoint. Shown as an
  advisory card with baseline-vs-enriched comparison.
- **Perch enrichment** (D'Amico Eq. 2.32) — on departure from a far-field approach, the
  perch state's e/i vectors can be adjusted to a user-requested minimum R/C separation. The
  solver returns **both** a baseline (unenriched) and an enriched plan. Default is baseline;
  the analyst explicitly opts in.
- **Transit monitoring** (D'Amico Eqs. 2.28–2.30) — per-leg e/i separation profiles along
  each coast arc with mission-wide minimum flagged. Eq. 2.28 is the J2 secular rate of δα,
  Eq. 2.29 the integrated form δα(t) that yields the per-leg profile, and Eq. 2.30 is the
  relative-perigee drift `dφ/du = (3/2)γ(5cos²i−1)`. Red zones identify when the formation
  would temporarily lose passive-abort capability during a guided approach.
- **Free-drift abort analysis** (D'Amico Eq. 2.33, bounded-motion criterion) — per-leg
  diagnostic showing what happens if the departure burn doesn't fire. Eq. 2.33 is the
  closed-orbit condition `γ·sin(2i)·Δi + (1/7)·Δa/a = 0`; abort analysis evaluates a given
  ROE state against it. Runs alongside the nominal plan, not after it.

## Why advisory, not enforced

Short maneuver legs naturally produce poor intermediate e/i geometry even when operational
safety (3D keep-out distance) is fully satisfied. Enforcing e/i as a hard planning constraint
would reject many otherwise operationally safe guided approaches. The tool enforces what must
not be violated (keep-out distance, checked at every trajectory sample) and evaluates what
requires analyst judgment (passive safety, advisory cards with explicit opt-in).

## Primitive → equation mapping

| Primitive                         | Function                                                       | Source                 |
| --------------------------------- | -------------------------------------------------------------- | ---------------------- |
| e/i vector passive safety         | `assess_safety()`                                              | D'Amico Eq. 2.22       |
| Null-space waypoint enrichment    | `enrich_waypoint()` → `accept_waypoint_enrichment()`           | D'Amico Eq. 2.17, 2.23 |
| Perch enrichment                  | `suggest_enrichment_from_parts()` → `apply_perch_enrichment()` | D'Amico Eq. 2.32       |
| Transit e/i profile               | (inside `plan_waypoint_mission()` output)                      | D'Amico Eqs. 2.28–2.30 |
| Free-drift abort + bounded-motion | `compute_free_drift_analysis()`                                | D'Amico Eq. 2.33       |

## See also

Collision avoidance (COLA) is a separate module — reactive burn planning via inverse GVE
when POCA crosses a threshold, rather than passive-geometry design. Sources: D'Amico §2.4
(Relative Orbit Control), Eqs. 2.38–2.41, 2.44, 2.50–2.56. Code: `rpo-core/mission/avoidance/`.
The shared infrastructure is the GVE B matrix (`dv` → `dROE`) at D'Amico Eq. 2.38, which
lives in `rpo-core/elements/gve.rs` and is used by both formation targeting and avoidance.

## References

- D'Amico, _Autonomous Formation Flying in Low Earth Orbit_, PhD thesis, TU Delft 2010.
  [PDF](references/Damico_PhD.pdf).
