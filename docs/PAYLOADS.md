# Postman WebSocket Payloads -- Step-by-Step RPO Mission Flow

Copy-paste-ready JSON payloads for testing the RPO WebSocket API via Postman, following the stateful session-based protocol.

## Setup

### 1. Start the server

```bash
cargo run -p rpo-api
```

Server binds to `127.0.0.1:3001` by default. Override with `RPO_BIND` and `RPO_PORT` env vars.

### 2. Health check (HTTP)

```
GET http://127.0.0.1:3001/health
```

Returns `"ok"` if the server is running.

### 3. Connect WebSocket in Postman

```
ws://127.0.0.1:3001/ws
```

All messages are JSON text frames. Send payloads as raw JSON in the message composer.

### 4. Session model

The API is **stateful**: the server holds a session per connection. You set state incrementally (chief, deputy, waypoints, config) and the server computes results from the stored session. No need to resend the full mission definition with every request.

**Session tiers:**

```
Tier 0 - Base inputs:     chief, deputy
Tier 1 - Spacecraft:      chief_config, deputy_config (default: cubesat_6u)
Tier 2 - Transfer design: transfer, perch, lambert_tof_s, lambert_config, proximity, drag_config
Tier 3 - Mission plan:    waypoints, config, propagator, safety_requirements, baseline/enriched missions, selected_variant
Tier 4 - Overlays:        navigation_accuracy, maneuver_uncertainty, monte_carlo_config
```

Changing an upstream tier invalidates downstream computed results. For example, `set_states` clears transfer, drag, and mission.

---

## Scenario A: ISS-like Far-Field Approach

Full far-field flow: set_states -> classify -> compute_transfer -> extract_drag -> set_safety_requirements (optional) -> set_waypoints -> get_trajectory -> update_config(j2_drag) -> validate -> run_mc.

The chief and deputy are ISS-class orbits (~400 km, ~51.6 deg) with ~1500 km separation (far-field).

---

### Step 1: Set States

Upload chief and deputy ECI states. This is always the first message in a session.

#### Send: set_states

```json
{
  "type": "set_states",
  "request_id": 1,
  "chief": {
    "epoch": "2024-01-01T00:00:00 UTC",
    "position_eci_km": [5876.261, 3392.661, 0.0],
    "velocity_eci_km_s": [-2.380512, 4.123167, 6.006917]
  },
  "deputy": {
    "epoch": "2024-01-01T00:00:00 UTC",
    "position_eci_km": [5199.839421, 4281.648523, 1398.070066],
    "velocity_eci_km_s": [-3.993103, 2.970313, 5.76454]
  }
}
```

**Expected response:** `state_updated` confirming what was set and what was invalidated.

```json
{
  "type": "state_updated",
  "request_id": 1,
  "updated": ["chief", "deputy"],
  "invalidated": ["transfer", "drag_config", "mission"]
}
```

---

### Step 1b: Set Spacecraft (optional)

Configure spacecraft properties for drag extraction, validation, and Monte Carlo. Defaults to `cubesat_6u` if never called.

#### Send: set_spacecraft (preset names)

```json
{
  "type": "set_spacecraft",
  "request_id": 2,
  "chief_config": "servicer_500kg",
  "deputy_config": "cubesat_6u"
}
```

#### Send: set_spacecraft (custom configs)

```json
{
  "type": "set_spacecraft",
  "request_id": 2,
  "chief_config": {
    "custom": {
      "dry_mass_kg": 500.0,
      "drag_area_m2": 1.0,
      "coeff_drag": 2.2,
      "srp_area_m2": 1.0,
      "coeff_reflectivity": 1.8
    }
  },
  "deputy_config": {
    "custom": {
      "dry_mass_kg": 12.0,
      "drag_area_m2": 0.06,
      "coeff_drag": 2.2,
      "srp_area_m2": 0.06,
      "coeff_reflectivity": 1.8
    }
  }
}
```

#### Spacecraft config options

| Format        | Example                 | Description                     |
| ------------- | ----------------------- | ------------------------------- |
| Preset string | `"cubesat_6u"`          | 12 kg, 0.06 m^2, Cd=2.2, Cr=1.5 |
| Preset string | `"servicer_500kg"`      | 500 kg, 1.0 m^2, Cd=2.2, Cr=1.5 |
| Custom object | `{ "custom": { ... } }` | Full custom spacecraft          |

**Expected response:**

```json
{
  "type": "state_updated",
  "request_id": 2,
  "updated": ["chief_config", "deputy_config"],
  "invalidated": ["drag_config"]
}
```

---

### Step 2: Classify Separation

Determines if the deputy is in proximity or far-field. Reads chief/deputy from the session. Returns immediately.

#### Send: classify

```json
{
  "type": "classify",
  "request_id": 3
}
```

**Expected response:** `classify_result` with `phase.far_field` (separation ~1500 km).

```json
{
  "type": "classify_result",
  "request_id": 3,
  "phase": {
    "far_field": {
      "chief_elements": {
        "a_km": 6886.6,
        "e": 0.001,
        "i_rad": 1.7001,
        "raan_rad": 0.0,
        "aop_rad": 0.0,
        "mean_anomaly_rad": 0.5236
      },
      "deputy_elements": { "...": "..." },
      "separation_km": 1532.4,
      "delta_r_over_r": 0.2225
    }
  }
}
```

---

### Step 3: Compute Lambert Transfer

Designs the Lambert transfer from far-field deputy to a perch point near the chief. Optional fields override session defaults. Returns immediately.

#### Send: compute_transfer (V-bar perch, 1 hour TOF)

```json
{
  "type": "compute_transfer",
  "request_id": 4,
  "perch": { "v_bar": { "along_track_km": 5.0 } },
  "lambert_tof_s": 3600.0
}
```

#### Perch geometry variants

Replace the `"perch"` field to try different perch types:

**V-bar hold** (along-track offset, positive = ahead of chief):

```json
"perch": { "v_bar": { "along_track_km": 5.0 } }
```

**R-bar hold** (radial offset, positive = above chief):

```json
"perch": { "r_bar": { "radial_km": 1.0 } }
```

**Custom ROE state** (specify exact relative orbit elements):

```json
"perch": {
  "custom": {
    "da": 0.0,
    "dlambda": 0.000726,
    "dex": 0.0,
    "dey": 0.0,
    "dix": 0.0,
    "diy": 0.0
  }
}
```

#### Lambert config options

Add `"lambert_config"` to control transfer direction and revolutions:

```json
{
  "type": "compute_transfer",
  "request_id": 5,
  "perch": { "v_bar": { "along_track_km": 5.0 } },
  "lambert_tof_s": 5400.0,
  "lambert_config": {
    "direction": "short_way",
    "revolutions": 0
  }
}
```

**`direction` options:** `"auto"` (default), `"short_way"`, `"long_way"`

**`revolutions`:** `0` = direct transfer, `1`+ = multi-revolution

#### Response format

```json
{
  "type": "transfer_computed",
  "request_id": 4,
  "result": {
    "plan": {
      "phase": { "far_field": { "...": "..." } },
      "transfer": {
        "departure_state": { "...": "..." },
        "arrival_state": { "...": "..." },
        "departure_dv_eci_km_s": [0.1, -0.2, 0.3],
        "arrival_dv_eci_km_s": [-0.1, 0.15, -0.25],
        "total_dv_km_s": 1.234,
        "tof_s": 3600.0,
        "c3_km2_s2": 5.67,
        "direction": "short_way"
      },
      "perch_roe": {
        "da": 0.0,
        "dlambda": 0.000726,
        "dex": 0.0,
        "dey": 0.0,
        "dix": 0.0,
        "diy": 0.0
      },
      "chief_at_arrival": { "a_km": 6886.6, "e": 0.001, "...": "..." }
    },
    "perch_chief": {
      "epoch": "...",
      "position_eci_km": ["..."],
      "velocity_eci_km_s": ["..."]
    },
    "perch_deputy": {
      "epoch": "...",
      "position_eci_km": ["..."],
      "velocity_eci_km_s": ["..."]
    },
    "arrival_epoch": "2024-01-01T01:00:00 UTC",
    "lambert_dv_km_s": 1.234
  }
}
```

---

### Step 3.5: Extract Drag (async, ~3 seconds)

Runs nyx full-physics simulation to extract differential drag rates. Background job. Uses perch states (post-Lambert) in far-field, original states in proximity. If spacecraft configs are identical, returns zero-rate immediately (no differential drag).

#### Send: extract_drag

```json
{
  "type": "extract_drag",
  "request_id": 6
}
```

**Expected response:**

```json
{
  "type": "drag_result",
  "request_id": 6,
  "drag": {
    "da_dot": -1.23e-12,
    "dex_dot": 4.56e-13,
    "dey_dot": -7.89e-14
  }
}
```

The drag result is stored in the session. Use `update_config` with `"propagator": "j2_drag"` to activate it.

---

### Step 3.6: Set Safety Requirements (optional, enables formation design)

Configure formation design enrichment. When set, `set_waypoints` returns both a baseline plan (unenriched) and an enriched plan with safe e/i vectors. Use `select_plan` to choose which is active for downstream operations.

#### Send: set_safety_requirements (enable)

```json
{
  "type": "set_safety_requirements",
  "request_id": 7,
  "requirements": {
    "min_separation_km": 0.15,
    "alignment": "auto"
  }
}
```

**`min_separation_km`:** Minimum R/C separation (km). Maps to `d_min` in D'Amico Eq. 2.22.

**`alignment`:** E/I vector alignment strategy:

- `"parallel"` (default) -- parallel e/i vectors, maximizes passive safety
- `"anti_parallel"` -- anti-parallel e/i vectors, also maximizes passive safety
- `"auto"` -- picks whichever requires the smallest perturbation

**Expected response:**

```json
{
  "type": "state_updated",
  "request_id": 7,
  "updated": ["safety_requirements"],
  "invalidated": ["mission"]
}
```

#### Send: set_safety_requirements (clear)

```json
{
  "type": "set_safety_requirements",
  "request_id": 8,
  "requirements": null
}
```

---

### Step 4: Set Waypoints (plan mission)

Set waypoint targets and plan/replan the mission. The server uses the stored transfer result to construct the departure state and runs Newton-Raphson targeting for each leg.

#### Send: set_waypoints (full plan, 3 legs)

```json
{
  "type": "set_waypoints",
  "request_id": 10,
  "waypoints": [
    {
      "label": "Approach to 2 km",
      "position_ric_km": [0.5, 2.0, 0.5],
      "velocity_ric_km_s": [0.0, 0.0, 0.0],
      "tof_s": 4200.0
    },
    {
      "label": "Close to 0.5 km",
      "position_ric_km": [0.5, 0.5, 0.5],
      "velocity_ric_km_s": [0.0, 0.001, 0.0],
      "tof_s": 4200.0
    },
    {
      "label": "Retreat to 5 km",
      "position_ric_km": [0.5, 5.0, 0.5],
      "velocity_ric_km_s": [0.0, -0.001, 0.0],
      "tof_s": 4200.0
    }
  ]
}
```

#### Send: set_waypoints (auto-optimized TOF)

Omit `tof_s` on a waypoint to let the solver optimize it:

```json
{
  "type": "set_waypoints",
  "request_id": 11,
  "waypoints": [
    {
      "label": "Approach to 2 km",
      "position_ric_km": [0.5, 2.0, 0.5],
      "tof_s": null
    }
  ]
}
```

#### Send: set_waypoints (incremental replan -- moved waypoint)

When a user drags a waypoint in the 3D view, use `changed_from` so earlier legs are cached:

```json
{
  "type": "set_waypoints",
  "request_id": 12,
  "waypoints": [
    {
      "label": "Approach to 2 km",
      "position_ric_km": [0.5, 2.0, 0.5],
      "tof_s": 4200.0
    },
    {
      "label": "MOVED -- Close to 1 km instead",
      "position_ric_km": [0.3, 1.0, 0.3],
      "tof_s": 4200.0
    },
    {
      "label": "Retreat to 5 km",
      "position_ric_km": [0.5, 5.0, 0.5],
      "tof_s": 4200.0
    }
  ],
  "changed_from": 1
}
```

`changed_from`: 0-based index of the first changed waypoint. Legs before this index are cached; legs from this index onward are re-solved.

#### Waypoint fields

| Field               | Required | Default                | Notes                                                                                                                                                                                                    |
| ------------------- | -------- | ---------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `position_ric_km`   | yes      | --                     | `[radial, in-track, cross-track]` in km                                                                                                                                                                  |
| `velocity_ric_km_s` | no       | `null` (position-only) | If omitted: position-only waypoint (zero-velocity hold for targeting, 3-DOF null-space freedom for formation enrichment). If set: velocity-constrained target; enrichment becomes advisory-only for that waypoint. |
| `tof_s`             | no       | `null` (auto-optimize) | Time of flight for this leg                                                                                                                                                                              |
| `label`             | no       | `null`                 | Human-readable label                                                                                                                                                                                     |

#### Response format

The response is a dual-plan `plan_result`. When `safety_requirements` is set, both `baseline` (unenriched) and `enriched` (safe e/i vectors) plans are returned with their respective formation design reports:

```json
{
  "type": "plan_result",
  "request_id": 10,
  "baseline": {
    "phase": { "far_field": { "...": "..." } },
    "transfer_summary": {
      "total_dv_km_s": 1.234,
      "tof_s": 3600.0,
      "direction": "short_way",
      "arrival_epoch": "2024-01-01T01:00:00 UTC"
    },
    "perch_roe": {
      "da": 0.0,
      "dlambda": 0.000726,
      "dex": 0.0,
      "dey": 0.0,
      "dix": 0.0,
      "diy": 0.0
    },
    "legs": [{ "...": "baseline legs" }],
    "total_dv_km_s": 1.237,
    "total_duration_s": 16200.0
  },
  "enriched": {
    "phase": { "far_field": { "...": "..." } },
    "perch_roe": {
      "da": 0.0,
      "dlambda": 0.000726,
      "dex": 0.0,
      "dey": 2.18e-5,
      "dix": 0.0,
      "diy": 2.18e-5
    },
    "legs": [{ "...": "enriched legs" }],
    "total_dv_km_s": 1.238,
    "total_duration_s": 16200.0
  },
  "baseline_formation": {
    "perch": { "baseline": { "...": "geometric ROE" } },
    "waypoints": ["..."],
    "transit_safety": ["..."],
    "mission_min_ei_separation_km": 0.0
  },
  "enriched_formation": {
    "perch": {
      "enriched": {
        "baseline_roe": { "...": "geometric ROE" },
        "roe": { "...": "enriched ROE" },
        "de_magnitude_km": 0.15,
        "di_magnitude_km": 0.15,
        "min_rc_separation_km": 0.15,
        "alignment": "parallel"
      }
    },
    "waypoints": ["..."],
    "transit_safety": ["..."],
    "mission_min_ei_separation_km": 0.142
  }
}
```

Without `safety_requirements`, only `baseline` is present (no `enriched`, `baseline_formation`, or `enriched_formation` fields).

---

### Step 4a: Select Plan (optional)

After receiving the dual-plan response, select which plan is active for downstream operations. Default is `baseline`.

**Send:**

```json
{
  "type": "select_plan",
  "request_id": 11,
  "variant": "enriched"
}
```

**Receive:**

```json
{
  "type": "plan_selected",
  "request_id": 11,
  "variant": "enriched"
}
```

All subsequent `validate`, `run_mc`, `get_trajectory`, `get_formation_design`, etc. will operate on the enriched plan.

---

### Step 4b: On-Demand Data Fetches (parallel)

After planning, fetch trajectory, eclipse, and covariance data on demand. These can be sent in parallel -- they all read from the stored mission without mutating.

#### Send: get_trajectory (all legs, resampled)

```json
{
  "type": "get_trajectory",
  "request_id": 20,
  "max_points": 100
}
```

#### Send: get_trajectory (specific legs)

```json
{
  "type": "get_trajectory",
  "request_id": 21,
  "legs": [0, 2],
  "max_points": 50
}
```

**Response:**

```json
{
  "type": "trajectory_data",
  "request_id": 20,
  "legs": [
    {
      "leg_index": 0,
      "points": [
        {
          "elapsed_s": 0.0,
          "position_ric_km": [0.0, 5.0, 0.0],
          "velocity_ric_km_s": [0.001, -0.002, 0.0005]
        },
        {
          "elapsed_s": 42.0,
          "position_ric_km": [0.02, 4.9, 0.01],
          "velocity_ric_km_s": [0.001, -0.002, 0.0005]
        },
        "..."
      ]
    }
  ]
}
```

#### Send: get_free_drift_trajectory (abort-case analysis)

Computes the free-drift trajectory for each leg (what happens if the departure burn doesn't fire). Returns trajectory points for visualization and safety summaries including a bounded-motion diagnostic.

```json
{
  "type": "get_free_drift_trajectory",
  "request_id": 25,
  "max_points": 100
}
```

#### Send: get_free_drift_trajectory (specific legs)

```json
{
  "type": "get_free_drift_trajectory",
  "request_id": 26,
  "legs": [0, 1],
  "max_points": 50
}
```

**Response:**

```json
{
  "type": "free_drift_data",
  "request_id": 25,
  "legs": [
    {
      "leg_index": 0,
      "points": [
        {
          "elapsed_s": 0.0,
          "position_ric_km": [0.0, 5.0, 0.0],
          "velocity_ric_km_s": [0.001, -0.002, 0.0005]
        },
        "..."
      ]
    }
  ],
  "analyses": [
    {
      "leg_index": 0,
      "min_distance_3d_km": 5.0,
      "min_rc_separation_km": 0.71,
      "min_ei_separation_km": 0.67,
      "bounded_motion_residual": 4.2e-5
    }
  ]
}
```

**`bounded_motion_residual`:** D'Amico Eq. 2.33 diagnostic. Zero = bounded relative motion (passively safe); nonzero = secular along-track drift. Useful for determining if a missed burn is recoverable.

---

#### Send: get_poca (Brent-refined closest approach)

Computes the refined closest-approach point for each leg using Brent's method on range rate. Returns the exact time, distance, and RIC position of the closest approach, which improves on the grid-sampled minimum distance from the safety analysis.

```json
{
  "type": "get_poca",
  "request_id": 27
}
```

#### Send: get_poca (specific legs)

```json
{
  "type": "get_poca",
  "request_id": 28,
  "legs": [1, 2]
}
```

**Response:**

```json
{
  "type": "poca_data",
  "request_id": 27,
  "points": [
    {
      "leg_index": 1,
      "elapsed_s": 1268.4,
      "distance_km": 1.342,
      "position_ric_km": [0.1562, 1.2677, -0.4117],
      "is_global_minimum": false
    },
    {
      "leg_index": 2,
      "elapsed_s": 569.1,
      "distance_km": 0.1852,
      "position_ric_km": [-0.0722, 0.129, 0.1115],
      "is_global_minimum": true
    }
  ]
}
```

Legs with no close approach (diverging geometry) produce no points for that leg index. The `is_global_minimum` flag is set on the single closest approach across all legs in the mission.

---

#### Send: run_cola (collision avoidance)

Computes minimum-Δv avoidance maneuvers for POCA violations with multi-leg post-avoidance verification. Requires a planned mission with safety config enabled. For each leg where the POCA distance is below `target_distance_km`, computes the optimal impulsive maneuver, then cascades the perturbation through downstream legs to detect secondary conjunctions.

> **Note:** COLA also auto-computes during `set_waypoints` when `config.safety` is present and POCA violations exist. The `run_cola` message provides explicit control over the target distance and budget.

```json
{
  "type": "run_cola",
  "request_id": 29,
  "config": {
    "target_distance_km": 0.3,
    "max_dv_km_s": 0.01
  }
}
```

#### ColaConfig fields

| Field                | Required | Description                                       |
| -------------------- | -------- | ------------------------------------------------- |
| `target_distance_km` | yes      | Desired minimum POCA distance after avoidance     |
| `max_dv_km_s`        | yes      | Maximum available delta-v budget for the maneuver |

**Response:**

```json
{
  "type": "cola_result",
  "request_id": 29,
  "maneuvers": [
    {
      "leg_index": 2,
      "dv_ric_km_s": [-0.000715, 0.0, 0.000486],
      "maneuver_location_rad": 1.795,
      "post_avoidance_poca_km": 0.282,
      "fuel_cost_km_s": 0.000864,
      "correction_type": "combined"
    }
  ],
  "secondary_conjunctions": [],
  "skipped": []
}
```

#### Response fields

**`maneuvers[]`** — avoidance maneuvers for POCA violations:

| Field                    | Description                                                             |
| ------------------------ | ----------------------------------------------------------------------- |
| `leg_index`              | 0-based leg index where the violation was found                         |
| `dv_ric_km_s`            | Avoidance delta-v in RIC frame `[radial, in-track, cross-track]` (km/s) |
| `maneuver_location_rad`  | Mean argument of latitude at the optimal burn location (rad)            |
| `post_avoidance_poca_km` | Verified post-avoidance POCA distance (km), from re-propagation         |
| `fuel_cost_km_s`         | Total delta-v magnitude (km/s)                                          |
| `correction_type`        | `"in_plane"`, `"cross_track"`, or `"combined"`                          |

**`secondary_conjunctions[]`** — downstream violations created by avoidance maneuvers (omitted when empty):

| Field                | Description                                        |
| -------------------- | -------------------------------------------------- |
| `original_leg_index` | Leg where the avoidance maneuver was applied       |
| `violated_leg_index` | Downstream leg where a new violation was detected  |
| `distance_km`        | Closest approach distance on the violated leg (km) |
| `elapsed_s`          | Elapsed time from downstream leg start (s)         |
| `position_ric_km`    | RIC position at closest approach `[R, I, C]` (km)  |

**`skipped[]`** — legs where COLA was attempted but failed (omitted when empty):

| Field           | Description                                         |
| --------------- | --------------------------------------------------- |
| `leg_index`     | Leg with the unaddressed POCA violation             |
| `error_message` | Why COLA failed (e.g., budget exceeded, degenerate) |

Empty `maneuvers` array means no POCA violations exceeded the threshold, or all violated legs appear in `skipped` (e.g., budget exceeded).

---

#### Send: get_formation_design (on-demand formation analysis)

Recomputes the formation design report from current session state. Useful after changing `safety_requirements` without replanning, or to inspect formation design independently.

```json
{
  "type": "get_formation_design",
  "request_id": 35
}
```

**Response:**

```json
{
  "type": "formation_design_result",
  "request_id": 35,
  "report": {
    "perch": {
      "status": "enriched",
      "roe": {
        "da": 0.0,
        "dlambda": 0.000726,
        "dex": 0.0,
        "dey": 2.18e-5,
        "dix": 0.0,
        "diy": 2.18e-5
      },
      "de_magnitude_km": 0.15,
      "di_magnitude_km": 0.15,
      "min_rc_separation_km": 0.15,
      "alignment": "parallel"
    },
    "waypoints": ["...per-leg advisory enrichment..."],
    "transit_safety": [
      {
        "min_ei_separation_km": 0.089,
        "min_elapsed_s": 2100.0,
        "min_phase_angle_rad": -1.51,
        "satisfies_requirement": false,
        "threshold_km": 0.1,
        "profile": ["...per-sample EiSample entries..."]
      },
      {
        "min_ei_separation_km": 0.142,
        "min_elapsed_s": 1800.0,
        "min_phase_angle_rad": 0.12,
        "satisfies_requirement": true,
        "threshold_km": 0.1,
        "profile": ["..."]
      }
    ],
    "mission_min_ei_separation_km": 0.089,
    "drift_prediction": {
      "predicted_min_ei_km": 0.135,
      "predicted_phase_angle_rad": 0.0
    }
  }
}
```

`drift_prediction` is a top-level field on the report (not per-leg) and predicts e/i separation for the **leg-1 coast arc only**. It is omitted when leg-1 TOF is outside the J2 drift compensation regime (arc exceeds 10 orbital periods or near-critical inclination). When present, it reports the predicted e/i state at **mid-transit** of leg 1, after pre-rotating departure e/i phases to compensate for J2 perigee drift; `predicted_phase_angle_rad` is approximately zero when compensation succeeds (parallel alignment at mid-transit).

#### Send: get_safe_alternative (single waypoint)

Shows what a specific waypoint's ROE would look like with safe e/i vectors. Does not modify the mission.

```json
{
  "type": "get_safe_alternative",
  "request_id": 36,
  "waypoint_index": 0
}
```

**Response:**

```json
{
  "type": "safe_alternative_result",
  "request_id": 36,
  "enriched": {
    "roe": {
      "da": 0.0,
      "dlambda": 0.000291,
      "dex": 1.5e-5,
      "dey": 2.18e-5,
      "dix": 0.0,
      "diy": 2.18e-5
    },
    "baseline_roe": {
      "da": 0.0,
      "dlambda": 0.000291,
      "dex": 0.0,
      "dey": 0.0,
      "dix": 0.0,
      "diy": 0.0
    },
    "position_ric_km": { "x": 0.5, "y": 2.0, "z": 0.5 },
    "enriched_ei": {
      "min_separation_km": 0.15,
      "de_magnitude": 2.18e-5,
      "di_magnitude": 2.18e-5,
      "phase_angle_rad": 0.0
    },
    "baseline_ei": {
      "min_separation_km": 0.0,
      "de_magnitude": 0.0,
      "di_magnitude": 0.0,
      "phase_angle_rad": 0.0
    },
    "perturbation_norm": 3.1e-5,
    "mode": "velocity_constrained",
    "resolved_alignment": "parallel"
  }
}
```

#### Send: accept_waypoint_enrichment

Accept the safe alternative shown above and replan the mission with the enriched waypoint velocity.

```json
{
  "type": "accept_waypoint_enrichment",
  "request_id": 37,
  "waypoint_index": 0
}
```

**Response:**

```json
{
  "type": "waypoint_enrichment_accepted",
  "request_id": 37,
  "result": {
    "phase": "proximity",
    "transfer_summary": null,
    "perch_roe": {
      "da": 0.000145,
      "dlambda": 0.000291,
      "dex": 1.5e-5,
      "dey": 2.18e-5,
      "dix": 0.0,
      "diy": 2.18e-5
    },
    "legs": [
      {
        "leg_index": 0,
        "from_label": "Perch",
        "to_label": "WP1",
        "departure_dv_km_s": 0.00312,
        "arrival_dv_km_s": 0.00089,
        "total_dv_km_s": 0.00401,
        "tof_s": 4200.0,
        "iterations": 6,
        "position_error_km": 2.1e-7
      }
    ],
    "total_dv_km_s": 0.00401,
    "total_duration_s": 4200.0,
    "safety": null
  }
}
```

After acceptance, the dual-plan is collapsed: the enriched trajectory is the new baseline. Subsequent `get_formation_design`, `validate`, and `run_mc` operate on the accepted plan.

---

#### Send: get_eclipse

```json
{
  "type": "get_eclipse",
  "request_id": 22
}
```

**Response:**

```json
{
  "type": "eclipse_data",
  "request_id": 22,
  "transfer": {
    "summary": {
      "intervals": [],
      "total_shadow_duration_s": 0.0,
      "...": "..."
    },
    "...": "..."
  },
  "mission": { "summary": { "...": "..." }, "legs": [] }
}
```

#### Send: get_covariance

Requires `navigation_accuracy` to be set via `update_config` first (see [Step 4c](#step-4c-update-config-re-solve-all-legs)). If not set, the server returns a `missing_session_state` error. Example prerequisite:

```json
{
  "type": "update_config",
  "request_id": 22,
  "navigation_accuracy": {
    "position_sigma_ric_km": [0.1, 0.1, 0.1],
    "velocity_sigma_ric_km_s": [0.0001, 0.0001, 0.0001]
  }
}
```

Then:

```json
{
  "type": "get_covariance",
  "request_id": 23
}
```

**Response:**

```json
{
  "type": "covariance_data",
  "request_id": 23,
  "report": {
    "legs": ["..."],
    "terminal_3sigma_ric_km": [0.3, 0.3, 0.3],
    "...": "..."
  }
}
```

---

### Step 4c: Update Config (re-solve all legs)

When the user changes safety thresholds, propagator, or covariance settings, `update_config` applies the changes and replans if needed.

#### Send: update_config (switch to J2+Drag after drag_result arrives)

```json
{
  "type": "update_config",
  "request_id": 25,
  "propagator": "j2_drag"
}
```

`"j2_drag"` resolves against the session's stored drag config (from `extract_drag`). Error if no drag has been extracted.

**Response:** `plan_result` (mission is replanned with J2+Drag propagator).

#### Send: update_config (safety thresholds + covariance overlays)

```json
{
  "type": "update_config",
  "request_id": 26,
  "config": {
    "safety": {
      "min_ei_separation_km": 0.2,
      "min_distance_3d_km": 0.1
    }
  },
  "navigation_accuracy": {
    "position_sigma_ric_km": [0.1, 0.1, 0.1],
    "velocity_sigma_ric_km_s": [0.0001, 0.0001, 0.0001]
  },
  "maneuver_uncertainty": {
    "magnitude_sigma": 0.01,
    "pointing_sigma_rad": 0.01745
  }
}
```

**Response:** `state_updated` if only overlays changed (no replan needed), or `plan_result` if config/propagator/proximity changed and a mission exists.

#### Send: update_config (classification thresholds)

```json
{
  "type": "update_config",
  "request_id": 27,
  "proximity": {
    "roe_threshold": 0.01
  }
}
```

---

### Step 5: Validate (full-physics, background job)

Per-leg nyx validation with full force models (J2, drag, SRP, Sun/Moon third-body). Background job with progress streaming. Reads mission and spacecraft configs from the session.

#### Send: validate

Requires a planned mission in the session. If not set, the server returns a `missing_session_state` error. Build the mission by sending `set_waypoints` first (see [Step 4](#step-4-set-waypoints-plan-mission)):

```json
{
  "type": "set_waypoints",
  "request_id": 29,
  "waypoints": [
    {
      "label": "Approach to 2 km",
      "position_ric_km": [0.5, 2.0, 0.5],
      "tof_s": 4200.0
    }
  ]
}
```

Then:

```json
{
  "type": "validate",
  "request_id": 30,
  "samples_per_leg": 50,
  "auto_drag": false
}
```

**`samples_per_leg`:** Number of comparison points per leg (default: 50). More = finer comparison.

**`auto_drag`:** If `true`, auto-derives drag rates and replans before validation (ephemeral -- does not modify session state).

#### Send: validate (with auto-drag)

```json
{
  "type": "validate",
  "request_id": 31,
  "auto_drag": true
}
```

#### Response sequence

```json
{"type": "progress", "request_id": 30, "phase": "validate", "detail": "Validating leg 1/3", "fraction": 0.33}
{"type": "progress", "request_id": 30, "phase": "validate", "detail": "Validating leg 2/3", "fraction": 0.67}
{"type": "heartbeat", "seq": 1}
{"type": "progress", "request_id": 30, "phase": "validate", "detail": "Validating leg 3/3", "fraction": 1.0}
```

Final:

```json
{
  "type": "validation_result",
  "request_id": 30,
  "report": {
    "leg_points": [
      [
        {
          "elapsed_s": 0.0,
          "analytical_ric": {},
          "numerical_ric": {},
          "position_error_km": 0.01,
          "velocity_error_km_s": 0.0001
        }
      ]
    ],
    "max_position_error_km": 0.15,
    "mean_position_error_km": 0.05,
    "rms_position_error_km": 0.07,
    "max_velocity_error_km_s": 0.001,
    "analytical_safety": { "...": "..." },
    "numerical_safety": { "...": "..." },
    "chief_config": {
      "dry_mass_kg": 500.0,
      "drag_area_m2": 1.0,
      "coeff_drag": 2.2,
      "srp_area_m2": 1.0,
      "coeff_reflectivity": 1.8
    },
    "deputy_config": {
      "dry_mass_kg": 12.0,
      "drag_area_m2": 0.06,
      "coeff_drag": 2.2,
      "srp_area_m2": 0.06,
      "coeff_reflectivity": 1.8
    },
    "eclipse_validation": null
  }
}
```

---

### Step 6: Monte Carlo (full-physics ensemble, background job)

Full-physics Monte Carlo with dispersed initial states, maneuver execution errors, and spacecraft property variations. Background job. Requires `monte_carlo` config -- pass it inline or set it via `update_config` beforehand.

#### Send: run_mc (closed-loop, 100 samples)

```json
{
  "type": "run_mc",
  "request_id": 40,
  "monte_carlo": {
    "num_samples": 100,
    "dispersions": {
      "state": {
        "position_radial_km": { "gaussian": { "sigma": 0.1 } },
        "position_intrack_km": { "gaussian": { "sigma": 0.1 } },
        "position_crosstrack_km": { "gaussian": { "sigma": 0.1 } },
        "velocity_radial_km_s": { "gaussian": { "sigma": 0.0001 } },
        "velocity_intrack_km_s": { "gaussian": { "sigma": 0.0001 } },
        "velocity_crosstrack_km_s": { "gaussian": { "sigma": 0.0001 } }
      },
      "maneuver": {
        "magnitude_sigma": 0.01,
        "pointing_sigma_rad": 0.01745
      }
    },
    "mode": "closed_loop",
    "seed": 42,
    "trajectory_steps": 100
  },
  "auto_drag": false
}
```

#### Monte Carlo config options

**`mode`:** `"open_loop"` (apply nominal maneuvers to dispersed states) or `"closed_loop"` (re-target from each dispersed initial state).

**`seed`:** Deterministic seed for reproducibility. Same seed = same results.

**`trajectory_steps`:** Number of trajectory points per leg for dispersion envelope computation.

**Distribution types** for dispersions:

```json
{ "gaussian": { "sigma": 0.1 } }
```

```json
{ "uniform": { "half_width": 0.2 } }
```

#### Adding spacecraft dispersions

Add `"spacecraft"` to the `"dispersions"` object:

```json
"dispersions": {
  "state": { "...": "..." },
  "maneuver": { "...": "..." },
  "spacecraft": {
    "coeff_drag": { "gaussian": { "sigma": 0.1 } },
    "drag_area_m2": { "gaussian": { "sigma": 0.005 } },
    "dry_mass_kg": { "gaussian": { "sigma": 0.5 } }
  }
}
```

#### Response sequence

```json
{"type": "progress", "request_id": 40, "phase": "mc", "detail": "Monte Carlo sample 10/100", "fraction": 0.1}
{"type": "heartbeat", "seq": 1}
{"type": "progress", "request_id": 40, "phase": "mc", "detail": "Monte Carlo sample 50/100", "fraction": 0.5}
{"type": "heartbeat", "seq": 2}
{"type": "progress", "request_id": 40, "phase": "mc", "detail": "Monte Carlo sample 100/100", "fraction": 1.0}
```

Final:

```json
{
  "type": "monte_carlo_result",
  "request_id": 40,
  "report": {
    "config": { "num_samples": 100, "...": "..." },
    "nominal_dv_km_s": 0.0034,
    "nominal_safety": { "...": "..." },
    "statistics": {
      "total_dv_km_s": {
        "min": 0.002,
        "p01": 0.002,
        "p05": 0.0025,
        "p25": 0.003,
        "p50": 0.0034,
        "p75": 0.004,
        "p95": 0.005,
        "p99": 0.006,
        "max": 0.007,
        "mean": 0.0035,
        "std_dev": 0.001
      },
      "min_3d_distance_km": { "...": "..." },
      "collision_probability": 0.0,
      "convergence_rate": 1.0,
      "ei_violation_rate": 0.15,
      "keepout_violation_rate": 0.0,
      "dispersion_envelope": []
    },
    "samples": [
      {
        "index": 0,
        "total_dv_km_s": 0.0033,
        "safety": {},
        "waypoint_miss_km": [0.001],
        "converged": true
      }
    ],
    "num_failures": 0,
    "elapsed_wall_s": 45.2,
    "covariance_cross_check": {
      "terminal_3sigma_containment": 0.997,
      "min_mahalanobis_distance": 5.2,
      "sigma_ratio_ric": [1.02, 0.98, 1.01]
    }
  }
}
```

---

## Scenario B: Proximity Operations

When the deputy is already close to the chief (~300 m), the flow is simpler: no Lambert transfer needed.

---

### Step 1: Set States (proximity)

```json
{
  "type": "set_states",
  "request_id": 1,
  "chief": {
    "epoch": "2024-01-01T00:00:00 UTC",
    "position_eci_km": [5876.261, 3392.661, 0.0],
    "velocity_eci_km_s": [-2.380512, 4.123167, 6.006917]
  },
  "deputy": {
    "epoch": "2024-01-01T00:00:00 UTC",
    "position_eci_km": [5876.168, 3392.822, 0.235],
    "velocity_eci_km_s": [-2.3806, 4.1231, 6.00685]
  }
}
```

### Step 2: Classify + Extract Drag (parallel)

Both only read `chief`/`deputy` from the session. Send in parallel:

```json
{
  "type": "classify",
  "request_id": 2
}
```

```json
{
  "type": "extract_drag",
  "request_id": 3
}
```

**classify_result** returns immediately with `phase.proximity` (includes ROE, separation, delta_r_over_r < 0.005):

```json
{
  "type": "classify_result",
  "request_id": 2,
  "phase": {
    "proximity": {
      "roe": {
        "da": 0.0,
        "dlambda": 0.00001,
        "dex": 0.0,
        "dey": 0.0,
        "dix": 0.0,
        "diy": 0.0
      },
      "chief_elements": { "...": "..." },
      "deputy_elements": { "...": "..." },
      "separation_km": 0.3,
      "delta_r_over_r": 0.00004
    }
  }
}
```

**drag_result** arrives ~3 seconds later.

### Step 3: Set Waypoints (auto-computes transfer for proximity)

For proximity, `set_waypoints` auto-computes the transfer internally. No explicit `compute_transfer` needed.

```json
{
  "type": "set_waypoints",
  "request_id": 4,
  "waypoints": [
    {
      "label": "Station-keep at 500m",
      "position_ric_km": [0.0, 0.5, 0.0],
      "tof_s": 2700.0
    },
    {
      "label": "Retreat to 2 km",
      "position_ric_km": [0.0, 2.0, 0.0],
      "tof_s": 2700.0
    }
  ]
}
```

**Response:** `plan_result` (same format as far-field, but `transfer_summary` is None).

### Step 4: On-Demand Fetches (parallel)

```json
{ "type": "get_trajectory", "request_id": 5, "max_points": 100 }
```

```json
{ "type": "get_eclipse", "request_id": 6 }
```

```json
{ "type": "get_free_drift_trajectory", "request_id": 8, "max_points": 100 }
```

```json
{
  "type": "run_cola",
  "request_id": 9,
  "config": { "target_distance_km": 0.3, "max_dv_km_s": 0.01 }
}
```

### Step 5: Validate

```json
{
  "type": "validate",
  "request_id": 7
}
```

---

## Session Management

### Get Session State

Inspect what's currently set in the session:

```json
{
  "type": "get_session",
  "request_id": 100
}
```

**Response:**

```json
{
  "type": "session_state",
  "request_id": 100,
  "summary": {
    "has_chief": true,
    "has_deputy": true,
    "has_transfer": true,
    "has_mission": true,
    "has_drag_config": true,
    "has_navigation_accuracy": false,
    "has_maneuver_uncertainty": false,
    "has_monte_carlo_config": false,
    "waypoint_count": 3,
    "chief_config": "servicer_500kg",
    "deputy_config": "cubesat_6u",
    "propagator": "j2",
    "lambert_tof_s": 3600.0
  }
}
```

### Reset Session

Clear all state back to defaults:

```json
{
  "type": "reset",
  "request_id": 101
}
```

**Response:**

```json
{
  "type": "state_updated",
  "request_id": 101,
  "updated": [],
  "invalidated": [
    "chief",
    "deputy",
    "transfer",
    "drag_config",
    "mission",
    "navigation_accuracy",
    "maneuver_uncertainty",
    "monte_carlo_config",
    "waypoints"
  ]
}
```

---

## Cancel a Background Job

Cancel a running `extract_drag`, `validate`, or `run_mc` operation.

### Send: cancel (cancel current job)

```json
{
  "type": "cancel",
  "request_id": null
}
```

### Send: cancel (cancel specific request)

```json
{
  "type": "cancel",
  "request_id": 40
}
```

### Response (if a job was active)

```json
{
  "type": "cancelled",
  "request_id": 40
}
```

If no job was active, no response is sent.

---

## Error Responses

All errors follow this format:

```json
{
  "type": "error",
  "request_id": 10,
  "code": "targeting_convergence",
  "message": "targeting did not converge after 100 iterations (error: 0.0015 km)",
  "detail": {
    "final_error_km": 0.0015,
    "iterations": 100
  }
}
```

### Error: classify before set_states

If you send `classify` without first calling `set_states`:

```json
{
  "type": "classify",
  "request_id": 1
}
```

Response:

```json
{
  "type": "error",
  "request_id": 1,
  "code": "missing_session_state",
  "message": "required session state not available: chief needed for set chief/deputy states via set_states before this operation",
  "detail": {
    "missing": "chief",
    "context": "set chief/deputy states via set_states before this operation"
  }
}
```

### Error: set_waypoints before compute_transfer (far-field)

If you set waypoints without computing a transfer first (far-field case):

```json
{
  "type": "error",
  "request_id": 10,
  "code": "missing_session_state",
  "message": "required session state not available: transfer needed for call compute_transfer before this operation",
  "detail": {
    "missing": "transfer",
    "context": "call compute_transfer before this operation"
  }
}
```

### Error codes

| Code                    | When                                                      | Tip                                                   |
| ----------------------- | --------------------------------------------------------- | ----------------------------------------------------- |
| `missing_session_state` | Required state not set (e.g., classify before set_states) | Follow the session flow order                         |
| `invalid_input`         | Malformed JSON, missing fields, bad values                | Check JSON syntax                                     |
| `targeting_convergence` | Newton-Raphson solver didn't converge                     | Try different `tof_s` or waypoint position            |
| `lambert_failure`       | Lambert solver failure                                    | Adjust `lambert_tof_s`, `direction`, or `revolutions` |
| `propagation_error`     | Propagation failure                                       | Check state vectors are valid                         |
| `validation_error`      | Nyx validation failure                                    | Ensure spacecraft configs are set                     |
| `monte_carlo_error`     | MC ensemble failure                                       | Check `monte_carlo` config                            |
| `nyx_bridge_error`      | Nyx bridge failure (almanac, dynamics)                    | Typically an internal issue                           |
| `covariance_error`      | Covariance propagation failure                            | Check `navigation_accuracy` values                    |
| `mission_error`         | General mission planning error                            | Check all mission fields                              |
| `cancelled`             | Operation was cancelled                                   | User or auto-cancel                                   |

---

## Quick Reference: Session Defaults

Fields that have defaults if never explicitly set:

| Session field                        | Default                                     | Notes                        |
| ------------------------------------ | ------------------------------------------- | ---------------------------- |
| `chief_config`                       | `cubesat_6u`                                | Preset: 12 kg, Cd=2.2        |
| `deputy_config`                      | `cubesat_6u`                                | Preset: 12 kg, Cd=2.2        |
| `perch`                              | `{ "v_bar": { "along_track_km": 5.0 } }`    | V-bar hold                   |
| `lambert_tof_s`                      | `3600.0`                                    | 1 hour                       |
| `lambert_config`                     | `{ "direction": "auto", "revolutions": 0 }` | Direct transfer              |
| `proximity`                          | `{ "roe_threshold": 0.005 }`                | Classification threshold     |
| `config`                             | `{}`                                        | All sub-fields have defaults |
| `config.targeting.max_iterations`    | `100`                                       |                              |
| `config.targeting.position_tol_km`   | `1e-6`                                      |                              |
| `config.targeting.trajectory_steps`  | `200`                                       | Points per leg               |
| `config.safety.min_ei_separation_km` | `0.2`                                       | Advisory threshold           |
| `config.safety.min_distance_3d_km`   | `0.1`                                       | Enforced keep-out            |
| `propagator`                         | `"j2"`                                      | J2-only analytical           |

---

## Typical Session Flows

### Far-Field

```
1.  Connect WebSocket to ws://127.0.0.1:3001/ws
2.  set_states                          -> state_updated (instant)
3.  set_spacecraft                      -> state_updated (instant, optional)
4.  classify                            -> classify_result (instant)
5.  compute_transfer                    -> transfer_computed (instant)
6.  extract_drag                        -> drag_result (~3s, async)
7.  set_waypoints                       -> plan_result (instant)
8.  get_trajectory / get_eclipse /       -> trajectory_data / eclipse_data /
    get_free_drift_trajectory /            free_drift_data / poca_data
    get_poca                               (parallel, on-demand)
9.  update_config(j2_drag)              -> plan_result (when drag_result arrives)
10. validate                            -> progress* + validation_result (seconds)
11. run_mc                              -> progress* + monte_carlo_result (minutes)
```

### Proximity

```
1.  Connect WebSocket to ws://127.0.0.1:3001/ws
2.  set_states                          -> state_updated (instant)
3.  classify + extract_drag (parallel)  -> classify_result + drag_result
4.  set_waypoints                       -> plan_result (auto-computes transfer)
5.  get_trajectory / get_eclipse /       -> trajectory_data / eclipse_data /
    get_free_drift_trajectory /            free_drift_data / poca_data
    get_poca                               (parallel, on-demand)
6.  validate                            -> progress* + validation_result
```
