use std::error::Error;
use std::path::PathBuf;

use clap::{Parser, Subcommand};
use hifitime::Epoch;
use serde::Deserialize;

use rpo_core::{
    // Elements layer
    compute_roe, keplerian_to_state, roe_to_ric, state_to_keplerian,
    // Propagation layer
    compute_j2_params, compute_stm, compute_stm_with_params, propagate_roe_j2_drag,
    propagate_roe_stm, J2DragStmPropagator, J2StmPropagator, RelativePropagator,
    // Mission layer
    classify_separation, dimensionless_separation, plan_mission, plan_proximity_mission,
    plan_waypoint_mission, solve_lambert_with_config, LambertConfig, TransferDirection,
    // Types
    DepartureState, DragConfig, KeplerianElements, MissionConfig, MissionPhase, MissionPlanConfig,
    PerchGeometry, ProximityConfig, QuasiNonsingularROE, SafetyConfig, StateVector,
    TargetingConfig, TofOptConfig, Waypoint,
};

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser)]
#[command(name = "rpo-cli", about = "RPO mission planning tool")]
struct Cli {
    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand)]
enum Command {
    /// Run the built-in API demo
    Demo,
    /// Plan a mission from user-provided ECI J2000 state vectors
    Run {
        /// Path to JSON input file
        #[arg(short, long)]
        input: PathBuf,
        /// Output as JSON instead of human-readable text
        #[arg(long)]
        json: bool,
    },
    /// Waypoint-based maneuver targeting
    Target {
        /// Path to JSON input file with chief/deputy states + waypoints
        #[arg(short, long)]
        input: PathBuf,
        /// Output as JSON instead of human-readable text
        #[arg(long)]
        json: bool,
    },
}

// ---------------------------------------------------------------------------
// Input schema (CLI-local)
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct MissionInput {
    chief: StateVector,
    deputy: StateVector,
    #[serde(default)]
    config: InputConfig,
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct InputConfig {
    perch: Option<PerchGeometry>,
    proximity: Option<ProximityConfig>,
    plan: Option<PlanOverrides>,
    propagator: PropagatorChoice,
    drag: Option<DragConfig>,
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct PlanOverrides {
    transfer_tof_s: Option<f64>,
    proximity_duration_s: Option<f64>,
    num_steps: Option<usize>,
}

#[derive(Deserialize, Default, Clone, Copy)]
#[serde(rename_all = "snake_case")]
enum PropagatorChoice {
    #[default]
    J2,
    J2Drag,
}

// ---------------------------------------------------------------------------
// Targeting input schema
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct TargetingInput {
    chief: StateVector,
    deputy: StateVector,
    waypoints: Vec<WaypointInput>,
    #[serde(default)]
    targeting: Option<TargetingConfig>,
    #[serde(default)]
    tof_opt: Option<TofOptConfig>,
    #[serde(default)]
    safety: Option<SafetyConfig>,
    #[serde(default)]
    propagator: PropagatorChoice,
    #[serde(default)]
    drag: Option<DragConfig>,
}

#[derive(Deserialize)]
struct WaypointInput {
    position: [f64; 3],
    #[serde(default)]
    velocity: Option<[f64; 3]>,
    tof_s: Option<f64>,
}

// ---------------------------------------------------------------------------
// Propagator dispatch
// ---------------------------------------------------------------------------

/// Load and parse a JSON file into the given type.
fn load_json<T: serde::de::DeserializeOwned>(path: &PathBuf) -> Result<T, Box<dyn Error>> {
    let contents = std::fs::read_to_string(path)
        .map_err(|e| format!("Failed to read {}: {e}", path.display()))?;
    serde_json::from_str(&contents)
        .map_err(|e| -> Box<dyn Error> { format!("Failed to parse JSON: {e}").into() })
}

fn make_propagator(
    choice: PropagatorChoice,
    drag: &Option<DragConfig>,
) -> Box<dyn RelativePropagator> {
    match (choice, drag) {
        (PropagatorChoice::J2Drag, Some(d)) => Box::new(J2DragStmPropagator { drag: *d }),
        (PropagatorChoice::J2Drag, None) => {
            Box::new(J2DragStmPropagator { drag: DragConfig::zero() })
        }
        _ => Box::new(J2StmPropagator),
    }
}

// ---------------------------------------------------------------------------
// Run mission from JSON input
// ---------------------------------------------------------------------------

fn run_mission(input_path: &PathBuf, json_output: bool) -> Result<(), Box<dyn Error>> {
    let input: MissionInput = load_json(input_path)?;

    let chief_ke = state_to_keplerian(&input.chief);
    let period_s = chief_ke.period();

    let perch = input.config.perch.unwrap_or(PerchGeometry::VBar { along_track_km: 5.0 });
    let proximity = input.config.proximity.unwrap_or_default();
    let overrides = input.config.plan.unwrap_or_default();

    let plan_config = MissionPlanConfig {
        transfer_tof_s: overrides.transfer_tof_s.unwrap_or(3600.0),
        proximity_duration_s: overrides.proximity_duration_s.unwrap_or(period_s * 2.0),
        num_steps: overrides.num_steps.unwrap_or(40),
    };

    let prop = make_propagator(input.config.propagator, &input.config.drag);
    let mission = plan_mission(
        &input.chief, &input.deputy, &perch, &proximity, prop.as_ref(), &plan_config,
    )?;

    if json_output {
        println!("{}", serde_json::to_string_pretty(&mission)?);
        return Ok(());
    }

    // --- Human-readable output ---
    println!("RPO Mission Plan");
    println!("================\n");

    // Phase classification
    match &mission.phase {
        MissionPhase::Proximity { separation_km, delta_r_over_r, roe, chief_elements, .. } => {
            println!("Phase: PROXIMITY");
            println!("  ECI separation:  {separation_km:.4} km");
            println!("  δr/r:            {delta_r_over_r:.6e}");
            println!();
            print_roe("Initial ROE", roe, chief_elements.a);
        }
        MissionPhase::FarField { separation_km, delta_r_over_r, .. } => {
            println!("Phase: FAR-FIELD (Lambert transfer required)");
            println!("  ECI separation:  {separation_km:.4} km");
            println!("  δr/r:            {delta_r_over_r:.6e}");
        }
    }

    // Lambert transfer
    if let Some(ref transfer) = mission.transfer {
        println!("\nLambert Transfer:");
        println!("  Total Δv:     {:.4} km/s", transfer.total_dv);
        println!("  Departure Δv: {:.4} km/s", transfer.departure_dv.norm());
        println!("  Arrival Δv:   {:.4} km/s", transfer.arrival_dv.norm());
        println!("  TOF:          {:.1} s ({:.2} min)", transfer.tof, transfer.tof / 60.0);
        if let Some(c3) = transfer.c3_km2_s2 {
            println!("  C3:           {:.4} km²/s²", c3);
        }
        println!("  Direction:    {:?}", transfer.direction);
    }

    // Perch ROE
    println!();
    print_roe("Perch ROE", &mission.perch_roe, chief_ke.a);

    // Trajectory table
    println!("\nProximity Trajectory ({} points):", mission.proximity_trajectory.len());
    let sample = if mission.proximity_trajectory.len() > 20 {
        mission.proximity_trajectory.len() / 20
    } else {
        1
    };
    print_trajectory_table(&mission.proximity_trajectory, sample);

    Ok(())
}

// ---------------------------------------------------------------------------
// Waypoint targeting from JSON input
// ---------------------------------------------------------------------------

fn run_targeting(input_path: &PathBuf, json_output: bool) -> Result<(), Box<dyn Error>> {
    let input: TargetingInput = load_json(input_path)?;

    let chief_ke = state_to_keplerian(&input.chief);
    let deputy_ke = state_to_keplerian(&input.deputy);
    let roe = compute_roe(&chief_ke, &deputy_ke);

    let waypoints: Vec<Waypoint> = input
        .waypoints
        .iter()
        .map(|wp| {
            let vel = wp.velocity.unwrap_or([0.0, 0.0, 0.0]);
            Waypoint {
                position: nalgebra::Vector3::new(wp.position[0], wp.position[1], wp.position[2]),
                velocity: nalgebra::Vector3::new(vel[0], vel[1], vel[2]),
                tof_s: wp.tof_s,
            }
        })
        .collect();

    let mission_config = MissionConfig {
        targeting: input.targeting.unwrap_or_default(),
        tof: input.tof_opt.unwrap_or_default(),
        safety: input.safety,
    };

    let departure = DepartureState {
        roe,
        chief: chief_ke,
        epoch: input.chief.epoch,
    };

    let prop = make_propagator(input.propagator, &input.drag);
    let mission = plan_waypoint_mission(
        &departure, &waypoints, &mission_config, prop.as_ref(),
    )?;

    if json_output {
        println!("{}", serde_json::to_string_pretty(&mission)?);
        return Ok(());
    }

    // --- Human-readable output ---
    println!("Waypoint Mission Plan");
    println!("=====================\n");

    println!("Initial ROE:");
    print_roe("  State", &roe, chief_ke.a);

    println!("\n{} waypoint(s), {} leg(s)", waypoints.len(), mission.legs.len());
    println!("Total Δv:       {:.6} km/s", mission.total_dv);
    println!("Total duration: {:.1} s ({:.2} min)\n", mission.total_duration_s, mission.total_duration_s / 60.0);

    println!(
        "  {:>4}  {:>10}  {:>12}  {:>12}  {:>12}  {:>12}",
        "Leg", "TOF (s)", "Δv1 (km/s)", "Δv2 (km/s)", "Total (km/s)", "Target I (km)"
    );
    println!("  {:-<4}  {:-<10}  {:-<12}  {:-<12}  {:-<12}  {:-<12}", "", "", "", "", "", "");
    for (i, leg) in mission.legs.iter().enumerate() {
        let dv1 = leg.departure_maneuver.dv.norm();
        let dv2 = leg.arrival_maneuver.dv.norm();
        let target_in = waypoints[i].position.y;
        println!(
            "  {:>4}  {:>10.1}  {:>12.6}  {:>12.6}  {:>12.6}  {:>12.4}",
            i + 1,
            leg.tof_s,
            dv1,
            dv2,
            leg.total_dv,
            target_in,
        );
    }

    if let Some(ref safety) = mission.safety {
        let sc = mission_config.safety.as_ref().map_or_else(SafetyConfig::default, |c| *c);
        let rc_ok = safety.min_rc_separation_km >= sc.min_rc_separation_km;
        let d3_ok = safety.min_distance_3d_km >= sc.min_distance_3d_km;
        let overall = rc_ok && d3_ok;
        println!("\nSafety Analysis:");
        println!("  R-C corridor constraint:  {}", if rc_ok { "PASS" } else { "FAIL" });
        println!("  Min R/C separation:       {:.4} km  (threshold: {:.2} km)", safety.min_rc_separation_km, sc.min_rc_separation_km);
        println!();
        println!("  3D distance constraint:   {}", if d3_ok { "PASS" } else { "FAIL" });
        println!("  Min 3D distance:          {:.4} km  (threshold: {:.2} km)", safety.min_distance_3d_km, sc.min_distance_3d_km);
        println!();
        println!("  Overall:                  {}", if overall { "PASS" } else { "FAIL" });
        println!("  δe magnitude:             {:.6e}", safety.de_magnitude);
        println!("  δi magnitude:             {:.6e}", safety.di_magnitude);
        println!("  e/i phase angle:          {:.2}°", safety.ei_phase_angle_rad.to_degrees());
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Orbit definitions — all distances in km, angles in radians
// ---------------------------------------------------------------------------

/// ISS-like chief orbit (near-circular LEO, 51.6° inclination).
fn iss_chief() -> KeplerianElements {
    KeplerianElements {
        a: 6786.0,           // ~408 km altitude
        e: 0.0001,           // near-circular
        i: 51.6_f64.to_radians(),
        raan: 30.0_f64.to_radians(),
        aop: 0.0,
        mean_anomaly: 0.0,
    }
}

/// Close deputy: 1 km higher, small phase offset — within ROE-valid range.
fn close_deputy() -> KeplerianElements {
    KeplerianElements {
        a: 6787.0,           // +1 km SMA → δa/a ≈ 1.5e-4
        e: 0.0001,
        i: 51.6_f64.to_radians(),
        raan: 30.0_f64.to_radians(),
        aop: 0.0,
        mean_anomaly: 0.01,  // small phase lead
    }
}

/// Eccentric orbit for demonstrating conversions on non-circular cases.
fn eccentric_orbit() -> KeplerianElements {
    KeplerianElements {
        a: 8000.0,           // higher altitude
        e: 0.15,             // moderate eccentricity (Molniya-like, but less extreme)
        i: 63.4_f64.to_radians(), // critical inclination (ω̇ ≈ 0)
        raan: 60.0_f64.to_radians(),
        aop: 270.0_f64.to_radians(),
        mean_anomaly: 45.0_f64.to_radians(),
    }
}

/// Distant deputy — well outside ROE-valid range, requires Lambert transfer.
fn far_deputy() -> KeplerianElements {
    KeplerianElements {
        a: 6786.0 + 500.0,  // +500 km SMA
        e: 0.01,
        i: 45.0_f64.to_radians(), // different plane
        raan: 90.0_f64.to_radians(),
        aop: 10.0_f64.to_radians(),
        mean_anomaly: 180.0_f64.to_radians(),
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Print all 6 ROE components with physical meaning.
fn print_roe(label: &str, roe: &QuasiNonsingularROE, chief_a: f64) {
    println!("  {label}:");
    println!("    δa       = {:+.6e}  (relative SMA, ≈ {:.4} km)", roe.da, roe.da * chief_a);
    println!("    δλ       = {:+.6e}  (relative mean longitude)", roe.dlambda);
    println!("    δex      = {:+.6e}  (relative e·cos ω)", roe.dex);
    println!("    δey      = {:+.6e}  (relative e·sin ω)", roe.dey);
    println!("    δix      = {:+.6e}  (relative inclination)", roe.dix);
    println!("    δiy      = {:+.6e}  (relative RAAN·sin i)", roe.diy);
}

/// Print a trajectory table (RIC positions at sampled points).
fn print_trajectory_table(
    trajectory: &[rpo_core::PropagatedState],
    sample_every: usize,
) {
    println!(
        "  {:>10}  {:>12}  {:>12}  {:>12}",
        "Time (s)", "R (km)", "I (km)", "C (km)"
    );
    println!("  {:-<10}  {:-<12}  {:-<12}  {:-<12}", "", "", "", "");
    for (k, state) in trajectory.iter().enumerate() {
        if k % sample_every == 0 || k == trajectory.len() - 1 {
            println!(
                "  {:10.1}  {:12.6}  {:12.6}  {:12.6}",
                state.elapsed_s,
                state.ric.position.x,
                state.ric.position.y,
                state.ric.position.z,
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Demo (original hardcoded demo)
// ---------------------------------------------------------------------------

fn run_demo() -> Result<(), Box<dyn Error>> {
    println!("RPO-RUST API Demo");
    println!("=================\n");

    let epoch = Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0);

    // ===================================================================
    // Section 1: Coordinate Conversions
    // ===================================================================
    println!("╔══════════════════════════════════════════════════╗");
    println!("║  Section 1: Coordinate Conversions              ║");
    println!("╚══════════════════════════════════════════════════╝\n");

    // --- ISS-like orbit: Keplerian → ECI ---
    let chief = iss_chief();
    let chief_sv = keplerian_to_state(&chief, epoch);
    println!("ISS-like orbit (near-circular, 51.6° inclination):");
    println!("  Keplerian: a={:.1} km, e={:.4}, i={:.1}°",
        chief.a, chief.e, chief.i.to_degrees());
    println!("  ECI pos: [{:.3}, {:.3}, {:.3}] km",
        chief_sv.position.x, chief_sv.position.y, chief_sv.position.z);
    println!("  ECI vel: [{:.6}, {:.6}, {:.6}] km/s",
        chief_sv.velocity.x, chief_sv.velocity.y, chief_sv.velocity.z);

    // --- Roundtrip: ECI → Keplerian → ECI ---
    let recovered = state_to_keplerian(&chief_sv);
    let recovered_sv = keplerian_to_state(&recovered, epoch);
    let pos_err = (chief_sv.position - recovered_sv.position).norm();
    let vel_err = (chief_sv.velocity - recovered_sv.velocity).norm();
    println!("\n  Roundtrip ECI→Kep→ECI errors:");
    println!("    Position: {pos_err:.2e} km");
    println!("    Velocity: {vel_err:.2e} km/s");

    // --- Derived quantities ---
    let nu = chief.true_anomaly();
    let n = chief.mean_motion();
    let period_s = std::f64::consts::TAU / n;
    println!("\n  Derived quantities:");
    println!("    True anomaly:  {:.4}° (from Kepler's equation)", nu.to_degrees());
    println!("    Mean motion:   {:.6e} rad/s", n);
    println!("    Orbital period: {:.1} s ({:.2} min)", period_s, period_s / 60.0);

    // --- Eccentric orbit ---
    let ecc = eccentric_orbit();
    let ecc_sv = keplerian_to_state(&ecc, epoch);
    let ecc_recovered = state_to_keplerian(&ecc_sv);
    let ecc_sv2 = keplerian_to_state(&ecc_recovered, epoch);
    let ecc_pos_err = (ecc_sv.position - ecc_sv2.position).norm();
    println!("\n  Eccentric orbit (e={:.2}, i={:.1}°, a={:.0} km):",
        ecc.e, ecc.i.to_degrees(), ecc.a);
    println!("    True anomaly:  {:.4}°", ecc.true_anomaly().to_degrees());
    println!("    Roundtrip pos error: {ecc_pos_err:.2e} km");

    // ===================================================================
    // Section 2: ROE Computation
    // ===================================================================
    println!("\n╔══════════════════════════════════════════════════╗");
    println!("║  Section 2: Quasi-Nonsingular ROE Computation   ║");
    println!("╚══════════════════════════════════════════════════╝\n");

    let deputy = close_deputy();
    let roe = compute_roe(&chief, &deputy);

    print_roe("ROE (chief → close deputy)", &roe, chief.a);

    // --- to_vector / from_vector roundtrip ---
    let roe_vec = roe.to_vector();
    let roe_back = QuasiNonsingularROE::from_vector(&roe_vec);
    println!("\n  to_vector():  [{:.6e}, {:.6e}, {:.6e}, {:.6e}, {:.6e}, {:.6e}]",
        roe_vec[0], roe_vec[1], roe_vec[2], roe_vec[3], roe_vec[4], roe_vec[5]);
    println!("  from_vector() roundtrip δa error: {:.2e}", (roe_back.da - roe.da).abs());

    // --- Dimensionless separation metric ---
    let dim_sep = dimensionless_separation(&chief, &deputy);
    println!("\n  Dimensionless separation δr/r: {dim_sep:.6e}");
    println!("    (uses max of |δa|, |δex|, |δey|, |δix|; excludes δλ, δiy)");
    println!("    Default proximity threshold: {:.4}", ProximityConfig::default().roe_threshold);
    println!("    → {}", if dim_sep < ProximityConfig::default().roe_threshold {
        "Within ROE-valid range"
    } else {
        "Exceeds ROE threshold"
    });

    // --- Standalone roe_to_ric ---
    let ric = roe_to_ric(&roe, &chief);
    println!("\n  ROE → RIC mapping (D'Amico Eq. 2.17):");
    println!("    RIC frame: R=radial, I=in-track (velocity direction), C=cross-track");
    println!("    Position: R={:.6} km, I={:.6} km, C={:.6} km",
        ric.position.x, ric.position.y, ric.position.z);
    println!("    Velocity: R={:.6e} km/s, I={:.6e} km/s, C={:.6e} km/s",
        ric.velocity.x, ric.velocity.y, ric.velocity.z);

    // ===================================================================
    // Section 3: J2 Perturbation Parameters
    // ===================================================================
    println!("\n╔══════════════════════════════════════════════════╗");
    println!("║  Section 3: J2 Perturbation Parameters          ║");
    println!("╚══════════════════════════════════════════════════╝\n");

    let j2p = compute_j2_params(&chief);

    println!("J2 params for ISS-like chief (Koenig Eqs. 13-16):");
    println!("  Mean motion n:   {:.6e} rad/s", j2p.n);
    println!("  κ (J2 freq):     {:.6e} rad/s", j2p.kappa);
    println!("  η (sqrt 1-e²)):  {:.10}", j2p.eta);
    println!("  p (semi-latus):  {:.4} km", j2p.p);

    println!("\n  Koenig auxiliaries (Eq. 14):");
    println!("    E = 1+η:       {:.10}", j2p.big_e);
    println!("    F = 4+3η:      {:.10}", j2p.big_f);
    println!("    G = 1/(η²E):   {:.10}", j2p.big_g);

    println!("\n  Angular auxiliaries (Eq. 15):");
    println!("    P = 3cos²i-1:  {:.10}", j2p.big_p);
    println!("    Q = 5cos²i-1:  {:.10}", j2p.big_q);
    println!("    R = cos i:     {:.10}", j2p.big_r);
    println!("    S = sin 2i:    {:.10}", j2p.big_s);
    println!("    T = sin²i:     {:.10}", j2p.big_t);

    // Secular rates in intuitive units
    let raan_deg_day = j2p.raan_dot.to_degrees() * 86400.0;
    let aop_deg_day = j2p.aop_dot.to_degrees() * 86400.0;
    println!("\n  Secular rates:");
    println!("    dΩ/dt = {:.6e} rad/s  ({raan_deg_day:+.3}°/day — RAAN regression)", j2p.raan_dot);
    println!("    dω/dt = {:.6e} rad/s  ({aop_deg_day:+.3}°/day — apsidal advance)", j2p.aop_dot);
    println!("    dM/dt = {:.6e} rad/s  (n + J2 correction)", j2p.m_dot);

    // --- STM structure ---
    println!("\n  J2 STM at τ = 1 orbit ({:.1} s):", period_s);
    let stm = compute_stm(&chief, period_s);
    println!("    Φ[0,0] = {:.6} (δa preserved — no secular drag)", stm[(0, 0)]);
    println!("    Φ[1,0] = {:.6} (along-track drift from δa)", stm[(1, 0)]);
    println!("    Φ[4,4] = {:.6} (δix preserved — no J2 coupling)", stm[(4, 4)]);
    println!("    Φ[5,0] = {:.6} (δiy drift from δa via J2)", stm[(5, 0)]);

    // --- compute_stm_with_params (reuse J2 params) ---
    let stm2 = compute_stm_with_params(&j2p, &chief, period_s);
    let stm_diff = (stm - stm2).norm();
    println!("    compute_stm_with_params() matches: diff = {stm_diff:.2e}");

    // ===================================================================
    // Section 4: J2 Propagation
    // ===================================================================
    println!("\n╔══════════════════════════════════════════════════╗");
    println!("║  Section 4: J2 Propagation (3 orbits)           ║");
    println!("╚══════════════════════════════════════════════════╝\n");

    let n_orbits = 3;
    let total_time = period_s * n_orbits as f64;
    let n_steps = 30 * n_orbits; // 30 steps per orbit

    // --- Standalone propagate_roe_stm ---
    let (roe_1orbit, chief_1orbit) = propagate_roe_stm(&roe, &chief, period_s);
    println!("Standalone propagate_roe_stm() after 1 orbit:");
    print_roe("Propagated ROE", &roe_1orbit, chief.a);
    println!("    Chief RAAN drift: {:.6}°",
        (chief_1orbit.raan - chief.raan).to_degrees());

    // --- Full trajectory via J2StmPropagator trait ---
    let j2_prop = J2StmPropagator;
    let trajectory = j2_prop.propagate_with_steps(
        &roe, &chief, epoch, total_time, n_steps,
    )?;

    println!("\nJ2StmPropagator trajectory ({n_orbits} orbits, {n_steps} steps):");
    print_trajectory_table(&trajectory, 15);

    // Show ROE evolution at orbit boundaries
    println!("\n  ROE at orbit boundaries:");
    let steps_per_orbit = n_steps / n_orbits;
    for orbit in 0..=n_orbits {
        let idx = (orbit * steps_per_orbit).min(trajectory.len() - 1);
        let s = &trajectory[idx];
        println!("    t={:.0}s (orbit {orbit}): δa={:.6e}, δλ={:.6e}, δex={:.6e}, δey={:.6e}",
            s.elapsed_s, s.roe.da, s.roe.dlambda, s.roe.dex, s.roe.dey);
    }

    // Along-track drift demonstration
    let final_j2 = trajectory.last().unwrap();
    println!("\n  Along-track drift from δa ≈ {:.4} km:", roe.da * chief.a);
    println!("    After {n_orbits} orbits: I = {:.6} km (secular growth from Φ[1,0]·δa)",
        final_j2.ric.position.y);

    // ===================================================================
    // Section 5: J2 + Drag Comparison
    // ===================================================================
    println!("\n╔══════════════════════════════════════════════════╗");
    println!("║  Section 5: J2 + Drag Comparison (Koenig VIII)  ║");
    println!("╚══════════════════════════════════════════════════╝\n");

    // Realistic differential drag for ISS-class objects (~1e-10 SMA decay per second)
    let drag = DragConfig {
        da_dot: -1e-10,   // SMA decay from differential drag
        dex_dot: 1e-11,   // eccentricity vector perturbation
        dey_dot: -1e-11,  // eccentricity vector perturbation
    };
    println!("Drag config (DMF — density-model-free):");
    println!("  δȧ_drag   = {:.2e} /s  (normalized SMA decay rate)", drag.da_dot);
    println!("  δėx_drag  = {:.2e} /s", drag.dex_dot);
    println!("  δėy_drag  = {:.2e} /s", drag.dey_dot);

    // --- J2+drag propagation ---
    let drag_prop = J2DragStmPropagator { drag };
    let drag_traj = drag_prop.propagate_with_steps(
        &roe, &chief, epoch, total_time, n_steps,
    )?;

    // --- Standalone propagate_roe_j2_drag ---
    let (roe_drag_1orbit, _) = propagate_roe_j2_drag(&roe, &chief, &drag, period_s);
    println!("\n  Standalone propagate_roe_j2_drag() after 1 orbit:");
    println!("    δa = {:.6e} (vs J2-only: {:.6e})", roe_drag_1orbit.da, roe_1orbit.da);

    // --- DragConfig::zero() equivalence ---
    let zero_drag_prop = J2DragStmPropagator { drag: DragConfig::zero() };
    let zero_drag_final = zero_drag_prop.propagate(
        &roe, &chief, epoch, total_time,
    );
    let j2_only_final = &trajectory.last().unwrap();
    let zero_drag_diff = (zero_drag_final.ric.position - j2_only_final.ric.position).norm();
    println!("\n  DragConfig::zero() vs J2-only:");
    println!("    RIC position difference: {zero_drag_diff:.2e} km (should be ~0)");

    // --- Side-by-side comparison ---
    let drag_final = drag_traj.last().unwrap();
    println!("\n  Side-by-side final states after {n_orbits} orbits:");
    println!("    {:>12}  {:>14}  {:>14}  {:>14}", "", "J2-only", "J2+drag", "Delta");
    println!("    {:>12}  {:>14.6}  {:>14.6}  {:>14.6}", "R (km)",
        j2_only_final.ric.position.x, drag_final.ric.position.x,
        drag_final.ric.position.x - j2_only_final.ric.position.x);
    println!("    {:>12}  {:>14.6}  {:>14.6}  {:>14.6}", "I (km)",
        j2_only_final.ric.position.y, drag_final.ric.position.y,
        drag_final.ric.position.y - j2_only_final.ric.position.y);
    println!("    {:>12}  {:>14.6}  {:>14.6}  {:>14.6}", "C (km)",
        j2_only_final.ric.position.z, drag_final.ric.position.z,
        drag_final.ric.position.z - j2_only_final.ric.position.z);

    println!("\n    {:>12}  {:>14}  {:>14}  {:>14}", "", "J2-only", "J2+drag", "Delta");
    println!("    {:>12}  {:>14.6e}  {:>14.6e}  {:>14.6e}", "δa",
        j2_only_final.roe.da, drag_final.roe.da,
        drag_final.roe.da - j2_only_final.roe.da);
    println!("    {:>12}  {:>14.6e}  {:>14.6e}  {:>14.6e}", "δλ",
        j2_only_final.roe.dlambda, drag_final.roe.dlambda,
        drag_final.roe.dlambda - j2_only_final.roe.dlambda);

    // ===================================================================
    // Section 6: Mission Classification & Planning
    // ===================================================================
    println!("\n╔══════════════════════════════════════════════════╗");
    println!("║  Section 6: Mission Classification & Planning   ║");
    println!("╚══════════════════════════════════════════════════╝\n");

    let config = ProximityConfig::default();
    let deputy_kep = close_deputy();
    let deputy_sv = keplerian_to_state(&deputy_kep, epoch);

    // --- Proximity case ---
    println!("Case A: Close deputy (+1 km SMA, small phase offset)");
    let phase = classify_separation(&chief_sv, &deputy_sv, &config);
    match &phase {
        MissionPhase::Proximity { separation_km, delta_r_over_r, .. } => {
            println!("  Classification: PROXIMITY");
            println!("  ECI separation: {separation_km:.4} km");
            println!("  δr/r: {delta_r_over_r:.6e} (threshold: {})", config.roe_threshold);
        }
        MissionPhase::FarField { .. } => println!("  Classification: FAR-FIELD"),
    }

    let prox_plan = plan_proximity_mission(
        &chief_sv, &deputy_sv, &config, &j2_prop,
        period_s * 2.0, // 2 orbits of proximity ops
        40,
    )?;

    println!("\n  Proximity mission plan:");
    print_roe("Perch ROE", &prox_plan.perch_roe, chief.a);
    println!("    Trajectory: {} points over {:.0} s",
        prox_plan.proximity_trajectory.len(),
        prox_plan.proximity_trajectory.last().unwrap().elapsed_s);

    // --- Far-field case ---
    let far_dep = far_deputy();
    let far_sv = keplerian_to_state(&far_dep, epoch);

    println!("\nCase B: Distant deputy (+500 km SMA, different plane)");
    let far_phase = classify_separation(&chief_sv, &far_sv, &config);
    match &far_phase {
        MissionPhase::Proximity { .. } => println!("  Classification: PROXIMITY"),
        MissionPhase::FarField { separation_km, delta_r_over_r, .. } => {
            println!("  Classification: FAR-FIELD (Lambert transfer required)");
            println!("  ECI separation: {separation_km:.4} km");
            println!("  δr/r: {delta_r_over_r:.6e} (exceeds threshold: {})", config.roe_threshold);
        }
    }

    // V-bar perch: deputy holds 5 km ahead along velocity vector
    let vbar_perch = PerchGeometry::VBar { along_track_km: 5.0 };
    let mission_config = MissionPlanConfig {
        transfer_tof_s: 3600.0,            // 1-hour Lambert transfer
        proximity_duration_s: period_s * 2.0, // 2 orbits proximity ops
        num_steps: 40,
    };

    let mission = plan_mission(
        &chief_sv, &far_sv, &vbar_perch, &config, &j2_prop, &mission_config,
    )?;

    if let Some(ref transfer) = mission.transfer {
        println!("\n  Lambert transfer (deputy → V-bar perch):");
        println!("    Total Δv:     {:.4} km/s", transfer.total_dv);
        println!("    Departure Δv: {:.4} km/s", transfer.departure_dv.norm());
        println!("    Arrival Δv:   {:.4} km/s", transfer.arrival_dv.norm());
        println!("    TOF:          {:.1} s ({:.2} min)", transfer.tof, transfer.tof / 60.0);
        if let Some(c3) = transfer.c3_km2_s2 {
            println!("    C3:           {:.4} km²/s²", c3);
        }
        println!("    Direction:    {:?}", transfer.direction);
    }

    println!("\n  Perch orbit (V-bar, 5 km ahead):");
    print_roe("Perch ROE state", &mission.perch_roe, chief.a);

    let prox_final = mission.proximity_trajectory.last().unwrap();
    println!("\n    Final proximity RIC: [{:.4}, {:.4}, {:.4}] km",
        prox_final.ric.position.x, prox_final.ric.position.y, prox_final.ric.position.z);

    // ===================================================================
    // Section 7: Advanced Lambert & R-Bar Perch
    // ===================================================================
    println!("\n╔══════════════════════════════════════════════════╗");
    println!("║  Section 7: Advanced Lambert & R-Bar Perch      ║");
    println!("╚══════════════════════════════════════════════════╝\n");

    // Build departure and arrival states for standalone Lambert calls
    let lambert_dep_ke = far_deputy();
    let lambert_dep_sv = keplerian_to_state(&lambert_dep_ke, epoch);
    // Target: a point near the chief 1 hour later
    let arrival_epoch = epoch + hifitime::Duration::from_seconds(3600.0);
    let lambert_arr_ke = KeplerianElements {
        a: chief.a + 5.0,  // slightly higher than chief (near perch)
        e: 0.001,
        i: chief.i,
        raan: chief.raan,
        aop: 0.0,
        mean_anomaly: 90.0_f64.to_radians(), // different position in orbit
    };
    let lambert_arr_sv = keplerian_to_state(&lambert_arr_ke, arrival_epoch);

    // --- Short-way transfer ---
    let short_config = LambertConfig {
        direction: TransferDirection::ShortWay,
        revolutions: 0,
    };
    let short_transfer = solve_lambert_with_config(
        &lambert_dep_sv, &lambert_arr_sv, &short_config,
    )?;
    println!("Short-way (prograde) transfer:");
    println!("  Δv: {:.4} km/s, TOF: {:.1} s", short_transfer.total_dv, short_transfer.tof);
    if let Some(c3) = short_transfer.c3_km2_s2 {
        println!("  C3: {:.4} km²/s²", c3);
    }

    // --- Long-way transfer ---
    let long_config = LambertConfig {
        direction: TransferDirection::LongWay,
        revolutions: 0,
    };
    let long_transfer = solve_lambert_with_config(
        &lambert_dep_sv, &lambert_arr_sv, &long_config,
    )?;
    println!("\nLong-way (retrograde) transfer:");
    println!("  Δv: {:.4} km/s, TOF: {:.1} s", long_transfer.total_dv, long_transfer.tof);
    if let Some(c3) = long_transfer.c3_km2_s2 {
        println!("  C3: {:.4} km²/s²", c3);
    }

    println!("\n  Short-way vs long-way Δv ratio: {:.2}x",
        long_transfer.total_dv / short_transfer.total_dv);

    // --- R-bar perch geometry ---
    println!("\nR-bar perch geometry (deputy 2 km above chief):");
    let rbar_perch = PerchGeometry::RBar { radial_km: 2.0 };
    let rbar_config = MissionPlanConfig {
        transfer_tof_s: 3600.0,
        proximity_duration_s: period_s * 2.0,
        num_steps: 40,
    };

    let rbar_mission = plan_mission(
        &chief_sv, &far_sv, &rbar_perch, &config, &j2_prop, &rbar_config,
    )?;

    println!("  R-bar perch ROE:");
    print_roe("R-bar state", &rbar_mission.perch_roe, chief.a);

    if let Some(ref transfer) = rbar_mission.transfer {
        println!("\n  Lambert to R-bar perch:");
        println!("    Total Δv: {:.4} km/s", transfer.total_dv);
    }

    // Compare V-bar vs R-bar perch
    let vbar_ric = roe_to_ric(&mission.perch_roe, &chief);
    let rbar_ric = roe_to_ric(&rbar_mission.perch_roe, &chief);
    println!("\n  Perch comparison (RIC at epoch):");
    println!("    {:>8}  {:>12}  {:>12}  {:>12}", "", "R (km)", "I (km)", "C (km)");
    println!("    {:>8}  {:>12.6}  {:>12.6}  {:>12.6}", "V-bar",
        vbar_ric.position.x, vbar_ric.position.y, vbar_ric.position.z);
    println!("    {:>8}  {:>12.6}  {:>12.6}  {:>12.6}", "R-bar",
        rbar_ric.position.x, rbar_ric.position.y, rbar_ric.position.z);

    // --- Custom perch geometry ---
    let custom_roe = QuasiNonsingularROE {
        da: 0.5 / chief.a,      // 0.5 km radial
        dlambda: 3.0 / chief.a,  // 3 km along-track
        dex: 0.0,
        dey: 0.0,
        dix: 0.0,
        diy: 0.0,
    };
    let custom_perch = PerchGeometry::Custom(custom_roe);
    let custom_ric = roe_to_ric(&custom_roe, &chief);
    println!("\n  Custom perch (0.5 km R + 3 km I):");
    println!("    RIC: [{:.4}, {:.4}, {:.4}] km",
        custom_ric.position.x, custom_ric.position.y, custom_ric.position.z);

    let custom_mission = plan_mission(
        &chief_sv, &far_sv, &custom_perch, &config, &j2_prop, &rbar_config,
    )?;
    if let Some(ref transfer) = custom_mission.transfer {
        println!("    Lambert Δv to custom perch: {:.4} km/s", transfer.total_dv);
    }

    // ===================================================================
    // Summary
    // ===================================================================
    println!("\n═══════════════════════════════════════════════════");
    println!("Demo complete. All 7 sections executed successfully.");
    println!("═══════════════════════════════════════════════════");

    Ok(())
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<(), Box<dyn Error>> {
    let cli = Cli::parse();

    match cli.command {
        None | Some(Command::Demo) => run_demo(),
        Some(Command::Run { input, json }) => run_mission(&input, json),
        Some(Command::Target { input, json }) => run_targeting(&input, json),
    }
}
