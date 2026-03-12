use std::error::Error;
use std::path::PathBuf;

use clap::{Parser, Subcommand};
use serde::Deserialize;

use rpo_core::{
    // Elements layer
    compute_roe, state_to_keplerian,
    // Propagation layer
    propagate_keplerian, PropagationModel,
    // Mission layer
    extract_dmf_rates, load_full_almanac, plan_mission, plan_waypoint_mission,
    validate_mission_nyx,
    // Types
    DepartureState, DragConfig, LambertConfig, MissionConfig, MissionPhase, MissionPlan,
    PerchGeometry, ProximityConfig, QuasiNonsingularROE, SafetyConfig, SafetyMetrics,
    SpacecraftConfig, StateVector, TargetingConfig, TofOptConfig, ValidationPoint,
    ValidationReport, Waypoint, WaypointMission,
};

mod demo;

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
    /// Waypoint-based maneuver targeting
    Target {
        /// Path to JSON input file with chief/deputy states + waypoints
        #[arg(short, long)]
        input: PathBuf,
        /// Output as JSON instead of human-readable text
        #[arg(long)]
        json: bool,
    },
    /// End-to-end far-field mission: Lambert transfer → perch → waypoint targeting
    Mission {
        /// Path to JSON input file with chief/deputy states, perch, and waypoints
        #[arg(short, long)]
        input: PathBuf,
        /// Output as JSON instead of human-readable text
        #[arg(long)]
        json: bool,
    },
    /// End-to-end mission with nyx high-fidelity validation (requires network on first run)
    Validate {
        /// Path to JSON input file with chief/deputy states, perch, waypoints, and spacecraft configs
        #[arg(short, long)]
        input: PathBuf,
        /// Output as JSON instead of human-readable text
        #[arg(long)]
        json: bool,
        /// Number of sample points per leg for validation comparison
        #[arg(long, default_value_t = 20)]
        samples_per_leg: u32,
        /// Auto-derive differential drag config from spacecraft properties via nyx
        #[arg(long)]
        auto_drag: bool,
    },
}

// ---------------------------------------------------------------------------
// Input schema (CLI-local)
// ---------------------------------------------------------------------------

#[derive(Deserialize, Default, Clone, Copy)]
#[serde(rename_all = "snake_case")]
enum PropagatorChoice {
    #[default]
    J2,
    J2Drag,
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct PropagatorConfig {
    propagator: PropagatorChoice,
    drag: Option<DragConfig>,
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct SolverConfig {
    targeting: Option<TargetingConfig>,
    tof_opt: Option<TofOptConfig>,
    safety: Option<SafetyConfig>,
}

impl PropagatorConfig {
    fn make_propagator(&self) -> PropagationModel {
        match (self.propagator, &self.drag) {
            (PropagatorChoice::J2Drag, Some(d)) => PropagationModel::J2DragStm { drag: *d },
            (PropagatorChoice::J2Drag, None) => {
                PropagationModel::J2DragStm { drag: DragConfig::zero() }
            }
            _ => PropagationModel::J2Stm,
        }
    }
}

impl SolverConfig {
    fn to_mission_config(&self) -> MissionConfig {
        MissionConfig {
            targeting: self.targeting.unwrap_or_default(),
            tof: self.tof_opt.unwrap_or_default(),
            safety: self.safety,
        }
    }
}

// ---------------------------------------------------------------------------
// Targeting input schema
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct TargetingInput {
    chief: StateVector,
    deputy: StateVector,
    waypoints: Vec<WaypointInput>,
    #[serde(flatten)]
    solver: SolverConfig,
    #[serde(flatten)]
    prop: PropagatorConfig,
}

#[derive(Deserialize)]
struct WaypointInput {
    position: [f64; 3],
    velocity: Option<[f64; 3]>,
    tof_s: Option<f64>,
    label: Option<String>,
}

// ---------------------------------------------------------------------------
// End-to-end mission input schema
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct EndToEndInput {
    chief: StateVector,
    deputy: StateVector,
    perch: Option<PerchGeometry>,
    lambert_tof_s: Option<f64>,
    lambert_config: Option<LambertConfig>,
    waypoints: Vec<WaypointInput>,
    proximity: Option<ProximityConfig>,
    #[serde(flatten)]
    solver: SolverConfig,
    #[serde(flatten)]
    prop: PropagatorConfig,
}

// ---------------------------------------------------------------------------
// Validate subcommand input/output schemas
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct ValidateInput {
    chief: StateVector,
    deputy: StateVector,
    perch: Option<PerchGeometry>,
    lambert_tof_s: Option<f64>,
    lambert_config: Option<LambertConfig>,
    waypoints: Vec<WaypointInput>,
    proximity: Option<ProximityConfig>,
    #[serde(flatten)]
    solver: SolverConfig,
    #[serde(flatten)]
    prop: PropagatorConfig,
    chief_config: Option<SpacecraftConfig>,
    deputy_config: Option<SpacecraftConfig>,
}

#[derive(serde::Serialize)]
struct ValidateOutput {
    transfer_phase: MissionPlan,
    waypoint_phase: WaypointMission,
    transfer_trajectory: Vec<StateVector>,
    chief_trajectory: Vec<StateVector>,
    lambert_dv_km_s: f64,
    waypoint_dv_km_s: f64,
    total_dv_km_s: f64,
    total_duration_s: f64,
    validation: ValidationReport,
    auto_drag_config: Option<DragConfig>,
}

/// Load and parse a JSON file into the given type.
fn load_json<T: serde::de::DeserializeOwned>(path: &PathBuf) -> Result<T, Box<dyn Error>> {
    let contents = std::fs::read_to_string(path)
        .map_err(|e| format!("Failed to read {}: {e}", path.display()))?;
    serde_json::from_str(&contents)
        .map_err(|e| -> Box<dyn Error> { format!("Failed to parse JSON: {e}").into() })
}

// ---------------------------------------------------------------------------
// Waypoint targeting from JSON input
// ---------------------------------------------------------------------------

fn run_targeting(input_path: &PathBuf, json_output: bool) -> Result<(), Box<dyn Error>> {
    let input: TargetingInput = load_json(input_path)?;

    let chief_ke = state_to_keplerian(&input.chief)?;
    let deputy_ke = state_to_keplerian(&input.deputy)?;
    let roe = compute_roe(&chief_ke, &deputy_ke)?;

    let waypoints = convert_waypoints(&input.waypoints);

    let mission_config = input.solver.to_mission_config();

    let departure = DepartureState {
        roe,
        chief: chief_ke,
        epoch: input.chief.epoch,
    };

    let prop = input.prop.make_propagator();
    let mission = plan_waypoint_mission(
        &departure, &waypoints, &mission_config, &prop,
    )?;

    if json_output {
        println!("{}", serde_json::to_string_pretty(&mission)?);
        return Ok(());
    }

    // --- Human-readable output ---
    println!("Waypoint Mission Plan");
    println!("=====================\n");

    println!("Initial ROE:");
    print_roe("  State", &roe, chief_ke.a_km);

    println!("\n{} waypoint(s), {} leg(s)", waypoints.len(), mission.legs.len());
    println!("Total Δv:       {:.6} km/s", mission.total_dv_km_s);
    println!("Total duration: {:.1} s ({:.2} min)\n", mission.total_duration_s, mission.total_duration_s / 60.0);

    println!(
        "  {:>4}  {:>10}  {:>12}  {:>12}  {:>12}  {:>12}",
        "Leg", "TOF (s)", "Δv1 (km/s)", "Δv2 (km/s)", "Total (km/s)", "Target I (km)"
    );
    println!("  {:-<4}  {:-<10}  {:-<12}  {:-<12}  {:-<12}  {:-<12}", "", "", "", "", "", "");
    for (i, leg) in mission.legs.iter().enumerate() {
        let dv1 = leg.departure_maneuver.dv_ric_km_s.norm();
        let dv2 = leg.arrival_maneuver.dv_ric_km_s.norm();
        let target_in = waypoints[i].position_ric_km.y;
        println!(
            "  {:>4}  {:>10.1}  {:>12.6}  {:>12.6}  {:>12.6}  {:>12.4}",
            i + 1,
            leg.tof_s,
            dv1,
            dv2,
            leg.total_dv_km_s,
            target_in,
        );
    }

    if let Some(ref safety) = mission.safety {
        let sc = mission_config.safety.unwrap_or_default();
        print_safety_analysis(safety, &sc);
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// End-to-end far-field mission
// ---------------------------------------------------------------------------

#[derive(serde::Serialize)]
struct EndToEndOutput {
    transfer_phase: MissionPlan,
    waypoint_phase: WaypointMission,
    transfer_trajectory: Vec<StateVector>,
    chief_trajectory: Vec<StateVector>,
    lambert_dv_km_s: f64,
    waypoint_dv_km_s: f64,
    total_dv_km_s: f64,
    total_duration_s: f64,
}

fn run_end_to_end_mission(input_path: &PathBuf, json_output: bool) -> Result<(), Box<dyn Error>> {
    let input: EndToEndInput = load_json(input_path)?;

    let chief_ke = state_to_keplerian(&input.chief)?;

    let perch = input.perch.unwrap_or(PerchGeometry::VBar { along_track_km: 5.0 });
    let proximity = input.proximity.unwrap_or_default();
    let lambert_tof_s = input.lambert_tof_s.unwrap_or(3600.0);
    let lambert_cfg = input.lambert_config.unwrap_or_default();

    // Phase 1: Classification + Lambert transfer
    let prop = input.prop.make_propagator();
    let mission = plan_mission(
        &input.chief, &input.deputy, &perch, &proximity, lambert_tof_s, &lambert_cfg,
    )?;

    let lambert_dv_km_s = mission.transfer.as_ref().map_or(0.0, |t| t.total_dv_km_s);

    // Phase 2: Waypoint targeting from perch
    let arrival_epoch = input.chief.epoch + hifitime::Duration::from_seconds(lambert_tof_s);

    let waypoints = convert_waypoints(&input.waypoints);

    let mission_config = input.solver.to_mission_config();

    let departure = DepartureState {
        roe: mission.perch_roe,
        chief: mission.chief_at_arrival,
        epoch: arrival_epoch,
    };

    let wp_mission = plan_waypoint_mission(
        &departure, &waypoints, &mission_config, &prop,
    )?;

    let waypoint_dv_km_s = wp_mission.total_dv_km_s;
    let total_dv_km_s = lambert_dv_km_s + waypoint_dv_km_s;
    let total_duration_s = lambert_tof_s + wp_mission.total_duration_s;

    if json_output {
        let n_arc_steps = 200;
        let (transfer_trajectory, chief_trajectory) = if let Some(ref transfer) = mission.transfer {
            (
                transfer.densify_arc(n_arc_steps)?,
                propagate_keplerian(&input.chief, transfer.tof_s, n_arc_steps)?,
            )
        } else {
            (vec![], vec![])
        };

        let output = EndToEndOutput {
            transfer_phase: mission,
            waypoint_phase: wp_mission,
            transfer_trajectory,
            chief_trajectory,
            lambert_dv_km_s,
            waypoint_dv_km_s,
            total_dv_km_s,
            total_duration_s,
        };
        println!("{}", serde_json::to_string_pretty(&output)?);
        return Ok(());
    }

    // --- Human-readable output ---
    println!("End-to-End Far-Field Mission");
    println!("============================\n");

    // Phase 1: Classification & Transfer
    println!("Phase 1: Classification & Transfer");
    println!("-----------------------------------");
    match &mission.phase {
        MissionPhase::Proximity { separation_km, delta_r_over_r, .. } => {
            println!("  Classification: PROXIMITY");
            println!("  ECI separation:  {separation_km:.4} km");
            println!("  δr/r:            {delta_r_over_r:.6e}");
        }
        MissionPhase::FarField { separation_km, delta_r_over_r, .. } => {
            println!("  Classification: FAR-FIELD (Lambert transfer required)");
            println!("  ECI separation:  {separation_km:.4} km");
            println!("  δr/r:            {delta_r_over_r:.6e}");
        }
    }

    if let Some(ref transfer) = mission.transfer {
        println!("\n  Lambert Transfer:");
        println!("    Total Δv:     {:.4} km/s", transfer.total_dv_km_s);
        println!("    Departure Δv: {:.4} km/s", transfer.departure_dv_eci_km_s.norm());
        println!("    Arrival Δv:   {:.4} km/s", transfer.arrival_dv_eci_km_s.norm());
        println!("    TOF:          {:.1} s ({:.2} min)", transfer.tof_s, transfer.tof_s / 60.0);
        println!("    C3:           {:.4} km²/s²", transfer.c3_km2_s2);
        println!("    Direction:    {:?}", transfer.direction);
        if lambert_cfg.revolutions > 0 {
            println!("    Revolutions:  {}", lambert_cfg.revolutions);
        }
        let v_circ = (rpo_core::constants::MU_EARTH / transfer.departure_state.position_eci_km.norm()).sqrt();
        println!("    Δv/v_circ:    {:.3}", transfer.total_dv_km_s / v_circ);
    }

    println!();
    print_roe("Perch ROE", &mission.perch_roe, chief_ke.a_km);

    // Phase 2: Waypoint Targeting
    println!("\n\nPhase 2: Waypoint Targeting ({} legs)", wp_mission.legs.len());
    println!("-----------------------------------");
    println!(
        "  {:>4}  {:>10}  {:>12}  {:>12}  {:>12}  Profile",
        "Leg", "TOF (s)", "Δv1 (km/s)", "Δv2 (km/s)", "Total (km/s)"
    );
    println!(
        "  {:-<4}  {:-<10}  {:-<12}  {:-<12}  {:-<12}  {:-<30}",
        "", "", "", "", "", ""
    );
    for (i, leg) in wp_mission.legs.iter().enumerate() {
        let dv1 = leg.departure_maneuver.dv_ric_km_s.norm();
        let dv2 = leg.arrival_maneuver.dv_ric_km_s.norm();
        let label = input.waypoints[i]
            .label
            .as_deref()
            .unwrap_or("-");
        println!(
            "  {:>4}  {:>10.1}  {:>12.6}  {:>12.6}  {:>12.6}  {}",
            i + 1,
            leg.tof_s,
            dv1,
            dv2,
            leg.total_dv_km_s,
            label,
        );
    }

    // Safety
    if let Some(ref safety) = wp_mission.safety {
        let sc = mission_config.safety.unwrap_or_default();
        print_safety_analysis(safety, &sc);
    }

    // Mission Summary
    println!("\nMission Summary:");
    println!("  Lambert transfer Δv:   {:.4} km/s", lambert_dv_km_s);
    println!("  Waypoint targeting Δv: {:.6} km/s", waypoint_dv_km_s);
    println!("  Total mission Δv:      {:.4} km/s", total_dv_km_s);
    println!("  Total mission time:    {:.1} s ({:.2} min)", total_duration_s, total_duration_s / 60.0);

    Ok(())
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Convert CLI waypoint inputs to domain waypoints.
fn convert_waypoints(inputs: &[WaypointInput]) -> Vec<Waypoint> {
    inputs
        .iter()
        .map(|wp| {
            let vel = wp.velocity.unwrap_or([0.0, 0.0, 0.0]);
            Waypoint {
                position_ric_km: nalgebra::Vector3::new(wp.position[0], wp.position[1], wp.position[2]),
                velocity_ric_km_s: nalgebra::Vector3::new(vel[0], vel[1], vel[2]),
                tof_s: wp.tof_s,
            }
        })
        .collect()
}

/// Print safety analysis results with pass/fail assessment.
fn print_safety_analysis(safety: &SafetyMetrics, config: &SafetyConfig) {
    let ei_ok = safety.min_ei_separation_km >= config.min_ei_separation_km;
    let d3_ok = safety.min_distance_3d_km >= config.min_distance_3d_km;
    let overall = ei_ok && d3_ok;
    println!("\nSafety Analysis:");
    println!(
        "  e/i separation constraint: {}",
        if ei_ok { "PASS" } else { "FAIL" }
    );
    println!(
        "  Min e/i separation:       {:.4} km  (threshold: {:.2} km)",
        safety.min_ei_separation_km, config.min_ei_separation_km
    );
    println!(
        "  3D distance constraint:   {}",
        if d3_ok { "PASS" } else { "FAIL" }
    );
    println!(
        "  Min 3D distance:          {:.4} km  (threshold: {:.2} km)",
        safety.min_distance_3d_km, config.min_distance_3d_km
    );
    println!(
        "    Leg: {}  Time: {:.1} s ({:.2} min)",
        safety.min_3d_leg_index + 1,
        safety.min_3d_elapsed_s,
        safety.min_3d_elapsed_s / 60.0,
    );
    println!(
        "    RIC: [{:.4}, {:.4}, {:.4}] km",
        safety.min_3d_ric_position_km.x,
        safety.min_3d_ric_position_km.y,
        safety.min_3d_ric_position_km.z,
    );
    println!(
        "  Overall:                  {}",
        if overall { "PASS" } else { "FAIL" }
    );
    println!(
        "  Min instantaneous R/C:    {:.4} km  (informational)",
        safety.min_rc_separation_km
    );
    println!(
        "    Leg: {}  Time: {:.1} s ({:.2} min)",
        safety.min_rc_leg_index + 1,
        safety.min_rc_elapsed_s,
        safety.min_rc_elapsed_s / 60.0,
    );
    println!(
        "    RIC: [{:.4}, {:.4}, {:.4}] km",
        safety.min_rc_ric_position_km.x,
        safety.min_rc_ric_position_km.y,
        safety.min_rc_ric_position_km.z,
    );
    println!("  δe magnitude:             {:.6e}", safety.de_magnitude);
    println!("  δi magnitude:             {:.6e}", safety.di_magnitude);
    println!(
        "  e/i phase angle:          {:.2}°",
        safety.ei_phase_angle_rad.to_degrees()
    );
}

/// Print per-leg position error breakdown (max, mean, RMS).
#[allow(clippy::cast_precision_loss)]
fn print_per_leg_errors(leg_points: &[Vec<ValidationPoint>]) {
    if leg_points.is_empty() {
        return;
    }
    println!("\n  Per-leg position error (km):");
    println!("    {:>4}  {:>10}  {:>10}  {:>10}", "Leg", "Max", "Mean", "RMS");
    println!("    {:->4}  {:->10}  {:->10}  {:->10}", "", "", "", "");
    for (i, points) in leg_points.iter().enumerate() {
        if points.is_empty() {
            continue;
        }
        let max = points.iter().fold(0.0_f64, |acc, p| acc.max(p.position_error_km));
        let sum: f64 = points.iter().map(|p| p.position_error_km).sum();
        let sum_sq: f64 = points.iter().map(|p| p.position_error_km.powi(2)).sum();
        let n = points.len() as f64;
        let mean = sum / n;
        let rms = (sum_sq / n).sqrt();
        println!("    {:>4}  {:>10.4}  {:>10.4}  {:>10.4}", i + 1, max, mean, rms);
    }
}

/// Print all 6 ROE components with physical meaning.
pub(crate) fn print_roe(label: &str, roe: &QuasiNonsingularROE, chief_a: f64) {
    println!("  {label}:");
    println!("    δa       = {:+.6e}  (relative SMA, ≈ {:.4} km)", roe.da, roe.da * chief_a);
    println!("    δλ       = {:+.6e}  (relative mean longitude)", roe.dlambda);
    println!("    δex      = {:+.6e}  (relative e·cos ω)", roe.dex);
    println!("    δey      = {:+.6e}  (relative e·sin ω)", roe.dey);
    println!("    δix      = {:+.6e}  (relative inclination)", roe.dix);
    println!("    δiy      = {:+.6e}  (relative RAAN·sin i)", roe.diy);
}

/// Print a trajectory table (RIC positions at sampled points).
pub(crate) fn print_trajectory_table(
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
                state.ric.position_ric_km.x,
                state.ric.position_ric_km.y,
                state.ric.position_ric_km.z,
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Validate: end-to-end mission + nyx high-fidelity validation
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_lines)]
fn run_validate(
    input_path: &PathBuf,
    json_output: bool,
    samples_per_leg: u32,
    auto_drag: bool,
) -> Result<(), Box<dyn Error>> {
    let input: ValidateInput = load_json(input_path)?;

    let chief_config = input.chief_config.unwrap_or_default();
    let deputy_config = input.deputy_config.unwrap_or_default();

    let chief_ke = state_to_keplerian(&input.chief)?;

    let perch = input.perch.unwrap_or(PerchGeometry::VBar { along_track_km: 5.0 });
    let proximity = input.proximity.unwrap_or_default();
    let lambert_tof_s = input.lambert_tof_s.unwrap_or(3600.0);
    let lambert_cfg = input.lambert_config.unwrap_or_default();

    // Load almanac (downloads on first run)
    eprintln!("Loading almanac (may download on first run)...");
    let almanac = load_full_almanac()
        .map_err(|e| format!("Almanac load failed: {e}"))?;

    // Phase 1: Classification + Lambert transfer
    eprintln!("Phase 1: Classification + Lambert transfer...");
    let mission = plan_mission(
        &input.chief, &input.deputy, &perch, &proximity, lambert_tof_s, &lambert_cfg,
    )?;

    let lambert_dv_km_s = mission.transfer.as_ref().map_or(0.0, |t| t.total_dv_km_s);

    // Compute post-Lambert states for validation and DMF extraction.
    // Far-field: propagate chief to arrival epoch, reconstruct deputy at perch.
    // Proximity: use original states (deputy already near chief).
    let arrival_epoch = input.chief.epoch + hifitime::Duration::from_seconds(lambert_tof_s);
    let (perch_chief, perch_deputy) = if let Some(ref transfer) = mission.transfer {
        let chief_traj = propagate_keplerian(&input.chief, lambert_tof_s, 1)?;
        let chief_at_arrival = chief_traj
            .last()
            .ok_or("Empty chief trajectory after Lambert propagation")?
            .clone();

        // Deputy at perch = Lambert arrival position + post-burn velocity.
        // arrival_state has Lambert arc velocity v2; arrival_dv = v_target - v2,
        // so v2 + arrival_dv = v_target (the actual deputy velocity at perch).
        let deputy_at_perch = StateVector {
            epoch: arrival_epoch,
            position_eci_km: transfer.arrival_state.position_eci_km,
            velocity_eci_km_s: transfer.arrival_state.velocity_eci_km_s
                + transfer.arrival_dv_eci_km_s,
        };

        (chief_at_arrival, deputy_at_perch)
    } else {
        (input.chief.clone(), input.deputy.clone())
    };

    // Optionally derive drag config from spacecraft properties
    let mut derived_drag: Option<DragConfig> = None;
    let prop = if auto_drag {
        eprintln!("Extracting differential drag rates via nyx...");
        let drag = extract_dmf_rates(
            &perch_chief, &perch_deputy,
            &chief_config, &deputy_config,
            &almanac,
        ).map_err(|e| format!("DMF extraction failed: {e}"))?;
        eprintln!(
            "  da_dot={:.6e}, dex_dot={:.6e}, dey_dot={:.6e}",
            drag.da_dot, drag.dex_dot, drag.dey_dot
        );
        derived_drag = Some(drag);
        PropagationModel::J2DragStm { drag }
    } else {
        input.prop.make_propagator()
    };

    // Phase 2: Waypoint targeting from perch
    eprintln!("Phase 2: Waypoint targeting...");
    let waypoints = convert_waypoints(&input.waypoints);
    let mission_config = input.solver.to_mission_config();

    let departure = DepartureState {
        roe: mission.perch_roe,
        chief: mission.chief_at_arrival,
        epoch: arrival_epoch,
    };

    let wp_mission = plan_waypoint_mission(
        &departure, &waypoints, &mission_config, &prop,
    )?;

    let waypoint_dv_km_s = wp_mission.total_dv_km_s;
    let total_dv_km_s = lambert_dv_km_s + waypoint_dv_km_s;
    let total_duration_s = lambert_tof_s + wp_mission.total_duration_s;

    // Phase 3: Nyx high-fidelity validation
    eprintln!("Phase 3: Nyx validation ({} samples/leg)...", samples_per_leg);
    let report = validate_mission_nyx(
        &wp_mission,
        &perch_chief,
        &perch_deputy,
        samples_per_leg,
        &chief_config,
        &deputy_config,
        &almanac,
    ).map_err(|e| format!("Nyx validation failed: {e}"))?;
    eprintln!("Validation complete.");

    if json_output {
        let n_arc_steps = 200;
        let (transfer_trajectory, chief_trajectory) = if let Some(ref transfer) = mission.transfer {
            (
                transfer.densify_arc(n_arc_steps)?,
                propagate_keplerian(&input.chief, transfer.tof_s, n_arc_steps)?,
            )
        } else {
            (vec![], vec![])
        };

        let output = ValidateOutput {
            transfer_phase: mission,
            waypoint_phase: wp_mission,
            transfer_trajectory,
            chief_trajectory,
            lambert_dv_km_s,
            waypoint_dv_km_s,
            total_dv_km_s,
            total_duration_s,
            validation: report,
            auto_drag_config: derived_drag,
        };
        println!("{}", serde_json::to_string_pretty(&output)?);
        return Ok(());
    }

    // --- Human-readable output ---
    println!("End-to-End Mission with Nyx Validation");
    println!("======================================\n");

    // Phase 1: Classification & Transfer
    println!("Phase 1: Classification & Transfer");
    println!("-----------------------------------");
    match &mission.phase {
        MissionPhase::Proximity { separation_km, delta_r_over_r, .. } => {
            println!("  Classification: PROXIMITY");
            println!("  ECI separation:  {separation_km:.4} km");
            println!("  δr/r:            {delta_r_over_r:.6e}");
        }
        MissionPhase::FarField { separation_km, delta_r_over_r, .. } => {
            println!("  Classification: FAR-FIELD (Lambert transfer required)");
            println!("  ECI separation:  {separation_km:.4} km");
            println!("  δr/r:            {delta_r_over_r:.6e}");
        }
    }

    if let Some(ref transfer) = mission.transfer {
        println!("\n  Lambert Transfer:");
        println!("    Total Δv:     {:.4} km/s", transfer.total_dv_km_s);
        println!("    Departure Δv: {:.4} km/s", transfer.departure_dv_eci_km_s.norm());
        println!("    Arrival Δv:   {:.4} km/s", transfer.arrival_dv_eci_km_s.norm());
        println!("    TOF:          {:.1} s ({:.2} min)", transfer.tof_s, transfer.tof_s / 60.0);
        println!("    C3:           {:.4} km²/s²", transfer.c3_km2_s2);
        println!("    Direction:    {:?}", transfer.direction);
        if lambert_cfg.revolutions > 0 {
            println!("    Revolutions:  {}", lambert_cfg.revolutions);
        }
        let v_circ = (rpo_core::constants::MU_EARTH / transfer.departure_state.position_eci_km.norm()).sqrt();
        println!("    Δv/v_circ:    {:.3}", transfer.total_dv_km_s / v_circ);
    }

    println!();
    print_roe("Perch ROE", &mission.perch_roe, chief_ke.a_km);

    // Phase 2: Waypoint Targeting
    println!("\n\nPhase 2: Waypoint Targeting ({} legs)", wp_mission.legs.len());
    println!("-----------------------------------");
    let prop_label = match &prop {
        PropagationModel::J2Stm => "J2 STM",
        PropagationModel::J2DragStm { .. } if auto_drag => "J2+Drag STM (auto-derived)",
        PropagationModel::J2DragStm { .. } => "J2+Drag STM (user-specified)",
    };
    println!("  Propagator: {prop_label}");
    println!(
        "  {:>4}  {:>10}  {:>12}  {:>12}  {:>12}  Profile",
        "Leg", "TOF (s)", "Δv1 (km/s)", "Δv2 (km/s)", "Total (km/s)"
    );
    println!(
        "  {:-<4}  {:-<10}  {:-<12}  {:-<12}  {:-<12}  {:-<30}",
        "", "", "", "", "", ""
    );
    for (i, leg) in wp_mission.legs.iter().enumerate() {
        let dv1 = leg.departure_maneuver.dv_ric_km_s.norm();
        let dv2 = leg.arrival_maneuver.dv_ric_km_s.norm();
        let label = input.waypoints[i]
            .label
            .as_deref()
            .unwrap_or("-");
        println!(
            "  {:>4}  {:>10.1}  {:>12.6}  {:>12.6}  {:>12.6}  {}",
            i + 1,
            leg.tof_s,
            dv1,
            dv2,
            leg.total_dv_km_s,
            label,
        );
    }

    // Safety
    if let Some(ref safety) = wp_mission.safety {
        let sc = mission_config.safety.unwrap_or_default();
        print_safety_analysis(safety, &sc);
    }

    // Phase 3: Validation
    println!("\n\nPhase 3: Nyx Validation ({} samples/leg)", samples_per_leg);
    println!("-----------------------------------");
    println!("  Position error (analytical vs nyx full-physics):");
    println!("    Max:  {:.6} km", report.max_position_error_km);
    println!("    Mean: {:.6} km", report.mean_position_error_km);
    println!("    RMS:  {:.6} km", report.rms_position_error_km);
    println!("    Max velocity error: {:.6e} km/s", report.max_velocity_error_km_s);

    print_per_leg_errors(&report.leg_points);

    println!("\n  Spacecraft configs:");
    println!(
        "    Chief:  {:.0} kg, drag area {:.2} m², Cd {:.1}",
        report.chief_config.dry_mass_kg,
        report.chief_config.drag_area_m2,
        report.chief_config.coeff_drag,
    );
    println!(
        "    Deputy: {:.0} kg, drag area {:.2} m², Cd {:.1}",
        report.deputy_config.dry_mass_kg,
        report.deputy_config.drag_area_m2,
        report.deputy_config.coeff_drag,
    );

    // Safety comparison (analytical vs numerical)
    print_safety_comparison(&report, &mission_config);
    println!(
        "  (analytical: {} steps/leg; numerical: {} nyx samples/leg)",
        mission_config.targeting.trajectory_steps, samples_per_leg,
    );

    if let Some(ref drag) = derived_drag {
        println!("\n  Auto-derived differential drag (nyx DMF):");
        println!("    da_dot:  {:+.6e}", drag.da_dot);
        println!("    dex_dot: {:+.6e}", drag.dex_dot);
        println!("    dey_dot: {:+.6e}", drag.dey_dot);
    }

    // Mission Summary
    println!("\nMission Summary:");
    println!("  Lambert transfer Δv:   {:.4} km/s", lambert_dv_km_s);
    println!("  Waypoint targeting Δv: {:.6} km/s", waypoint_dv_km_s);
    println!("  Total mission Δv:      {:.4} km/s", total_dv_km_s);
    println!("  Total mission time:    {:.1} s ({:.2} min)", total_duration_s, total_duration_s / 60.0);

    Ok(())
}

/// Print safety comparison table (analytical vs numerical from validation report).
fn print_safety_comparison(report: &ValidationReport, config: &MissionConfig) {
    let num = &report.numerical_safety;
    println!("\n  Safety Comparison (Analytical vs Numerical):");
    if let Some(ref ana) = report.analytical_safety {
        let sc = config.safety.unwrap_or_default();
        println!(
            "    {:>20}  {:>12}  {:>12}  {:>8}",
            "", "Analytical", "Numerical", "Thresh."
        );
        println!(
            "    {:>20}  {:>12.4}  {:>12.4}  {:>8}",
            "R/C sep (km)", ana.min_rc_separation_km, num.min_rc_separation_km, "N/A"
        );
        println!(
            "    {:>20}  {:>12.4}  {:>12.4}  {:>8.2}",
            "3D dist (km)", ana.min_distance_3d_km, num.min_distance_3d_km, sc.min_distance_3d_km
        );
        println!(
            "    {:>20}  {:>12.4}  {:>12.4}  {:>8.2}",
            "e/i sep (km)", ana.min_ei_separation_km, num.min_ei_separation_km, sc.min_ei_separation_km
        );
    } else {
        println!("    (no analytical safety available)");
        println!(
            "    {:>20}  {:>12.4}",
            "R/C sep (km)", num.min_rc_separation_km
        );
        println!(
            "    {:>20}  {:>12.4}",
            "3D dist (km)", num.min_distance_3d_km
        );
        println!(
            "    {:>20}  {:>12.4}",
            "e/i sep (km)", num.min_ei_separation_km
        );
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<(), Box<dyn Error>> {
    let cli = Cli::parse();

    match cli.command {
        None | Some(Command::Demo) => demo::run_demo(),
        Some(Command::Target { input, json }) => run_targeting(&input, json),
        Some(Command::Mission { input, json }) => run_end_to_end_mission(&input, json),
        Some(Command::Validate { input, json, samples_per_leg, auto_drag }) =>
            run_validate(&input, json, samples_per_leg, auto_drag),
    }
}
