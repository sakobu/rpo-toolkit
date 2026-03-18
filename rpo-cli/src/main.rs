use std::error::Error;
use std::path::PathBuf;

use clap::{Parser, Subcommand};
use serde::Deserialize;

use rpo_core::{
    // Elements layer
    state_to_keplerian,
    // Propagation layer
    propagate_keplerian, propagate_mission_covariance, ric_accuracy_to_roe_covariance,
    PropagationModel,
    // Mission layer
    assess_safety, compute_transfer_eclipse, extract_dmf_rates, load_full_almanac, plan_mission,
    plan_waypoint_mission, run_monte_carlo, validate_mission_nyx,
    // Types
    DepartureState, DragConfig, EclipseValidation, KeplerianElements, LambertConfig,
    ManeuverUncertainty, MissionConfig, MissionCovarianceReport, MissionEclipseData,
    MissionPhase, MissionPlan,
    MonteCarloConfig, MonteCarloInput, MonteCarloMode, MonteCarloReport, NavigationAccuracy,
    PercentileStats, PerchGeometry,
    ProximityConfig, QuasiNonsingularROE, RcContext, SafetyConfig, SafetyMetrics,
    SpacecraftConfig, StateVector, TransferEclipseData, ValidationPoint,
    ValidationReport, Waypoint, WaypointMission,
};

/// Default covariance sample count per leg used when the MC subcommand
/// runs ancillary covariance propagation.
const DEFAULT_COVARIANCE_SAMPLES: usize =
    rpo_core::constants::DEFAULT_COVARIANCE_SAMPLES_PER_LEG;

/// Position error threshold (km) below which the model is suitable for
/// close-proximity planning. Derived from typical J2 STM accuracy for
/// single-orbit LEO legs.
const FIDELITY_CLOSE_PROXIMITY_KM: f64 = 0.05;

/// Position error threshold (km) below which the model is suitable for
/// preliminary safety screening. Above this, only coarse mission planning.
const FIDELITY_SAFETY_SCREENING_KM: f64 = 0.2;

/// Ratio threshold for flagging non-conservative analytical predictions.
/// When numerical < analytical × this factor, the analytical model
/// overestimates the safety margin by >10%.
const NONCONSERVATIVE_RATIO: f64 = 0.9;

/// Δv percentage threshold above which Lambert transfer is annotated as
/// the dominant cost driver (far-field approach dominates the budget).
const FAR_FIELD_DV_DRIVER_PCT: f64 = 90.0;

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
    /// End-to-end mission: classification → Lambert transfer → perch → waypoint targeting
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
        #[arg(long, default_value_t = 50)]
        samples_per_leg: u32,
        /// Auto-derive differential drag config from spacecraft properties via nyx
        #[arg(long)]
        auto_drag: bool,
    },
    /// Full-physics Monte Carlo ensemble analysis (requires network on first run)
    Mc {
        /// Path to JSON input file
        #[arg(short, long)]
        input: PathBuf,
        /// Output as JSON instead of human-readable text
        #[arg(long)]
        json: bool,
        /// Auto-derive differential drag config from spacecraft properties via nyx
        #[arg(long)]
        auto_drag: bool,
    },
}

// ---------------------------------------------------------------------------
// Input schemas (CLI-local)
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

#[derive(Deserialize)]
struct WaypointInput {
    position: [f64; 3],
    velocity: Option<[f64; 3]>,
    tof_s: Option<f64>,
    label: Option<String>,
}

// ---------------------------------------------------------------------------
// Unified mission input schema (used by mission, validate, mc)
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct MissionInput {
    chief: StateVector,
    deputy: StateVector,
    #[serde(default)]
    perch: Option<PerchGeometry>,
    #[serde(default)]
    lambert_tof_s: Option<f64>,
    #[serde(default)]
    lambert_config: Option<LambertConfig>,
    waypoints: Vec<WaypointInput>,
    #[serde(default)]
    proximity: Option<ProximityConfig>,
    #[serde(flatten, default)]
    config: MissionConfig,
    #[serde(flatten)]
    prop: PropagatorConfig,
    #[serde(default)]
    chief_config: Option<SpacecraftConfig>,
    #[serde(default)]
    deputy_config: Option<SpacecraftConfig>,
    #[serde(default)]
    navigation_accuracy: Option<NavigationAccuracy>,
    #[serde(default)]
    maneuver_uncertainty: Option<ManeuverUncertainty>,
    #[serde(default)]
    monte_carlo: Option<MonteCarloConfig>,
}

// ---------------------------------------------------------------------------
// Unified mission output schema (used by mission, validate JSON output)
// ---------------------------------------------------------------------------

#[derive(serde::Serialize)]
struct MissionOutput<'a> {
    transfer_phase: &'a MissionPlan,
    waypoint_phase: &'a WaypointMission,
    transfer_trajectory: Vec<StateVector>,
    chief_trajectory: Vec<StateVector>,
    lambert_dv_km_s: f64,
    waypoint_dv_km_s: f64,
    total_dv_km_s: f64,
    total_duration_s: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    validation: Option<ValidationReport>,
    #[serde(skip_serializing_if = "Option::is_none")]
    auto_drag_config: Option<DragConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    covariance: Option<MissionCovarianceReport>,
    #[serde(skip_serializing_if = "Option::is_none")]
    monte_carlo: Option<MonteCarloReport>,
    #[serde(skip_serializing_if = "Option::is_none")]
    transfer_eclipse: Option<TransferEclipseData>,
}

// ---------------------------------------------------------------------------
// Shared mission planning pipeline
// ---------------------------------------------------------------------------

/// Result of classification + Lambert + perch state computation.
///
/// Computed before waypoint targeting so that callers can optionally
/// extract differential drag rates from the perch states.
struct TransferResult {
    plan: MissionPlan,
    chief_keplerian: KeplerianElements,
    perch_chief: StateVector,
    perch_deputy: StateVector,
    arrival_epoch: hifitime::Epoch,
    lambert_dv_km_s: f64,
    lambert_tof_s: f64,
}

/// A fully planned mission: transfer + waypoint targeting.
struct PlannedMission {
    transfer: TransferResult,
    wp_mission: WaypointMission,
    propagator: PropagationModel,
    mission_config: MissionConfig,
}

/// Resolve the propagation model, optionally extracting differential drag rates
/// from nyx full-physics dynamics via density-model-free (DMF) estimation.
///
/// When `auto_drag` is true, calls `extract_dmf_rates` and returns a
/// `J2DragStm` propagator with the derived drag config. Otherwise returns
/// the propagator from the input JSON.
fn resolve_propagator(
    auto_drag: bool,
    transfer: &TransferResult,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &std::sync::Arc<anise::prelude::Almanac>,
    default_prop: PropagationModel,
) -> Result<(PropagationModel, Option<DragConfig>), Box<dyn Error>> {
    if !auto_drag {
        return Ok((default_prop, None));
    }
    eprintln!("Extracting differential drag rates via nyx...");
    let drag = extract_dmf_rates(
        &transfer.perch_chief, &transfer.perch_deputy,
        chief_config, deputy_config,
        almanac,
    ).map_err(|e| format!("DMF extraction failed: {e}"))?;
    eprintln!(
        "  da_dot={:.6e}, dex_dot={:.6e}, dey_dot={:.6e}",
        drag.da_dot, drag.dex_dot, drag.dey_dot
    );
    Ok((PropagationModel::J2DragStm { drag }, Some(drag)))
}

/// Classify separation, solve Lambert if far-field, compute perch handoff states.
fn compute_transfer(input: &MissionInput) -> Result<TransferResult, Box<dyn Error>> {
    let chief_ke = state_to_keplerian(&input.chief)?;
    let perch = input.perch.clone().unwrap_or(PerchGeometry::VBar { along_track_km: 5.0 });
    let proximity = input.proximity.unwrap_or_default();
    let lambert_tof_s = input.lambert_tof_s.unwrap_or(3600.0);
    let lambert_cfg = input.lambert_config.clone().unwrap_or_default();

    let plan = plan_mission(
        &input.chief, &input.deputy, &perch, &proximity, lambert_tof_s, &lambert_cfg,
    )?;

    let lambert_dv_km_s = plan.transfer.as_ref().map_or(0.0, |t| t.total_dv_km_s);
    let arrival_epoch = input.chief.epoch + hifitime::Duration::from_seconds(lambert_tof_s);

    let (perch_chief, perch_deputy) = if let Some(ref transfer) = plan.transfer {
        let chief_traj = propagate_keplerian(&input.chief, lambert_tof_s, 1)?;
        let chief_at_arrival = chief_traj
            .last()
            .ok_or("Empty chief trajectory after Lambert propagation")?
            .clone();

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

    Ok(TransferResult {
        plan,
        chief_keplerian: chief_ke,
        perch_chief,
        perch_deputy,
        arrival_epoch,
        lambert_dv_km_s,
        lambert_tof_s,
    })
}

/// Plan waypoint mission given a transfer result and propagator.
fn plan_from_transfer(
    input: &MissionInput,
    transfer: TransferResult,
    propagator: PropagationModel,
) -> Result<PlannedMission, Box<dyn Error>> {
    let waypoints = convert_waypoints(&input.waypoints);
    let mission_config = input.config.clone();

    let departure = DepartureState {
        roe: transfer.plan.perch_roe,
        chief: transfer.plan.chief_at_arrival,
        epoch: transfer.arrival_epoch,
    };

    let wp_mission = plan_waypoint_mission(
        &departure, &waypoints, &mission_config, &propagator,
    )?;

    Ok(PlannedMission {
        transfer,
        wp_mission,
        propagator,
        mission_config,
    })
}

/// Build a `MissionOutput` for JSON serialization from a planned mission.
fn build_mission_output<'a>(planned: &'a PlannedMission, input: &MissionInput) -> Result<MissionOutput<'a>, Box<dyn Error>> {
    let (transfer_trajectory, chief_trajectory) = densify_transfer(&planned.transfer.plan, &input.chief)?;

    let waypoint_dv_km_s = planned.wp_mission.total_dv_km_s;
    let lambert_dv_km_s = planned.transfer.lambert_dv_km_s;
    let lambert_tof_s = planned.transfer.lambert_tof_s;

    Ok(MissionOutput {
        transfer_phase: &planned.transfer.plan,
        waypoint_phase: &planned.wp_mission,
        transfer_trajectory,
        chief_trajectory,
        lambert_dv_km_s,
        waypoint_dv_km_s,
        total_dv_km_s: lambert_dv_km_s + waypoint_dv_km_s,
        total_duration_s: lambert_tof_s + planned.wp_mission.total_duration_s,
        validation: None,
        auto_drag_config: None,
        covariance: None,
        monte_carlo: None,
        transfer_eclipse: None,
    })
}

/// Densify the Lambert transfer arc for visualization.
fn densify_transfer(
    plan: &MissionPlan,
    chief: &StateVector,
) -> Result<(Vec<StateVector>, Vec<StateVector>), Box<dyn Error>> {
    if let Some(ref transfer) = plan.transfer {
        let arc_steps = 200;
        Ok((
            transfer.densify_arc(arc_steps)?,
            propagate_keplerian(chief, transfer.tof_s, arc_steps)?,
        ))
    } else {
        Ok((vec![], vec![]))
    }
}

/// Load and parse a JSON file into the given type.
fn load_json<T: serde::de::DeserializeOwned>(path: &PathBuf) -> Result<T, Box<dyn Error>> {
    let contents = std::fs::read_to_string(path)
        .map_err(|e| format!("Failed to read {}: {e}", path.display()))?;
    serde_json::from_str(&contents)
        .map_err(|e| -> Box<dyn Error> { format!("Failed to parse JSON: {e}").into() })
}

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

// ---------------------------------------------------------------------------
// End-to-end mission
// ---------------------------------------------------------------------------

fn run_end_to_end_mission(input_path: &PathBuf, json_output: bool) -> Result<(), Box<dyn Error>> {
    let input: MissionInput = load_json(input_path)?;
    let transfer = compute_transfer(&input)?;
    let prop = input.prop.make_propagator();
    let planned = plan_from_transfer(&input, transfer, prop)?;

    let transfer_eclipse = planned.transfer.plan.transfer.as_ref().and_then(|t| {
        compute_transfer_eclipse(t, &input.chief, 200).ok()
    });

    if json_output {
        let mut output = build_mission_output(&planned, &input)?;
        output.transfer_eclipse = transfer_eclipse;
        println!("{}", serde_json::to_string_pretty(&output)?);
        return Ok(());
    }

    print_mission_human(&planned, &input, false, transfer_eclipse.as_ref());
    print_mission_verdict(&planned, None);
    Ok(())
}

// ---------------------------------------------------------------------------
// Validate: end-to-end mission + nyx high-fidelity validation
// ---------------------------------------------------------------------------

fn run_validate(
    input_path: &PathBuf,
    json_output: bool,
    samples_per_leg: u32,
    auto_drag: bool,
) -> Result<(), Box<dyn Error>> {
    let input: MissionInput = load_json(input_path)?;

    let chief_config = input.chief_config.unwrap_or_default();
    let deputy_config = input.deputy_config.unwrap_or_default();

    eprintln!("Loading almanac (may download on first run)...");
    let almanac = load_full_almanac()
        .map_err(|e| format!("Almanac load failed: {e}"))?;

    eprintln!("Phase 1: Classification + Lambert transfer...");
    let transfer = compute_transfer(&input)?;

    let (prop, derived_drag) = resolve_propagator(
        auto_drag, &transfer, &chief_config, &deputy_config,
        &almanac, input.prop.make_propagator(),
    )?;

    eprintln!("Phase 2: Waypoint targeting...");
    let planned = plan_from_transfer(&input, transfer, prop)?;

    // Phase 3: Nyx high-fidelity validation
    eprintln!("Phase 3: Nyx validation ({} samples/leg)...", samples_per_leg);
    let report = validate_mission_nyx(
        &planned.wp_mission,
        &planned.transfer.perch_chief,
        &planned.transfer.perch_deputy,
        samples_per_leg,
        &chief_config,
        &deputy_config,
        &almanac,
    ).map_err(|e| format!("Nyx validation failed: {e}"))?;
    eprintln!("Validation complete.");

    let transfer_eclipse = planned.transfer.plan.transfer.as_ref().and_then(|t| {
        compute_transfer_eclipse(t, &input.chief, 200).ok()
    });

    if json_output {
        let mut output = build_mission_output(&planned, &input)?;
        output.validation = Some(report);
        output.auto_drag_config = derived_drag;
        output.transfer_eclipse = transfer_eclipse;
        println!("{}", serde_json::to_string_pretty(&output)?);
        return Ok(());
    }

    // --- Human-readable output ---
    print_mission_human(&planned, &input, auto_drag, transfer_eclipse.as_ref());

    // Validation details
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

    print_safety_comparison(&report, &planned.mission_config);
    println!(
        "  (analytical: {} steps/leg; numerical: {} nyx samples/leg)",
        planned.mission_config.targeting.trajectory_steps, samples_per_leg,
    );

    if let Some(ref ev) = report.eclipse_validation {
        let max_eclipse_s = planned.wp_mission.eclipse.as_ref()
            .map_or(0.0, |e| e.summary.max_shadow_duration_s);
        print_eclipse_validation(ev, max_eclipse_s);
    }

    if let Some(ref drag) = derived_drag {
        print_derived_drag(drag);
    }

    print_mission_verdict(&planned, Some(&report));

    Ok(())
}

// ---------------------------------------------------------------------------
// Full-physics Monte Carlo from JSON input
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_lines)]
fn run_mc(input_path: &PathBuf, json_output: bool, auto_drag: bool) -> Result<(), Box<dyn Error>> {
    let input: MissionInput = load_json(input_path)?;

    let chief_config = input.chief_config
        .ok_or("mc subcommand requires 'chief_config' in input JSON")?;
    let deputy_config = input.deputy_config
        .ok_or("mc subcommand requires 'deputy_config' in input JSON")?;
    let mc_config = input.monte_carlo.as_ref()
        .ok_or("mc subcommand requires 'monte_carlo' in input JSON")?;

    eprintln!("Phase 1: Classification + Lambert transfer...");
    let transfer = compute_transfer(&input)?;

    // Load almanac (always needed for MC) + optional drag extraction
    eprintln!("Loading almanac (may download on first run)...");
    let almanac = load_full_almanac().map_err(|e| format!("Almanac load failed: {e}"))?;

    let (prop, derived_drag) = resolve_propagator(
        auto_drag, &transfer, &chief_config, &deputy_config,
        &almanac, input.prop.make_propagator(),
    )?;

    eprintln!("Phase 2: Waypoint targeting...");
    let planned = plan_from_transfer(&input, transfer, prop)?;

    // Phase 3: Optional covariance propagation
    let covariance_report = if let Some(ref nav) = input.navigation_accuracy {
        eprintln!("Running covariance propagation...");
        let initial_p = ric_accuracy_to_roe_covariance(nav, &planned.transfer.plan.chief_at_arrival)
            .map_err(|e| format!("Covariance initialization failed: {e}"))?;
        let report = propagate_mission_covariance(
            &planned.wp_mission,
            &initial_p,
            nav,
            input.maneuver_uncertainty.as_ref(),
            &planned.propagator,
            DEFAULT_COVARIANCE_SAMPLES,
        )
        .map_err(|e| format!("Covariance propagation failed: {e}"))?;
        Some(report)
    } else {
        None
    };

    // Phase 4: Run Monte Carlo
    eprintln!(
        "Phase 4: Running Monte Carlo ({} samples, {:?} mode)...",
        mc_config.num_samples, mc_config.mode,
    );

    let mc_input = MonteCarloInput {
        nominal_mission: &planned.wp_mission,
        initial_chief: &planned.transfer.perch_chief,
        initial_deputy: &planned.transfer.perch_deputy,
        config: mc_config,
        mission_config: &planned.mission_config,
        chief_config: &chief_config,
        deputy_config: &deputy_config,
        propagator: &planned.propagator,
        almanac: &almanac,
        covariance_report: covariance_report.as_ref(),
    };

    let report =
        run_monte_carlo(&mc_input).map_err(|e| format!("Monte Carlo failed: {e}"))?;

    eprintln!("Monte Carlo complete ({:.1}s wall time).", report.elapsed_wall_s);

    if json_output {
        println!("{}", serde_json::to_string_pretty(&report)?);
        return Ok(());
    }

    // --- Human-readable output ---
    print_mc_report(&report, planned.transfer.lambert_dv_km_s);

    if let Some(ref drag) = derived_drag {
        print_derived_drag(drag);
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Shared human-readable output helpers
// ---------------------------------------------------------------------------

/// Print the common Phase 1 + Phase 2 output shared by mission/validate handlers.
fn print_mission_human(
    planned: &PlannedMission,
    input: &MissionInput,
    auto_drag: bool,
    transfer_eclipse: Option<&TransferEclipseData>,
) {
    let transfer = &planned.transfer;
    let lambert_cfg = input.lambert_config.clone().unwrap_or_default();

    println!("End-to-End Mission");
    println!("==================\n");

    // Phase 1: Classification & Transfer
    println!("Phase 1: Classification & Transfer");
    println!("-----------------------------------");
    match &transfer.plan.phase {
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

    if let Some(ref lambert) = transfer.plan.transfer {
        println!("\n  Lambert Transfer:");
        println!("    Total Δv:     {:.4} km/s", lambert.total_dv_km_s);
        println!("    Departure Δv: {:.4} km/s", lambert.departure_dv_eci_km_s.norm());
        println!("    Arrival Δv:   {:.4} km/s", lambert.arrival_dv_eci_km_s.norm());
        println!("    TOF:          {:.1} s ({:.2} min)", lambert.tof_s, lambert.tof_s / 60.0);
        println!("    Direction:    {:?}", lambert.direction);
        if lambert_cfg.revolutions > 0 {
            println!("    Revolutions:  {}", lambert_cfg.revolutions);
        }
        let v_circ = (rpo_core::constants::MU_EARTH / lambert.departure_state.position_eci_km.norm()).sqrt();
        println!("    \u{0394}v/v_circ:    {:.1}%", lambert.total_dv_km_s / v_circ * 100.0);
    }

    if let Some(te) = transfer_eclipse {
        println!("\n  Transfer Eclipse:");
        println!("    Deputy shadow intervals: {}", te.summary.intervals.len());
        if te.summary.total_shadow_duration_s > 0.0 {
            println!(
                "    Deputy shadow time: {:.0} s ({:.1}% of transfer)",
                te.summary.total_shadow_duration_s,
                te.summary.time_in_shadow_fraction * 100.0,
            );
        } else {
            println!("    No shadow during transfer");
        }
    }

    println!();
    print_roe("Perch ROE", &transfer.plan.perch_roe, transfer.chief_keplerian.a_km);

    // Phase 2: Waypoint Targeting
    println!("\n\nPhase 2: Waypoint Targeting ({} legs)", planned.wp_mission.legs.len());
    println!("-----------------------------------");
    let prop_label = match &planned.propagator {
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
    for (i, leg) in planned.wp_mission.legs.iter().enumerate() {
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
    if let Some(ref safety) = planned.wp_mission.safety {
        let sc = planned.mission_config.safety.unwrap_or_default();
        print_safety_analysis(safety, &sc);
    }

    if let Some(ref eclipse) = planned.wp_mission.eclipse {
        print_eclipse_summary(eclipse);

        // Combined mission eclipse: transfer + waypoint phases
        if let Some(te) = transfer_eclipse {
            let total_shadow = te.summary.total_shadow_duration_s
                + eclipse.summary.total_shadow_duration_s;
            let total_duration = planned.transfer.lambert_tof_s
                + planned.wp_mission.total_duration_s;
            if total_duration > 0.0 {
                println!(
                    "  Combined (transfer + waypoint): {:.0} s ({:.1}% of full mission)",
                    total_shadow,
                    total_shadow / total_duration * 100.0,
                );
            }
        }
    }
}

/// Print the mission verdict with Δv breakdown, safety summary, and model confidence.
///
/// When a `ValidationReport` is available (validate subcommand), uses the numerical
/// safety values (more conservative) and includes model confidence assessment.
fn print_mission_verdict(
    planned: &PlannedMission,
    validation: Option<&ValidationReport>,
) {
    let lambert_dv_km_s = planned.transfer.lambert_dv_km_s;
    let waypoint_dv_km_s = planned.wp_mission.total_dv_km_s;
    let total_dv_km_s = lambert_dv_km_s + waypoint_dv_km_s;
    let lambert_tof_s = planned.transfer.plan.transfer.as_ref().map_or(0.0, |t| t.tof_s);
    let total_duration_s = lambert_tof_s + planned.wp_mission.total_duration_s;

    println!("\nMission Summary");
    println!("===============");

    // Verdict (use numerical safety when available, otherwise analytical)
    let sc = planned.mission_config.safety.unwrap_or_default();
    if let Some(report) = validation {
        let num_assessment = assess_safety(&report.numerical_safety, &sc);
        let verdict = if num_assessment.overall_pass {
            "FEASIBLE (safety metrics remain above threshold under nyx full-physics)"
        } else {
            "CAUTION (numerical safety below threshold)"
        };
        println!("  Verdict:               {verdict}");
    } else if let Some(ref safety) = planned.wp_mission.safety {
        let ana_assessment = assess_safety(safety, &sc);
        let verdict = if ana_assessment.overall_pass {
            "FEASIBLE (analytical safety above threshold)"
        } else {
            "CAUTION (analytical safety below threshold)"
        };
        println!("  Verdict:               {verdict}");
    }

    // Δv budget with interpretation
    println!("\n  \u{0394}v Budget:");
    if total_dv_km_s > 0.0 {
        let lambert_pct = lambert_dv_km_s / total_dv_km_s * 100.0;
        let wp_pct = waypoint_dv_km_s / total_dv_km_s * 100.0;
        let lambert_note = if lambert_pct > FAR_FIELD_DV_DRIVER_PCT { " -- far-field driver" } else { "" };
        println!(
            "    Lambert transfer:    {:.4} km/s  ({:.1}%{lambert_note})",
            lambert_dv_km_s, lambert_pct,
        );
        println!(
            "    Waypoint targeting:  {:.4} km/s  ({:.1}%)",
            waypoint_dv_km_s, wp_pct,
        );
    } else {
        println!("    Lambert transfer:    {:.4} km/s", lambert_dv_km_s);
        println!("    Waypoint targeting:  {:.6} km/s", waypoint_dv_km_s);
    }
    println!("    Total:               {:.4} km/s", total_dv_km_s);

    println!(
        "\n  Duration:              {:.1} min ({:.1} min transfer + {:.1} min proximity)",
        total_duration_s / 60.0,
        lambert_tof_s / 60.0,
        planned.wp_mission.total_duration_s / 60.0,
    );

    // Safety summary — use numerical values when available
    if let Some(report) = validation {
        print_safety_summary("nyx", &report.numerical_safety, &sc);
    } else if let Some(ref safety) = planned.wp_mission.safety {
        print_safety_summary("analytical", safety, &sc);
    }

    // Model confidence (only with validation data)
    if let Some(report) = validation {
        println!("\n  Model Fidelity:");
        println!(
            "    Position error: max {:.3} km, mean {:.3} km",
            report.max_position_error_km, report.mean_position_error_km,
        );
        if report.max_position_error_km < FIDELITY_CLOSE_PROXIMITY_KM {
            println!("    Suitable for:  interactive planning, preliminary safety screening, close proximity");
        } else if report.max_position_error_km < FIDELITY_SAFETY_SCREENING_KM {
            println!("    Suitable for:  interactive planning, preliminary safety screening");
        } else {
            println!("    Suitable for:  preliminary mission planning only");
        }
        println!("    Not for:       high-confidence operational truth analysis");
    }
}

/// Print compact safety metrics with threshold context.
fn print_safety_summary(label: &str, safety: &SafetyMetrics, config: &SafetyConfig) {
    println!("\n  Safety ({label}):");
    println!(
        "    Min 3D distance:     {:.3} km  (threshold: {:.2} km)",
        safety.operational.min_distance_3d_km, config.min_distance_3d_km,
    );
    println!(
        "    Abort safety (e/i):  {:.3} km  (threshold: {:.2} km)",
        safety.passive.min_ei_separation_km, config.min_ei_separation_km,
    );
}

/// Print auto-derived differential drag rates.
fn print_derived_drag(drag: &DragConfig) {
    println!("\nAuto-derived differential drag (nyx DMF):");
    println!("  da_dot:  {:+.6e}", drag.da_dot);
    println!("  dex_dot: {:+.6e}", drag.dex_dot);
    println!("  dey_dot: {:+.6e}", drag.dey_dot);
}

/// Print safety analysis results with pass/fail assessment.
fn print_safety_analysis(safety: &SafetyMetrics, config: &SafetyConfig) {
    let assessment = assess_safety(safety, config);

    println!("\nOperational Safety (analytical, sampled every ~21 s):");
    println!(
        "  3D distance constraint:   {}",
        if assessment.distance_3d_pass { "PASS" } else { "FAIL" }
    );
    println!(
        "  Min 3D distance:          {:.4} km  (threshold: {:.2} km)",
        safety.operational.min_distance_3d_km, config.min_distance_3d_km
    );
    println!(
        "    Leg: {}  Time: {:.1} s ({:.2} min, from WP start)",
        safety.operational.min_3d_leg_index + 1,
        safety.operational.min_3d_elapsed_s,
        safety.operational.min_3d_elapsed_s / 60.0,
    );
    println!(
        "    RIC: [{:.4}, {:.4}, {:.4}] km",
        safety.operational.min_3d_ric_position_km.x,
        safety.operational.min_3d_ric_position_km.y,
        safety.operational.min_3d_ric_position_km.z,
    );

    // R/C plane distance with geometry context
    match assessment.rc_context {
        RcContext::AlongTrackDominated { along_track_km } => {
            println!(
                "  Min R/C plane dist:       {:.4} km  (along-track dominated: {:.2} km along-track)",
                safety.operational.min_rc_separation_km, along_track_km,
            );
        }
        RcContext::RadialCrossTrack => {
            println!(
                "  Min R/C plane dist:       {:.4} km",
                safety.operational.min_rc_separation_km
            );
        }
    }
    println!(
        "    Leg: {}  Time: {:.1} s ({:.2} min, from WP start)",
        safety.operational.min_rc_leg_index + 1,
        safety.operational.min_rc_elapsed_s,
        safety.operational.min_rc_elapsed_s / 60.0,
    );
    println!(
        "    RIC: [{:.4}, {:.4}, {:.4}] km",
        safety.operational.min_rc_ric_position_km.x,
        safety.operational.min_rc_ric_position_km.y,
        safety.operational.min_rc_ric_position_km.z,
    );

    println!("\nAbort Safety (free-drift e/i bound):");
    println!(
        "  e/i separation constraint: {}",
        if assessment.ei_separation_pass { "PASS" } else { "FAIL" }
    );
    println!(
        "  Min e/i separation:       {:.4} km  (threshold: {:.2} km)",
        safety.passive.min_ei_separation_km, config.min_ei_separation_km
    );
    let margin = safety.passive.min_ei_separation_km - config.min_ei_separation_km;
    if margin > 0.0 {
        println!("  Margin:                   +{:.4} km above threshold", margin);
    }
    println!(
        "  e/i phase angle:          {:.2}\u{00b0}",
        safety.passive.ei_phase_angle_rad.to_degrees()
    );

    println!(
        "\n  Overall:                  {}",
        if assessment.overall_pass { "PASS" } else { "FAIL" }
    );
}

/// Print eclipse summary (intervals, durations, shadow fraction).
fn print_eclipse_summary(eclipse: &MissionEclipseData) {
    println!("\nEclipse Summary:");
    println!("  Shadow intervals:   {}", eclipse.summary.intervals.len());
    println!(
        "  Total shadow time:  {:.0} s ({:.1}% of waypoint phase)",
        eclipse.summary.total_shadow_duration_s,
        eclipse.summary.time_in_shadow_fraction * 100.0,
    );
    if eclipse.summary.max_shadow_duration_s > 0.0 {
        println!(
            "  Max single eclipse: {:.0} s ({:.1} min)",
            eclipse.summary.max_shadow_duration_s,
            eclipse.summary.max_shadow_duration_s / 60.0,
        );
    }
}

/// Print eclipse validation comparison (analytical vs ANISE).
fn print_eclipse_validation(ev: &EclipseValidation, max_eclipse_duration_s: f64) {
    println!("\n  Eclipse Validation (Analytical vs ANISE):");
    println!(
        "    Sun direction error:  max {:.4}\u{00b0}, mean {:.4}\u{00b0}",
        ev.max_sun_direction_error_rad.to_degrees(),
        ev.mean_sun_direction_error_rad.to_degrees(),
    );
    println!(
        "    Eclipse intervals:    {} analytical, {} numerical, {} matched",
        ev.analytical_interval_count,
        ev.numerical_interval_count,
        ev.matched_interval_count,
    );
    if !ev.interval_comparisons.is_empty() {
        println!(
            "    Timing error:         max {:.1} s, mean {:.1} s (entry/exit)",
            ev.max_timing_error_s,
            ev.mean_timing_error_s,
        );
        if max_eclipse_duration_s > 0.0 {
            let pct = ev.max_timing_error_s / max_eclipse_duration_s * 100.0;
            println!(
                "      ({:.1}% of max eclipse; adequate for planning, not power-critical analysis)",
                pct,
            );
        }
    }
    if ev.unmatched_interval_count > 0 {
        println!(
            "    Unmatched intervals:  {}",
            ev.unmatched_interval_count,
        );
    }
}

/// Print per-leg position error breakdown (max, mean, RMS) with optional target range context.
fn print_per_leg_errors(leg_points: &[Vec<ValidationPoint>]) {
    if leg_points.is_empty() {
        return;
    }
    println!("\n  Per-leg position error (km):");
    println!(
        "    {:>4}  {:>10}  {:>10}  {:>10}",
        "Leg", "Max", "Mean", "RMS"
    );
    println!(
        "    {:->4}  {:->10}  {:->10}  {:->10}",
        "", "", "", ""
    );
    for (i, points) in leg_points.iter().enumerate() {
        if points.is_empty() {
            continue;
        }
        let max = points.iter().fold(0.0_f64, |acc, p| acc.max(p.position_error_km));
        let sum: f64 = points.iter().map(|p| p.position_error_km).sum();
        let sum_sq: f64 = points.iter().map(|p| p.position_error_km.powi(2)).sum();
        let n = f64::from(u32::try_from(points.len()).unwrap_or(u32::MAX));
        let mean = sum / n;
        let rms = (sum_sq / n).sqrt();
        println!(
            "    {:>4}  {:>10.4}  {:>10.4}  {:>10.4}",
            i + 1, max, mean, rms
        );
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

/// Format a non-conservative direction flag for safety comparison.
fn nonconservative_flag(analytical: f64, numerical: f64) -> &'static str {
    // For minimum-distance metrics, numerical < analytical means the
    // analytical model overestimates the margin (non-conservative).
    if numerical < analytical * NONCONSERVATIVE_RATIO {
        "  *"
    } else {
        ""
    }
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
            "R/C plane (km)", ana.operational.min_rc_separation_km, num.operational.min_rc_separation_km, "N/A"
        );
        let d3_flag = nonconservative_flag(
            ana.operational.min_distance_3d_km,
            num.operational.min_distance_3d_km,
        );
        println!(
            "    {:>20}  {:>12.4}  {:>12.4}  {:>8.2}{}",
            "3D dist (km)", ana.operational.min_distance_3d_km, num.operational.min_distance_3d_km,
            sc.min_distance_3d_km, d3_flag,
        );
        let ei_flag = nonconservative_flag(
            ana.passive.min_ei_separation_km,
            num.passive.min_ei_separation_km,
        );
        println!(
            "    {:>20}  {:>12.4}  {:>12.4}  {:>8.2}{}",
            "e/i sep (km)", ana.passive.min_ei_separation_km, num.passive.min_ei_separation_km,
            sc.min_ei_separation_km, ei_flag,
        );
        if !d3_flag.is_empty() || !ei_flag.is_empty() {
            println!("  * = numerical margin is >10% smaller than analytical");
        }
    } else {
        println!("    (no analytical safety available)");
        println!(
            "    {:>20}  {:>12.4}",
            "R/C plane (km)", num.operational.min_rc_separation_km
        );
        println!(
            "    {:>20}  {:>12.4}",
            "3D dist (km)", num.operational.min_distance_3d_km
        );
        println!(
            "    {:>20}  {:>12.4}",
            "e/i sep (km)", num.passive.min_ei_separation_km
        );
    }
}

/// Print human-readable Monte Carlo ensemble report.
fn print_mc_report(report: &MonteCarloReport, lambert_dv_km_s: f64) {
    println!(
        "Full-Physics Monte Carlo Analysis ({:?})",
        report.config.mode
    );
    println!("===============================================\n");

    println!(
        "Config: {} samples, seed={}",
        report.config.num_samples,
        report.config.seed.unwrap_or(42),
    );
    if lambert_dv_km_s > 0.0 {
        let total = lambert_dv_km_s + report.nominal_dv_km_s;
        println!("Nominal \u{0394}v (waypoint):  {:.6} km/s", report.nominal_dv_km_s);
        println!("Lambert transfer \u{0394}v:    {:.4} km/s", lambert_dv_km_s);
        println!("Total mission \u{0394}v:       {:.4} km/s", total);
    } else {
        println!("Nominal \u{0394}v: {:.6} km/s", report.nominal_dv_km_s);
    }
    println!("Wall time:  {:.1}s\n", report.elapsed_wall_s);

    let stats = &report.statistics;

    // Delta-v distribution
    println!("\u{0394}v Distribution (km/s):");
    print_percentile_stats("  ", &stats.total_dv_km_s);

    let n = f64::from(report.config.num_samples);

    // Operational safety
    println!("\nOperational Safety:");
    println!(
        "  Collision Probability: {:.4} ({:.0} / {})",
        stats.collision_probability,
        (stats.collision_probability * n).round(),
        report.config.num_samples,
    );
    if stats.keepout_violation_rate > 0.0 {
        println!(
            "  Keep-out sphere violations: {:.1}% ({:.0} / {})",
            stats.keepout_violation_rate * 100.0,
            (stats.keepout_violation_rate * n).round(),
            report.config.num_samples,
        );
    } else {
        println!("  Keep-out sphere violations: 0.0% (0 / {})", report.config.num_samples);
    }
    println!("  Min R/C distance (km):");
    print_optional_percentile_summary("    ", stats.min_rc_distance_km.as_ref());
    println!("  Min 3D distance (km):");
    print_optional_percentile_summary("    ", stats.min_3d_distance_km.as_ref());

    // Passive / abort safety
    println!("\nPassive / Abort Safety:");
    if stats.ei_violation_rate > 0.0 {
        println!(
            "  e/i separation violations: {:.1}% ({:.0} / {})",
            stats.ei_violation_rate * 100.0,
            (stats.ei_violation_rate * n).round(),
            report.config.num_samples,
        );
    } else {
        println!("  e/i separation violations: 0.0% (0 / {})", report.config.num_samples);
    }
    println!("  Min e/i separation (km):");
    print_optional_percentile_summary("    ", stats.min_ei_separation_km.as_ref());
    if stats.ei_violation_rate > 0.0 {
        println!("  Note: e/i separation is an orbit-averaged passive safety metric (D'Amico Eq. 2.22)");
        println!("        derived for free-drift relative motion. In actively guided waypoint missions,");
        println!("        high violation rates can occur even when operational safety constraints remain");
        println!("        satisfied. Treat e/i here as an abort/contingency indicator unless explicitly");
        println!("        enforced during targeting.");
    }

    // Convergence
    println!(
        "\nConvergence Rate: {:.1}% ({:.0} / {})",
        stats.convergence_rate * 100.0,
        (stats.convergence_rate * n).round(),
        report.config.num_samples,
    );
    println!("Failures: {}", report.num_failures);

    // Per-waypoint miss distances
    if !stats.waypoint_miss_km.is_empty() {
        println!("\nPer-Waypoint Miss (km):");
        for (i, wp_stats) in stats.waypoint_miss_km.iter().enumerate() {
            if let Some(s) = wp_stats {
                println!(
                    "    WP{}:  p50: {:.4}    p95: {:.4}",
                    i + 1, s.p50, s.p95,
                );
            } else {
                println!("    WP{}:  N/A", i + 1);
            }
        }
    }

    // Covariance cross-check
    if let Some(ref cv) = report.covariance_cross_check {
        println!("\nCovariance Cross-Check:");
        if matches!(report.config.mode, MonteCarloMode::ClosedLoop) {
            println!(
                "  Terminal 3\u{03c3} box containment: {:.1}% of closed-loop MC samples",
                cv.terminal_3sigma_containment * 100.0,
            );
            println!("    (compared against open-loop covariance prediction)");
        } else {
            println!(
                "  Terminal 3\u{03c3} box containment: {:.1}% of samples",
                cv.terminal_3sigma_containment * 100.0,
            );
        }
        println!(
            "  Sigma ratio (R/I/C): {:.2} / {:.2} / {:.2}",
            cv.sigma_ratio_ric.x, cv.sigma_ratio_ric.y, cv.sigma_ratio_ric.z,
        );
        if matches!(report.config.mode, MonteCarloMode::ClosedLoop) {
            println!(
                "    (ClosedLoop: retargeting suppresses dispersion relative to open-loop covariance; values << 1 expected)"
            );
        }
        println!(
            "  Closest Mahalanobis approach over mission: {:.2}",
            cv.min_mahalanobis_distance,
        );
        println!(
            "    (Open-loop covariance diagnostic only; <1 means the nominal trajectory enters the 1\u{03c3} covariance envelope)"
        );
    }
}

/// Format a float for display, showing "N/A" for NaN (no-data sentinel).
fn fmt_or_na(v: f64, precision: usize) -> String {
    if v.is_nan() {
        "N/A".to_string()
    } else {
        format!("{v:.prec$}", prec = precision)
    }
}

/// Print full percentile statistics (mean, std, p05/p95, min/max).
fn print_percentile_stats(indent: &str, stats: &PercentileStats) {
    println!(
        "{indent}Mean:   {}    Std: {}",
        fmt_or_na(stats.mean, 6),
        fmt_or_na(stats.std_dev, 6),
    );
    println!(
        "{indent}p05:    {}    p95: {}",
        fmt_or_na(stats.p05, 6),
        fmt_or_na(stats.p95, 6),
    );
    println!(
        "{indent}Min:    {}    Max: {}",
        fmt_or_na(stats.min, 6),
        fmt_or_na(stats.max, 6),
    );
}

/// Print compact percentile summary (p05/p50/p95 one-liner).
fn print_percentile_summary(indent: &str, stats: &PercentileStats) {
    println!(
        "{indent}p05: {}    p50: {}    p95: {}",
        fmt_or_na(stats.p05, 4),
        fmt_or_na(stats.p50, 4),
        fmt_or_na(stats.p95, 4),
    );
}

/// Print compact percentile summary, or "N/A" if no data.
fn print_optional_percentile_summary(indent: &str, stats: Option<&PercentileStats>) {
    if let Some(s) = stats {
        print_percentile_summary(indent, s);
    } else {
        println!("{indent}N/A (no data)");
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<(), Box<dyn Error>> {
    let cli = Cli::parse();

    match cli.command {
        None => {
            Cli::parse_from(["rpo-cli", "--help"]);
            Ok(())
        }
        Some(Command::Mission { input, json }) => run_end_to_end_mission(&input, json),
        Some(Command::Validate {
            input,
            json,
            samples_per_leg,
            auto_drag,
        }) => run_validate(&input, json, samples_per_leg, auto_drag),
        Some(Command::Mc {
            input,
            json,
            auto_drag,
        }) => run_mc(&input, json, auto_drag),
    }
}
