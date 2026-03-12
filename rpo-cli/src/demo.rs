use std::error::Error;

use hifitime::Epoch;

use rpo_core::{
    // Elements layer
    compute_roe, keplerian_to_state, roe_to_ric, state_to_keplerian,
    // Propagation layer
    compute_j2_params, compute_stm, compute_stm_with_params, propagate_roe_j2_drag,
    propagate_roe_stm, PropagationModel,
    // Mission layer
    classify_separation, dimensionless_separation, plan_mission,
    solve_lambert_with_config, LambertConfig, TransferDirection,
    // Types
    DragConfig, KeplerianElements, MissionPhase, PerchGeometry,
    ProximityConfig, QuasiNonsingularROE,
};

use crate::{print_roe, print_trajectory_table};

// ---------------------------------------------------------------------------
// Orbit definitions — all distances in km, angles in radians
// ---------------------------------------------------------------------------

/// ISS-like chief orbit (near-circular LEO, 51.6° inclination).
fn iss_chief() -> KeplerianElements {
    KeplerianElements {
        a_km: 6786.0,           // ~408 km altitude
        e: 0.0001,              // near-circular
        i_rad: 51.6_f64.to_radians(),
        raan_rad: 30.0_f64.to_radians(),
        aop_rad: 0.0,
        mean_anomaly_rad: 0.0,
    }
}

/// Close deputy: 1 km higher, small phase offset — within ROE-valid range.
fn close_deputy() -> KeplerianElements {
    KeplerianElements {
        a_km: 6787.0,           // +1 km SMA → δa/a ≈ 1.5e-4
        e: 0.0001,
        i_rad: 51.6_f64.to_radians(),
        raan_rad: 30.0_f64.to_radians(),
        aop_rad: 0.0,
        mean_anomaly_rad: 0.01,  // small phase lead
    }
}

/// Eccentric orbit for demonstrating conversions on non-circular cases.
fn eccentric_orbit() -> KeplerianElements {
    KeplerianElements {
        a_km: 8000.0,           // higher altitude
        e: 0.15,                // moderate eccentricity (Molniya-like, but less extreme)
        i_rad: 63.4_f64.to_radians(), // critical inclination (ω̇ ≈ 0)
        raan_rad: 60.0_f64.to_radians(),
        aop_rad: 270.0_f64.to_radians(),
        mean_anomaly_rad: 45.0_f64.to_radians(),
    }
}

/// Distant deputy — well outside ROE-valid range, requires Lambert transfer.
/// Coplanar with chief, +500 km SMA.
fn far_deputy() -> KeplerianElements {
    KeplerianElements {
        a_km: 6786.0 + 500.0,  // +500 km SMA → δa/a ≈ 0.074
        e: 0.01,
        i_rad: 51.6_f64.to_radians(),  // same plane as chief
        raan_rad: 30.0_f64.to_radians(),
        aop_rad: 0.0,
        mean_anomaly_rad: 45.0_f64.to_radians(),
    }
}

// ---------------------------------------------------------------------------
// Demo (original hardcoded demo)
// ---------------------------------------------------------------------------

pub(crate) fn run_demo() -> Result<(), Box<dyn Error>> {
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
    let chief_sv = keplerian_to_state(&chief, epoch)?;
    println!("ISS-like orbit (near-circular, 51.6° inclination):");
    println!("  Keplerian: a={:.1} km, e={:.4}, i={:.1}°",
        chief.a_km, chief.e, chief.i_rad.to_degrees());
    println!("  ECI pos: [{:.3}, {:.3}, {:.3}] km",
        chief_sv.position_eci_km.x, chief_sv.position_eci_km.y, chief_sv.position_eci_km.z);
    println!("  ECI vel: [{:.6}, {:.6}, {:.6}] km/s",
        chief_sv.velocity_eci_km_s.x, chief_sv.velocity_eci_km_s.y, chief_sv.velocity_eci_km_s.z);

    // --- Roundtrip: ECI → Keplerian → ECI ---
    let recovered = state_to_keplerian(&chief_sv)?;
    let recovered_sv = keplerian_to_state(&recovered, epoch)?;
    let pos_err = (chief_sv.position_eci_km - recovered_sv.position_eci_km).norm();
    let vel_err = (chief_sv.velocity_eci_km_s - recovered_sv.velocity_eci_km_s).norm();
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
    let ecc_sv = keplerian_to_state(&ecc, epoch)?;
    let ecc_recovered = state_to_keplerian(&ecc_sv)?;
    let ecc_sv2 = keplerian_to_state(&ecc_recovered, epoch)?;
    let ecc_pos_err = (ecc_sv.position_eci_km - ecc_sv2.position_eci_km).norm();
    println!("\n  Eccentric orbit (e={:.2}, i={:.1}°, a={:.0} km):",
        ecc.e, ecc.i_rad.to_degrees(), ecc.a_km);
    println!("    True anomaly:  {:.4}°", ecc.true_anomaly().to_degrees());
    println!("    Roundtrip pos error: {ecc_pos_err:.2e} km");

    // ===================================================================
    // Section 2: ROE Computation
    // ===================================================================
    println!("\n╔══════════════════════════════════════════════════╗");
    println!("║  Section 2: Quasi-Nonsingular ROE Computation   ║");
    println!("╚══════════════════════════════════════════════════╝\n");

    let deputy = close_deputy();
    let roe = compute_roe(&chief, &deputy)?;

    print_roe("ROE (chief → close deputy)", &roe, chief.a_km);

    // --- to_vector / from_vector roundtrip ---
    let roe_vec = roe.to_vector();
    let roe_back = QuasiNonsingularROE::from_vector(&roe_vec);
    println!("\n  to_vector():  [{:.6e}, {:.6e}, {:.6e}, {:.6e}, {:.6e}, {:.6e}]",
        roe_vec[0], roe_vec[1], roe_vec[2], roe_vec[3], roe_vec[4], roe_vec[5]);
    println!("  from_vector() roundtrip δa error: {:.2e}", (roe_back.da - roe.da).abs());

    // --- Dimensionless separation metric ---
    let dim_sep = dimensionless_separation(&chief, &deputy)?;
    println!("\n  Dimensionless separation δr/r: {dim_sep:.6e}");
    println!("    (uses max of |δa|, |δex|, |δey|, |δix|; excludes δλ, δiy)");
    println!("    Default proximity threshold: {:.4}", ProximityConfig::default().roe_threshold);
    println!("    → {}", if dim_sep < ProximityConfig::default().roe_threshold {
        "Within ROE-valid range"
    } else {
        "Exceeds ROE threshold"
    });

    // --- Standalone roe_to_ric ---
    let ric = roe_to_ric(&roe, &chief)?;
    println!("\n  ROE → RIC mapping (D'Amico Eq. 2.17):");
    println!("    RIC frame: R=radial, I=in-track (velocity direction), C=cross-track");
    println!("    Position: R={:.6} km, I={:.6} km, C={:.6} km",
        ric.position_ric_km.x, ric.position_ric_km.y, ric.position_ric_km.z);
    println!("    Velocity: R={:.6e} km/s, I={:.6e} km/s, C={:.6e} km/s",
        ric.velocity_ric_km_s.x, ric.velocity_ric_km_s.y, ric.velocity_ric_km_s.z);

    // ===================================================================
    // Section 3: J2 Perturbation Parameters
    // ===================================================================
    println!("\n╔══════════════════════════════════════════════════╗");
    println!("║  Section 3: J2 Perturbation Parameters          ║");
    println!("╚══════════════════════════════════════════════════╝\n");

    let j2p = compute_j2_params(&chief)?;

    println!("J2 params for ISS-like chief (Koenig Eqs. 13-16):");
    println!("  Mean motion n:   {:.6e} rad/s", j2p.n_rad_s);
    println!("  κ (J2 freq):     {:.6e} rad/s", j2p.kappa);
    println!("  η (sqrt 1-e²)):  {:.10}", j2p.eta);
    println!("  p (semi-latus):  {:.4} km", j2p.p_km);

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
    let raan_deg_day = j2p.raan_dot_rad_s.to_degrees() * 86400.0;
    let aop_deg_day = j2p.aop_dot_rad_s.to_degrees() * 86400.0;
    println!("\n  Secular rates:");
    println!("    dΩ/dt = {:.6e} rad/s  ({raan_deg_day:+.3}°/day — RAAN regression)", j2p.raan_dot_rad_s);
    println!("    dω/dt = {:.6e} rad/s  ({aop_deg_day:+.3}°/day — apsidal advance)", j2p.aop_dot_rad_s);
    println!("    dM/dt = {:.6e} rad/s  (n + J2 correction)", j2p.m_dot_rad_s);

    // --- STM structure ---
    println!("\n  J2 STM at τ = 1 orbit ({:.1} s):", period_s);
    let stm = compute_stm(&chief, period_s)?;
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
    let (roe_1orbit, chief_1orbit) = propagate_roe_stm(&roe, &chief, period_s)?;
    println!("Standalone propagate_roe_stm() after 1 orbit:");
    print_roe("Propagated ROE", &roe_1orbit, chief.a_km);
    println!("    Chief RAAN drift: {:.6}°",
        (chief_1orbit.raan_rad - chief.raan_rad).to_degrees());

    // --- Full trajectory via J2StmPropagator trait ---
    let j2_prop = PropagationModel::J2Stm;
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
    println!("\n  Along-track drift from δa ≈ {:.4} km:", roe.da * chief.a_km);
    println!("    After {n_orbits} orbits: I = {:.6} km (secular growth from Φ[1,0]·δa)",
        final_j2.ric.position_ric_km.y);

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
    let drag_prop = PropagationModel::J2DragStm { drag };
    let drag_traj = drag_prop.propagate_with_steps(
        &roe, &chief, epoch, total_time, n_steps,
    )?;

    // --- Standalone propagate_roe_j2_drag ---
    let (roe_drag_1orbit, _) = propagate_roe_j2_drag(&roe, &chief, &drag, period_s)?;
    println!("\n  Standalone propagate_roe_j2_drag() after 1 orbit:");
    println!("    δa = {:.6e} (vs J2-only: {:.6e})", roe_drag_1orbit.da, roe_1orbit.da);

    // --- DragConfig::zero() equivalence ---
    let zero_drag_prop = PropagationModel::J2DragStm { drag: DragConfig::zero() };
    let zero_drag_final = zero_drag_prop.propagate(
        &roe, &chief, epoch, total_time,
    )?;
    let j2_only_final = &trajectory.last().unwrap();
    let zero_drag_diff = (zero_drag_final.ric.position_ric_km - j2_only_final.ric.position_ric_km).norm();
    println!("\n  DragConfig::zero() vs J2-only:");
    println!("    RIC position difference: {zero_drag_diff:.2e} km (should be ~0)");

    // --- Side-by-side comparison ---
    let drag_final = drag_traj.last().unwrap();
    println!("\n  Side-by-side final states after {n_orbits} orbits:");
    println!("    {:>12}  {:>14}  {:>14}  {:>14}", "", "J2-only", "J2+drag", "Delta");
    println!("    {:>12}  {:>14.6}  {:>14.6}  {:>14.6}", "R (km)",
        j2_only_final.ric.position_ric_km.x, drag_final.ric.position_ric_km.x,
        drag_final.ric.position_ric_km.x - j2_only_final.ric.position_ric_km.x);
    println!("    {:>12}  {:>14.6}  {:>14.6}  {:>14.6}", "I (km)",
        j2_only_final.ric.position_ric_km.y, drag_final.ric.position_ric_km.y,
        drag_final.ric.position_ric_km.y - j2_only_final.ric.position_ric_km.y);
    println!("    {:>12}  {:>14.6}  {:>14.6}  {:>14.6}", "C (km)",
        j2_only_final.ric.position_ric_km.z, drag_final.ric.position_ric_km.z,
        drag_final.ric.position_ric_km.z - j2_only_final.ric.position_ric_km.z);

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
    let deputy_sv = keplerian_to_state(&deputy_kep, epoch)?;

    // --- Proximity case ---
    println!("Case A: Close deputy (+1 km SMA, small phase offset)");
    let phase = classify_separation(&chief_sv, &deputy_sv, &config)?;
    match &phase {
        MissionPhase::Proximity { separation_km, delta_r_over_r, .. } => {
            println!("  Classification: PROXIMITY");
            println!("  ECI separation: {separation_km:.4} km");
            println!("  δr/r: {delta_r_over_r:.6e} (threshold: {})", config.roe_threshold);
        }
        MissionPhase::FarField { .. } => println!("  Classification: FAR-FIELD"),
    }

    let prox_roe = compute_roe(&chief, &deputy_kep)?;
    println!("\n  Proximity ROE state:");
    print_roe("ROE", &prox_roe, chief.a_km);

    // --- Far-field case ---
    let far_dep = far_deputy();
    let far_sv = keplerian_to_state(&far_dep, epoch)?;

    println!("\nCase B: Distant deputy (+500 km SMA, coplanar)");
    let far_phase = classify_separation(&chief_sv, &far_sv, &config)?;
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

    let mission = plan_mission(
        &chief_sv, &far_sv, &vbar_perch, &config, 3600.0, &LambertConfig::default(),
    )?;

    if let Some(ref transfer) = mission.transfer {
        println!("\n  Lambert transfer (deputy → V-bar perch):");
        println!("    Total Δv:     {:.4} km/s", transfer.total_dv_km_s);
        println!("    Departure Δv: {:.4} km/s", transfer.departure_dv_eci_km_s.norm());
        println!("    Arrival Δv:   {:.4} km/s", transfer.arrival_dv_eci_km_s.norm());
        println!("    TOF:          {:.1} s ({:.2} min)", transfer.tof_s, transfer.tof_s / 60.0);
        println!("    C3:           {:.4} km²/s²", transfer.c3_km2_s2);
        println!("    Direction:    {:?}", transfer.direction);
    }

    println!("\n  Perch orbit (V-bar, 5 km ahead):");
    print_roe("Perch ROE state", &mission.perch_roe, chief.a_km);

    // ===================================================================
    // Section 7: Advanced Lambert & R-Bar Perch
    // ===================================================================
    println!("\n╔══════════════════════════════════════════════════╗");
    println!("║  Section 7: Advanced Lambert & R-Bar Perch      ║");
    println!("╚══════════════════════════════════════════════════╝\n");

    // Build departure and arrival states for standalone Lambert calls
    let lambert_dep_ke = far_deputy();
    let lambert_dep_sv = keplerian_to_state(&lambert_dep_ke, epoch)?;
    // Target: a point near the chief 1 hour later
    let arrival_epoch = epoch + hifitime::Duration::from_seconds(3600.0);
    let lambert_arr_ke = KeplerianElements {
        a_km: chief.a_km + 5.0,  // slightly higher than chief (near perch)
        e: 0.001,
        i_rad: chief.i_rad,
        raan_rad: chief.raan_rad,
        aop_rad: 0.0,
        mean_anomaly_rad: 90.0_f64.to_radians(), // different position in orbit
    };
    let lambert_arr_sv = keplerian_to_state(&lambert_arr_ke, arrival_epoch)?;

    // --- Short-way transfer ---
    let short_config = LambertConfig {
        direction: TransferDirection::ShortWay,
        revolutions: 0,
    };
    let short_transfer = solve_lambert_with_config(
        &lambert_dep_sv, &lambert_arr_sv, &short_config,
    )?;
    println!("Short-way (prograde) transfer:");
    println!("  Δv: {:.4} km/s, TOF: {:.1} s", short_transfer.total_dv_km_s, short_transfer.tof_s);
    println!("  C3: {:.4} km²/s²", short_transfer.c3_km2_s2);

    // --- Long-way transfer ---
    let long_config = LambertConfig {
        direction: TransferDirection::LongWay,
        revolutions: 0,
    };
    let long_transfer = solve_lambert_with_config(
        &lambert_dep_sv, &lambert_arr_sv, &long_config,
    )?;
    println!("\nLong-way (retrograde) transfer:");
    println!("  Δv: {:.4} km/s, TOF: {:.1} s", long_transfer.total_dv_km_s, long_transfer.tof_s);
    println!("  C3: {:.4} km²/s²", long_transfer.c3_km2_s2);

    println!("\n  Short-way vs long-way Δv ratio: {:.2}x",
        long_transfer.total_dv_km_s / short_transfer.total_dv_km_s);

    // --- R-bar perch geometry ---
    println!("\nR-bar perch geometry (deputy 2 km above chief):");
    let rbar_perch = PerchGeometry::RBar { radial_km: 2.0 };

    let rbar_mission = plan_mission(
        &chief_sv, &far_sv, &rbar_perch, &config, 3600.0, &LambertConfig::default(),
    )?;

    println!("  R-bar perch ROE:");
    print_roe("R-bar state", &rbar_mission.perch_roe, chief.a_km);

    if let Some(ref transfer) = rbar_mission.transfer {
        println!("\n  Lambert to R-bar perch:");
        println!("    Total Δv: {:.4} km/s", transfer.total_dv_km_s);
    }

    // Compare V-bar vs R-bar perch
    let vbar_ric = roe_to_ric(&mission.perch_roe, &chief)?;
    let rbar_ric = roe_to_ric(&rbar_mission.perch_roe, &chief)?;
    println!("\n  Perch comparison (RIC at epoch):");
    println!("    {:>8}  {:>12}  {:>12}  {:>12}", "", "R (km)", "I (km)", "C (km)");
    println!("    {:>8}  {:>12.6}  {:>12.6}  {:>12.6}", "V-bar",
        vbar_ric.position_ric_km.x, vbar_ric.position_ric_km.y, vbar_ric.position_ric_km.z);
    println!("    {:>8}  {:>12.6}  {:>12.6}  {:>12.6}", "R-bar",
        rbar_ric.position_ric_km.x, rbar_ric.position_ric_km.y, rbar_ric.position_ric_km.z);

    // --- Custom perch geometry ---
    let custom_roe = QuasiNonsingularROE {
        da: 0.5 / chief.a_km,      // 0.5 km radial
        dlambda: 3.0 / chief.a_km,  // 3 km along-track
        dex: 0.0,
        dey: 0.0,
        dix: 0.0,
        diy: 0.0,
    };
    let custom_perch = PerchGeometry::Custom(custom_roe);
    let custom_ric = roe_to_ric(&custom_roe, &chief)?;
    println!("\n  Custom perch (0.5 km R + 3 km I):");
    println!("    RIC: [{:.4}, {:.4}, {:.4}] km",
        custom_ric.position_ric_km.x, custom_ric.position_ric_km.y, custom_ric.position_ric_km.z);

    let custom_mission = plan_mission(
        &chief_sv, &far_sv, &custom_perch, &config, 3600.0, &LambertConfig::default(),
    )?;
    if let Some(ref transfer) = custom_mission.transfer {
        println!("    Lambert Δv to custom perch: {:.4} km/s", transfer.total_dv_km_s);
    }

    // ===================================================================
    // Summary
    // ===================================================================
    println!("\n═══════════════════════════════════════════════════");
    println!("Demo complete. All 7 sections executed successfully.");
    println!("═══════════════════════════════════════════════════");

    Ok(())
}
