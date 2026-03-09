use roe_core::{
    classify_separation, compute_j2_params, compute_roe, eci_separation_km, keplerian_to_state,
    plan_mission, plan_proximity_mission, DragConfig, J2DragStmPropagator, J2StmPropagator,
    KeplerianElements, MissionPhase, MissionPlanConfig, PerchGeometry, ProximityConfig,
    RelativePropagator,
};

fn main() {
    println!("ROE-RUST RPO Mission Planner — Phase 3B");
    println!("=======================================\n");

    // Example: ISS-like chief orbit (mean elements)
    let epoch = hifitime::Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0);

    let chief_mean = KeplerianElements {
        a: 6786.0,
        e: 0.0001,
        i: 51.6_f64.to_radians(),
        raan: 30.0_f64.to_radians(),
        aop: 0.0,
        mean_anomaly: 0.0,
    };

    // Deputy: 1 km higher, small phase offset
    let deputy_mean = KeplerianElements {
        a: 6787.0,
        e: 0.0001,
        i: 51.6_f64.to_radians(),
        raan: 30.0_f64.to_radians(),
        aop: 0.0,
        mean_anomaly: 0.01,
    };

    // Step 1: ECI state vectors (from mean elements for visualization)
    let chief_sv = keplerian_to_state(&chief_mean, epoch);
    let deputy_sv = keplerian_to_state(&deputy_mean, epoch);
    println!("Chief ECI:  pos={:.3} km", chief_sv.position.transpose());
    println!("Deputy ECI: pos={:.3} km", deputy_sv.position.transpose());

    println!("\nChief mean SMA:  {:.4} km", chief_mean.a);
    println!("Deputy mean SMA: {:.4} km", deputy_mean.a);

    // === Phase 3B: Mission Classification ===
    println!("\n\n=== Mission Phase Classification ===\n");

    let config = ProximityConfig::default();
    let sep_km = eci_separation_km(&chief_sv, &deputy_sv);
    println!("ECI separation: {sep_km:.4} km");
    println!("ROE threshold:  {:.4} (dimensionless)", config.roe_threshold);

    let phase = classify_separation(&chief_sv, &deputy_sv, &config);
    match &phase {
        MissionPhase::Proximity {
            delta_r_over_r,
            separation_km,
            ..
        } => {
            println!("Classification: PROXIMITY");
            println!("  delta_r/r: {delta_r_over_r:.6e}");
            println!("  separation: {separation_km:.4} km");
        }
        MissionPhase::FarField {
            delta_r_over_r,
            separation_km,
            ..
        } => {
            println!("Classification: FAR-FIELD");
            println!("  delta_r/r: {delta_r_over_r:.6e}");
            println!("  separation: {separation_km:.4} km");
        }
    }

    // === Proximity Mission Planning ===
    println!("\n\n=== Proximity Mission Plan ===\n");

    let propagator = J2StmPropagator;
    let j2p = compute_j2_params(&chief_mean);
    let period = std::f64::consts::TAU / j2p.n;
    let n_orbits = 3;

    let plan = plan_proximity_mission(
        &chief_sv,
        &deputy_sv,
        &config,
        &propagator,
        period * n_orbits as f64,
        30 * n_orbits,
    )
    .expect("proximity mission failed");

    println!(
        "Perch ROE: da={:.6e}, dlambda={:.6e}",
        plan.perch_roe.da, plan.perch_roe.dlambda
    );
    println!("Trajectory points: {}", plan.proximity_trajectory.len());

    println!(
        "\n{:>10}  {:>12}  {:>12}  {:>12}",
        "Time (s)", "R (km)", "I (km)", "C (km)"
    );

    let traj = &plan.proximity_trajectory;
    for (k, state) in traj.iter().enumerate() {
        if k % 10 == 0 || k == traj.len() - 1 {
            println!(
                "{:10.1}  {:12.4}  {:12.4}  {:12.4}",
                state.elapsed_s,
                state.ric.position.x,
                state.ric.position.y,
                state.ric.position.z,
            );
        }
    }

    let final_state = traj.last().unwrap();
    println!(
        "\nFinal RIC position: [{:.4}, {:.4}, {:.4}] km",
        final_state.ric.position.x,
        final_state.ric.position.y,
        final_state.ric.position.z
    );

    // === Far-field scenario ===
    println!("\n\n=== Far-Field Classification Demo ===\n");

    let distant_deputy = KeplerianElements {
        a: chief_mean.a + 500.0,
        e: 0.01,
        i: 45.0_f64.to_radians(),
        raan: 90.0_f64.to_radians(),
        aop: 10.0_f64.to_radians(),
        mean_anomaly: 180.0_f64.to_radians(),
    };
    let distant_sv = keplerian_to_state(&distant_deputy, epoch);

    let far_phase = classify_separation(&chief_sv, &distant_sv, &config);
    match &far_phase {
        MissionPhase::Proximity { .. } => println!("Classification: PROXIMITY"),
        MissionPhase::FarField {
            delta_r_over_r,
            separation_km,
            ..
        } => {
            println!("Classification: FAR-FIELD (Lambert transfer needed)");
            println!("  delta_r/r: {delta_r_over_r:.6e}");
            println!("  separation: {separation_km:.4} km");
        }
    }

    // === Lambert transfer demo ===
    println!("\n\n=== Lambert Transfer + Perch Orbit Handoff ===\n");

    let perch = PerchGeometry::VBar {
        along_track_km: 5.0,
    };
    let mission_plan_config = MissionPlanConfig {
        transfer_tof_s: 3600.0, // 1 hour transfer
        proximity_duration_s: period * 2.0, // 2 orbits proximity ops
        num_steps: 60,
    };

    let mission = plan_mission(
        &chief_sv,
        &distant_sv,
        &perch,
        &config,
        &propagator,
        &mission_plan_config,
    )
    .expect("far-field mission failed");

    if let Some(ref transfer) = mission.transfer {
        println!("Lambert Transfer:");
        println!("  Total dv: {:.4} km/s", transfer.total_dv);
        println!("  Departure dv: {:.4} km/s", transfer.departure_dv.norm());
        println!("  Arrival dv:   {:.4} km/s", transfer.arrival_dv.norm());
        println!("  TOF:          {:.1} s", transfer.tof);
        if let Some(c3) = transfer.c3_km2_s2 {
            println!("  C3:           {:.4} km²/s²", c3);
        }
    }

    println!(
        "\nPerch ROE: da={:.6e}, dlambda={:.6e}",
        mission.perch_roe.da, mission.perch_roe.dlambda
    );
    println!(
        "Proximity trajectory: {} points",
        mission.proximity_trajectory.len()
    );

    let prox_final = mission.proximity_trajectory.last().unwrap();
    println!(
        "Final proximity RIC: [{:.4}, {:.4}, {:.4}] km",
        prox_final.ric.position.x,
        prox_final.ric.position.y,
        prox_final.ric.position.z,
    );

    // === J2 + Differential Drag Comparison (DMF, Koenig Sec. VIII) ===
    println!("\n\n=== J2 + Drag Comparison (DMF, Koenig Sec. VIII) ===\n");

    let roe = compute_roe(&chief_mean, &deputy_mean);
    let n_steps = 30 * n_orbits;
    let total_time = period * n_orbits as f64;

    let trajectory = propagator
        .propagate_with_steps(&roe, &chief_mean, epoch, total_time, n_steps)
        .expect("propagation failed");

    let drag = DragConfig {
        da_dot: -1e-10,
        dex_dot: 1e-11,
        dey_dot: -1e-11,
    };
    println!(
        "Drag config: da_dot={:.2e}, dex_dot={:.2e}, dey_dot={:.2e}",
        drag.da_dot, drag.dex_dot, drag.dey_dot
    );

    let drag_prop = J2DragStmPropagator { drag };
    let drag_traj = drag_prop
        .propagate_with_steps(&roe, &chief_mean, epoch, total_time, n_steps)
        .expect("drag propagation failed");

    let j2_final = trajectory.last().unwrap();
    let drag_final = drag_traj.last().unwrap();
    println!("\nDrag effect after {n_orbits} orbits:");
    println!(
        "  Along-track delta: {:.6} km",
        drag_final.ric.position.y - j2_final.ric.position.y
    );
    println!(
        "  Radial delta:      {:.6} km",
        drag_final.ric.position.x - j2_final.ric.position.x
    );
    println!(
        "  Cross-track delta: {:.6} km",
        drag_final.ric.position.z - j2_final.ric.position.z
    );
}
