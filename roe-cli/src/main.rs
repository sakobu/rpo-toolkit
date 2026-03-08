use roe_core::{
    compute_j2_params, compute_roe, keplerian_to_state, roe_to_ric, DragConfig,
    J2DragStmPropagator, J2StmPropagator, KeplerianElements, RelativePropagator,
};

fn main() {
    println!("ROE-RUST RPO Mission Planner — Phase 3A");
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

    // Step 2: Compute QNS ROEs (directly from mean elements)
    let roe = compute_roe(&chief_mean, &deputy_mean);
    println!(
        "\nQNS ROEs:\n  da={:.6e}\n  dlambda={:.6e}\n  dex={:.6e}\n  dey={:.6e}\n  dix={:.6e}\n  diy={:.6e}",
        roe.da, roe.dlambda, roe.dex, roe.dey, roe.dix, roe.diy
    );

    // Step 3: J2 parameters
    let j2p = compute_j2_params(&chief_mean);
    println!("\nJ2 Secular Rates:");
    println!(
        "  RAAN rate: {:.4} deg/day",
        j2p.raan_dot.to_degrees() * 86400.0
    );
    println!(
        "  AoP rate:  {:.4} deg/day",
        j2p.aop_dot.to_degrees() * 86400.0
    );

    // Step 4: Initial RIC state
    let ric = roe_to_ric(&roe, &chief_mean);
    println!(
        "\nInitial RIC position: [{:.4}, {:.4}, {:.4}] km",
        ric.position.x, ric.position.y, ric.position.z
    );

    // Step 5: Propagate over 3 orbits with 30 steps per orbit
    let period = std::f64::consts::TAU / j2p.n;
    let n_orbits = 3;
    let n_steps = 30 * n_orbits;
    let total_time = period * n_orbits as f64;

    let propagator = J2StmPropagator;
    let trajectory = propagator
        .propagate_with_steps(&roe, &chief_mean, epoch, total_time, n_steps)
        .expect("propagation failed");

    println!("\nPropagated trajectory ({n_orbits} orbits, {n_steps} steps):");
    println!(
        "{:>10}  {:>12}  {:>12}  {:>12}",
        "Time (s)", "R (km)", "I (km)", "C (km)"
    );

    for (k, state) in trajectory.iter().enumerate() {
        if k % 10 == 0 || k == trajectory.len() - 1 {
            println!(
                "{:10.1}  {:12.4}  {:12.4}  {:12.4}",
                state.elapsed_s,
                state.ric.position.x,
                state.ric.position.y,
                state.ric.position.z,
            );
        }
    }

    let final_state = trajectory.last().unwrap();
    println!(
        "\nFinal RIC position: [{:.4}, {:.4}, {:.4}] km",
        final_state.ric.position.x,
        final_state.ric.position.y,
        final_state.ric.position.z
    );
    println!(
        "Final RIC velocity: [{:.6}, {:.6}, {:.6}] km/s",
        final_state.ric.velocity.x,
        final_state.ric.velocity.y,
        final_state.ric.velocity.z
    );

    // === Phase 3A: J2 + Differential Drag comparison ===
    println!("\n\n=== J2 + Drag Comparison (DMF, Koenig Sec. VIII) ===\n");

    let drag = DragConfig {
        da_dot: -1e-10,   // differential SMA decay
        dex_dot: 1e-11,   // eccentricity x drift
        dey_dot: -1e-11,  // eccentricity y drift
    };
    println!(
        "Drag config: da_dot={:.2e}, dex_dot={:.2e}, dey_dot={:.2e}",
        drag.da_dot, drag.dex_dot, drag.dey_dot
    );

    let drag_prop = J2DragStmPropagator { drag };
    let drag_traj = drag_prop
        .propagate_with_steps(&roe, &chief_mean, epoch, total_time, n_steps)
        .expect("drag propagation failed");

    println!(
        "\n{:>10}  {:>12} {:>12}  {:>12} {:>12}",
        "Time (s)", "I_j2 (km)", "I_drag (km)", "dI (km)", "dR (km)"
    );

    for k in (0..drag_traj.len()).step_by(10) {
        let j2_s = &trajectory[k];
        let dr_s = &drag_traj[k];
        let di = dr_s.ric.position.y - j2_s.ric.position.y;
        let dr = dr_s.ric.position.x - j2_s.ric.position.x;
        println!(
            "{:10.1}  {:12.4} {:12.4}  {:12.6} {:12.6}",
            j2_s.elapsed_s,
            j2_s.ric.position.y,
            dr_s.ric.position.y,
            di,
            dr,
        );
    }

    let j2_final = trajectory.last().unwrap();
    let drag_final = drag_traj.last().unwrap();
    println!(
        "\nDrag effect after {n_orbits} orbits:"
    );
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
