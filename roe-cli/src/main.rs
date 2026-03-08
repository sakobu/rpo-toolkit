use roe_core::{
    compute_roe, keplerian_to_state, roe_to_ric, state_to_keplerian, KeplerianElements,
};

fn main() {
    println!("ROE-RUST RPO Mission Planner — Phase 1");

    // Example: ISS-like chief orbit
    let epoch = hifitime::Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0);

    let chief_ke = KeplerianElements {
        a: 6786.0,
        e: 0.0001,
        i: 51.6_f64.to_radians(),
        raan: 30.0_f64.to_radians(),
        aop: 0.0,
        mean_anomaly: 0.0,
    };

    // Deputy: 1 km higher, small phase offset
    let deputy_ke = KeplerianElements {
        a: 6787.0,
        e: 0.0001,
        i: 51.6_f64.to_radians(),
        raan: 30.0_f64.to_radians(),
        aop: 0.0,
        mean_anomaly: 0.01,
    };

    let chief_sv = keplerian_to_state(&chief_ke, epoch);
    let deputy_sv = keplerian_to_state(&deputy_ke, epoch);

    println!("Chief ECI:  pos={:.3} km", chief_sv.position.transpose());
    println!("Deputy ECI: pos={:.3} km", deputy_sv.position.transpose());

    let chief_ke2 = state_to_keplerian(&chief_sv);
    let deputy_ke2 = state_to_keplerian(&deputy_sv);

    let roe = compute_roe(&chief_ke2, &deputy_ke2);
    println!(
        "\nQNS ROEs:\n  da={:.6e}\n  dlambda={:.6e}\n  dex={:.6e}\n  dey={:.6e}\n  dix={:.6e}\n  diy={:.6e}",
        roe.da, roe.dlambda, roe.dex, roe.dey, roe.dix, roe.diy
    );

    let ric = roe_to_ric(&roe, &chief_ke2);
    println!(
        "\nRIC position: [{:.6}, {:.6}, {:.6}] km",
        ric.position.x, ric.position.y, ric.position.z
    );
    println!(
        "RIC velocity: [{:.6}, {:.6}, {:.6}] km/s",
        ric.velocity.x, ric.velocity.y, ric.velocity.z
    );
}
