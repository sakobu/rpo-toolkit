//! Criterion benchmarks for core analytical operations.
//!
//! Benchmark files cannot use `test_helpers` (not compiled with `cfg(test)`),
//! so fixture data is defined inline.

use criterion::{criterion_group, criterion_main, Criterion};
use hifitime::Epoch;
use nalgebra::Vector3;

use rpo_core::elements::{keplerian_to_state, roe_to_ric, state_to_keplerian};
use rpo_core::mission::{
    analyze_safety, assess_cola, classify_separation, compute_ei_separation, compute_free_drift,
    find_closest_approaches, plan_waypoint_mission, solve_leg, ColaConfig, MissionConfig,
    ProximityConfig, TargetingConfig, Waypoint,
};
use rpo_core::propagation::{DragConfig, PropagationModel};
use rpo_core::types::{DepartureState, KeplerianElements, QuasiNonsingularROE, StateVector};

// ---------------------------------------------------------------------------
// Inline fixtures (ISS-like chief, ~200 m deputy offset)
// ---------------------------------------------------------------------------

fn iss_chief() -> KeplerianElements {
    KeplerianElements {
        a_km: 6786.0,
        e: 0.0001,
        i_rad: 51.6_f64.to_radians(),
        raan_rad: 0.0,
        aop_rad: 0.0,
        mean_anomaly_rad: 0.0,
    }
}

fn epoch() -> Epoch {
    Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0)
}

fn small_roe(chief: &KeplerianElements) -> QuasiNonsingularROE {
    QuasiNonsingularROE {
        da: 1.0 / chief.a_km,
        dlambda: 0.001,
        dex: 0.0001,
        dey: 0.0001,
        dix: 0.0005,
        diy: 0.0003,
    }
}

fn departure(chief: &KeplerianElements) -> DepartureState {
    DepartureState {
        roe: small_roe(chief),
        chief: *chief,
        epoch: epoch(),
    }
}

// ---------------------------------------------------------------------------
// Benchmarks
// ---------------------------------------------------------------------------

fn bench_classify_separation(c: &mut Criterion) {
    let chief = iss_chief();
    let ep = epoch();
    let chief_sv = keplerian_to_state(&chief, ep).unwrap();

    // Deputy ~200 m in-track offset
    let deputy_sv = StateVector {
        epoch: ep,
        position_eci_km: chief_sv.position_eci_km + Vector3::new(0.0, 0.2, 0.0),
        velocity_eci_km_s: chief_sv.velocity_eci_km_s,
    };
    let config = ProximityConfig::default();

    c.bench_function("classify_separation", |b| {
        b.iter(|| classify_separation(&chief_sv, &deputy_sv, &config));
    });
}

fn bench_plan_waypoint_mission(c: &mut Criterion) {
    let chief = iss_chief();
    let dep = departure(&chief);
    let propagator = PropagationModel::J2Stm;
    let config = MissionConfig::default();

    let waypoints = vec![
        Waypoint {
            position_ric_km: Vector3::new(0.0, 0.5, 0.0),
            velocity_ric_km_s: None,
            tof_s: Some(3600.0),
        },
        Waypoint {
            position_ric_km: Vector3::new(0.0, 0.0, 0.0),
            velocity_ric_km_s: Some(Vector3::new(0.0, 0.0, 0.0)),
            tof_s: Some(3600.0),
        },
    ];

    c.bench_function("plan_waypoint_mission", |b| {
        b.iter(|| plan_waypoint_mission(&dep, &waypoints, &config, &propagator));
    });
}

fn bench_solve_leg(c: &mut Criterion) {
    let chief = iss_chief();
    let dep = departure(&chief);
    let propagator = PropagationModel::J2Stm;
    let config = TargetingConfig::default();

    let target_pos = Vector3::new(0.0, 0.5, 0.0);
    let target_vel = Vector3::new(0.0, 0.0, 0.0);
    let tof = 3600.0;

    c.bench_function("solve_leg", |b| {
        b.iter(|| solve_leg(&dep, &target_pos, &target_vel, tof, &config, &propagator));
    });
}

fn bench_propagate_j2stm(c: &mut Criterion) {
    let chief = iss_chief();
    let roe = small_roe(&chief);
    let ep = epoch();
    let propagator = PropagationModel::J2Stm;
    let period = chief.period().unwrap();

    c.bench_function("propagate_j2stm", |b| {
        b.iter(|| propagator.propagate(&roe, &chief, ep, period));
    });
}

fn bench_roe_to_ric(c: &mut Criterion) {
    let chief = iss_chief();
    let roe = small_roe(&chief);

    c.bench_function("roe_to_ric", |b| {
        b.iter(|| roe_to_ric(&roe, &chief));
    });
}

fn bench_analyze_safety(c: &mut Criterion) {
    let chief = iss_chief();
    let roe = small_roe(&chief);
    let ric_pos = Vector3::new(0.0, 0.5, 0.0);

    c.bench_function("analyze_safety", |b| {
        b.iter(|| analyze_safety(&roe, &chief, &ric_pos));
    });
}

fn bench_compute_ei_separation(c: &mut Criterion) {
    let chief = iss_chief();
    let roe = small_roe(&chief);

    c.bench_function("compute_ei_separation", |b| {
        b.iter(|| compute_ei_separation(&roe, &chief));
    });
}

fn bench_assess_cola(c: &mut Criterion) {
    let chief = iss_chief();
    let dep = departure(&chief);
    let propagator = PropagationModel::J2Stm;
    let config = MissionConfig::default();

    let waypoints = vec![
        Waypoint {
            position_ric_km: Vector3::new(0.0, 0.5, 0.0),
            velocity_ric_km_s: None,
            tof_s: Some(3600.0),
        },
        Waypoint {
            position_ric_km: Vector3::new(0.0, 0.0, 0.0),
            velocity_ric_km_s: Some(Vector3::new(0.0, 0.0, 0.0)),
            tof_s: Some(3600.0),
        },
    ];

    let mission = plan_waypoint_mission(&dep, &waypoints, &config, &propagator).unwrap();

    // Compute POCA for each leg
    let poca: Vec<Vec<_>> = mission
        .legs
        .iter()
        .enumerate()
        .map(|(i, leg)| {
            find_closest_approaches(
                &leg.trajectory,
                &leg.departure_chief_mean,
                leg.departure_maneuver.epoch,
                &propagator,
                &leg.post_departure_roe,
                i,
            )
            .unwrap_or_default()
        })
        .collect();

    let cola_config = ColaConfig {
        target_distance_km: 0.5,
        max_dv_km_s: 0.001,
    };

    c.bench_function("assess_cola", |b| {
        b.iter(|| assess_cola(&mission, &poca, &propagator, &cola_config));
    });
}

fn bench_propagate_j2_drag_stm(c: &mut Criterion) {
    let chief = iss_chief();
    let roe = small_roe(&chief);
    let ep = epoch();
    let period = chief.period().unwrap();

    // Representative differential drag: da_dot ~ -1e-10 /s (Koenig Appendix D test value)
    let propagator = PropagationModel::J2DragStm {
        drag: DragConfig {
            da_dot: -1e-10,
            dex_dot: 1e-12,
            dey_dot: -1e-12,
        },
    };

    c.bench_function("propagate_j2_drag_stm", |b| {
        b.iter(|| propagator.propagate(&roe, &chief, ep, period));
    });
}

fn bench_find_closest_approaches(c: &mut Criterion) {
    let chief = iss_chief();
    let dep = departure(&chief);
    let propagator = PropagationModel::J2Stm;
    let config = MissionConfig::default();

    let waypoints = vec![Waypoint {
        position_ric_km: Vector3::new(0.0, 0.5, 0.0),
        velocity_ric_km_s: Some(Vector3::new(0.0, 0.0, 0.0)),
        tof_s: Some(3600.0),
    }];

    let mission = plan_waypoint_mission(&dep, &waypoints, &config, &propagator).unwrap();
    let leg = &mission.legs[0];

    c.bench_function("find_closest_approaches", |b| {
        b.iter(|| {
            find_closest_approaches(
                &leg.trajectory,
                &leg.departure_chief_mean,
                leg.departure_maneuver.epoch,
                &propagator,
                &leg.post_departure_roe,
                0,
            )
        });
    });
}

fn bench_compute_free_drift(c: &mut Criterion) {
    let chief = iss_chief();
    let roe = small_roe(&chief);
    let ep = epoch();
    let propagator = PropagationModel::J2Stm;
    let period = chief.period().unwrap();

    c.bench_function("compute_free_drift", |b| {
        b.iter(|| compute_free_drift(&roe, &chief, ep, period, &propagator, 200));
    });
}

fn bench_keplerian_conversions(c: &mut Criterion) {
    let chief = iss_chief();
    let ep = epoch();
    let sv = keplerian_to_state(&chief, ep).unwrap();

    c.bench_function("keplerian_to_state", |b| {
        b.iter(|| keplerian_to_state(&chief, ep));
    });

    c.bench_function("state_to_keplerian", |b| {
        b.iter(|| state_to_keplerian(&sv));
    });
}

criterion_group!(
    benches,
    bench_classify_separation,
    bench_plan_waypoint_mission,
    bench_solve_leg,
    bench_propagate_j2stm,
    bench_propagate_j2_drag_stm,
    bench_roe_to_ric,
    bench_analyze_safety,
    bench_compute_ei_separation,
    bench_assess_cola,
    bench_find_closest_approaches,
    bench_compute_free_drift,
    bench_keplerian_conversions,
);
criterion_main!(benches);
