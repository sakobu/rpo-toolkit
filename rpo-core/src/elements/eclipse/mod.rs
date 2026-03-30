//! Eclipse computation: analytical ephemeris, conical shadow model, and interval extraction.
//!
//! Provides Sun and Moon positions from any `hifitime::Epoch` using
//! low-precision analytical series (Meeus Ch. 25 and Ch. 47). No ephemeris
//! files, no ANISE dependency. Microseconds per call.
//!
//! # References
//!
//! - Meeus, Jean — *Astronomical Algorithms* (2nd ed., 1998), Ch. 25 (Sun), Ch. 47 (Moon)
//! - Montenbruck & Gill — *Satellite Orbits* (2000), Sec. 3.4 (shadow models)

pub(crate) mod ephemeris;
pub(crate) mod shadow;
mod snapshots;
mod intervals;
#[cfg(test)]
mod test_fixtures;

// Public API (visible outside the crate)
pub use ephemeris::{moon_position_eci_km, sun_position_eci_km};
pub use shadow::compute_eclipse_state;
pub use snapshots::{compute_celestial_snapshots, compute_eclipse_from_states};
pub use intervals::extract_eclipse_intervals;

// Crate-internal API (used by mission/waypoints, mission/validation)
pub(crate) use shadow::build_celestial_snapshot;
pub(crate) use intervals::is_deeper_shadow;
