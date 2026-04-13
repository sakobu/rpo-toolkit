//! Eclipse computation: analytical ephemeris, conical shadow model, and interval extraction.
//!
//! Provides Sun and Moon positions from any `hifitime::Epoch` using
//! low-precision analytical series (Meeus Ch. 25 and Ch. 47). No ephemeris
//! files, no ANISE dependency. Microseconds per call.
//!
//! # Accuracy budgets
//!
//! - Sun direction: ~0.01° (Meeus Eqs. 25.2–25.6, precession correction applied).
//! - Moon direction: ~0.5° (truncated Meeus Ch. 47 series; E-correction factor
//!   and quadratic mean-element terms omitted). Adequate for shadow geometry
//!   at LEO — the Sun subtends ~0.5° and Earth ~60°, so this level of
//!   directional error shifts shadow boundaries by well under a second.
//!
//! # References
//!
//! - Meeus, Jean — *Astronomical Algorithms* (2nd ed., 1998), Ch. 25 (Sun), Ch. 47 (Moon)
//! - Montenbruck & Gill — *Satellite Orbits* (2000), Sec. 3.4 (shadow models)

mod ephemeris;
mod errors;
mod intervals;
mod shadow;
mod snapshots;
#[cfg(test)]
mod test_fixtures;

// Public API (visible outside the crate)
pub use errors::EclipseGeometryError;
pub use ephemeris::{moon_position_eci_km, sun_position_eci_km};
pub use shadow::compute_eclipse_state;
pub use snapshots::{compute_celestial_snapshots, compute_eclipse_from_states};
pub use intervals::extract_eclipse_intervals;

// Crate-internal API (used by mission/waypoints, mission/validation)
pub(crate) use shadow::build_celestial_snapshot;
pub(crate) use intervals::is_deeper_shadow;
