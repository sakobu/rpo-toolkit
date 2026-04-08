//! Spacecraft physical properties for force modeling.

use serde::{Deserialize, Serialize};

/// Physical properties of a spacecraft for force modeling.
///
/// Entry point for the entire tool: user defines spacecraft properties,
/// and everything downstream derives from them:
/// - nyx full-physics propagation (via `config_to_spacecraft()`)
/// - Analytical DMF drag rates (via `extract_dmf_rates()` → `DragConfig`)
/// - Mission validation (via `validate_mission_nyx()`)
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SpacecraftConfig {
    /// Spacecraft dry mass in kg
    pub dry_mass_kg: f64,
    /// Drag reference area in m²
    pub drag_area_m2: f64,
    /// Coefficient of drag (dimensionless, typically 2.0–2.5)
    pub coeff_drag: f64,
    /// SRP reference area in m²
    pub srp_area_m2: f64,
    /// Coefficient of reflectivity (dimensionless, typically 1.0–2.0)
    pub coeff_reflectivity: f64,
}

impl SpacecraftConfig {
    /// Typical 6U cubesat: 12 kg, 0.06 m² cross-section.
    pub const CUBESAT_6U: Self = Self {
        dry_mass_kg: 12.0,
        drag_area_m2: 0.06,
        coeff_drag: 2.2,
        srp_area_m2: 0.06,
        coeff_reflectivity: 1.5,
    };

    /// Typical 500 kg servicer spacecraft (also the `Default`).
    pub const SERVICER_500KG: Self = Self {
        dry_mass_kg: 500.0,
        drag_area_m2: 1.0,
        coeff_drag: 2.2,
        srp_area_m2: 1.0,
        coeff_reflectivity: 1.5,
    };
}
