//! Error types for the nyx-space integration bridge.

use anise::almanac::planetary::PlanetaryDataError;
use anise::errors::AlmanacError;
use nyx_space::dynamics::DynamicsError;
use nyx_space::propagators::PropagationError as NyxPropagationError;

use rpo_core::elements::keplerian_conversions::ConversionError;
use rpo_core::elements::eci_ric_dcm::DcmError;
use rpo_core::types::KeplerError;

/// Errors from the nyx-space integration bridge.
#[derive(Debug, thiserror::Error)]
pub enum NyxBridgeError {
    /// Operation cancelled cooperatively by caller.
    #[error("operation cancelled")]
    Cancelled,
    /// `MetaAlmanac` / ANISE kernel loading failure.
    #[error("almanac loading failed: {source}")]
    AlmanacLoad {
        /// The underlying almanac error.
        #[source]
        source: Box<AlmanacError>,
    },
    /// Frame information retrieval failure (`IAU_EARTH`, etc.)
    #[error("frame lookup failed: {source}")]
    FrameLookup {
        /// The underlying planetary data error.
        #[source]
        source: PlanetaryDataError,
    },
    /// Force model initialization failure (drag, SRP).
    #[error("dynamics setup failed: {source}")]
    DynamicsSetup {
        /// The underlying dynamics error.
        #[source]
        source: DynamicsError,
    },
    /// Nyx propagation failure.
    #[error("nyx propagation failed: {source}")]
    Propagation {
        /// The underlying nyx propagation error.
        #[source]
        source: NyxPropagationError,
    },
    /// ECI → Keplerian or ROE conversion failure.
    #[error("conversion failed: {source}")]
    Conversion {
        /// The underlying conversion error.
        #[source]
        source: ConversionError,
    },
    /// Nyx propagation returned no data points.
    #[error("nyx propagation returned no data points")]
    EmptyResult,
    /// ECI↔RIC frame conversion failed.
    #[error("frame conversion failed: {0}")]
    DcmFailure(#[from] DcmError),
    /// ANISE ephemeris translation query failed (e.g., Sun/Moon position).
    #[error("ephemeris query failed: {source}")]
    EphemerisQuery {
        /// The underlying ephemeris error.
        #[source]
        source: Box<anise::ephemerides::EphemerisError>,
    },
}

// Two-step tunnel: KeplerError → ConversionError → NyxBridgeError::Conversion.
// Hand-rolled because thiserror's #[from] only synthesizes one-hop conversions.
impl From<KeplerError> for NyxBridgeError {
    fn from(e: KeplerError) -> Self {
        Self::Conversion { source: ConversionError::from(e) }
    }
}

// Struct-form target variant (Conversion { source }) — #[from] only works on
// tuple variants, so this routing is hand-rolled.
impl From<ConversionError> for NyxBridgeError {
    fn from(e: ConversionError) -> Self {
        Self::Conversion { source: e }
    }
}

impl From<NyxPropagationError> for NyxBridgeError {
    fn from(e: NyxPropagationError) -> Self {
        Self::Propagation { source: e }
    }
}

impl From<DynamicsError> for NyxBridgeError {
    fn from(e: DynamicsError) -> Self {
        Self::DynamicsSetup { source: e }
    }
}

impl From<PlanetaryDataError> for NyxBridgeError {
    fn from(e: PlanetaryDataError) -> Self {
        Self::FrameLookup { source: e }
    }
}
