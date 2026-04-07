//! Error types for the nyx-space integration bridge.

use anise::almanac::planetary::PlanetaryDataError;
use anise::errors::AlmanacError;
use nyx_space::dynamics::DynamicsError;
use nyx_space::propagators::PropagationError as NyxPropagationError;

use rpo_core::elements::keplerian_conversions::ConversionError;
use rpo_core::elements::eci_ric_dcm::DcmError;
use rpo_core::types::KeplerError;

/// Errors from the nyx-space integration bridge.
#[derive(Debug)]
pub enum NyxBridgeError {
    /// `MetaAlmanac` / ANISE kernel loading failure.
    AlmanacLoad {
        /// The underlying almanac error.
        source: Box<AlmanacError>,
    },
    /// Frame information retrieval failure (`IAU_EARTH`, etc.)
    FrameLookup {
        /// The underlying planetary data error.
        source: PlanetaryDataError,
    },
    /// Force model initialization failure (drag, SRP).
    DynamicsSetup {
        /// The underlying dynamics error.
        source: DynamicsError,
    },
    /// Nyx propagation failure.
    Propagation {
        /// The underlying nyx propagation error.
        source: NyxPropagationError,
    },
    /// ECI → Keplerian or ROE conversion failure.
    Conversion {
        /// The underlying conversion error.
        source: ConversionError,
    },
    /// Nyx propagation returned no data points.
    EmptyResult,
    /// ECI↔RIC frame conversion failed.
    DcmFailure(DcmError),
    /// ANISE ephemeris translation query failed (e.g., Sun/Moon position).
    EphemerisQuery {
        /// The underlying ephemeris error.
        source: Box<anise::ephemerides::EphemerisError>,
    },
}

impl std::fmt::Display for NyxBridgeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::AlmanacLoad { source } => {
                write!(f, "almanac loading failed: {source}")
            }
            Self::FrameLookup { source } => {
                write!(f, "frame lookup failed: {source}")
            }
            Self::DynamicsSetup { source } => {
                write!(f, "dynamics setup failed: {source}")
            }
            Self::Propagation { source } => {
                write!(f, "nyx propagation failed: {source}")
            }
            Self::Conversion { source } => {
                write!(f, "conversion failed: {source}")
            }
            Self::EmptyResult => {
                write!(f, "nyx propagation returned no data points")
            }
            Self::DcmFailure(e) => {
                write!(f, "frame conversion failed: {e}")
            }
            Self::EphemerisQuery { source } => {
                write!(f, "ephemeris query failed: {source}")
            }
        }
    }
}

impl std::error::Error for NyxBridgeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::AlmanacLoad { source } => Some(source.as_ref()),
            Self::EphemerisQuery { source } => Some(source.as_ref()),
            Self::FrameLookup { source } => Some(source),
            Self::DynamicsSetup { source } => Some(source),
            Self::Propagation { source } => Some(source),
            Self::Conversion { source } => Some(source),
            Self::DcmFailure(e) => Some(e),
            Self::EmptyResult => None,
        }
    }
}

impl From<DcmError> for NyxBridgeError {
    fn from(e: DcmError) -> Self {
        Self::DcmFailure(e)
    }
}

impl From<KeplerError> for NyxBridgeError {
    fn from(e: KeplerError) -> Self {
        Self::Conversion { source: ConversionError::from(e) }
    }
}

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
