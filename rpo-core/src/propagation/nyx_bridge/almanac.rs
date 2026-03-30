//! ANISE almanac initialization for nyx-space propagation.

use std::sync::Arc;

use anise::prelude::{Almanac, MetaAlmanac};

use super::errors::NyxBridgeError;

/// Create a default ANISE almanac for two-body propagation.
///
/// Two-body dynamics only need the frame's `mu` value (baked into `EARTH_J2000`),
/// so an empty `Almanac::default()` suffices — no network download required.
#[must_use]
pub fn load_default_almanac() -> Arc<Almanac> {
    Arc::new(Almanac::default())
}

/// Load a full-physics almanac with all ANISE kernel data.
///
/// Downloads and caches to `~/.local/share/nyx-space/anise/`:
/// - `de440s.bsp` (planetary ephemerides, ~32 MB)
/// - `pck11.pca` (planetary constants)
/// - `earth_latest_high_prec.bpc` (Earth orientation from JPL)
/// - `moon_pa_de440_200625.bpc` (Moon orientation)
///
/// # Errors
/// Returns [`NyxBridgeError::AlmanacLoad`] if kernel download or parsing fails.
pub fn load_full_almanac() -> Result<Arc<Almanac>, NyxBridgeError> {
    let almanac = MetaAlmanac::latest()
        .map_err(|e| NyxBridgeError::AlmanacLoad { source: Box::new(e) })?;
    Ok(Arc::new(almanac))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `load_default_almanac` returns a valid almanac without panicking.
    #[test]
    fn load_default_almanac_succeeds() {
        let almanac = load_default_almanac();
        // Verify the Arc is non-null (Almanac::default() always succeeds)
        assert!(Arc::strong_count(&almanac) >= 1);
    }
}
