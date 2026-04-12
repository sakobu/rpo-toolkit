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
///
/// # Thread safety
/// `MetaAlmanac::latest()` is **not** safe against concurrent callers
/// refreshing the same on-disk BPC file — parallel entry can intermittently
/// fail with "file cannot be inspected or loaded directly in ANISE".
/// Production call sites (CLI one-shots, app startup) call this once per
/// process, so they are unaffected. **Test code must route through
/// `shared_almanac_for_tests` instead** (gated behind the `test-support`
/// Cargo feature), which serializes the load via `OnceLock::get_or_init`.
pub fn load_full_almanac() -> Result<Arc<Almanac>, NyxBridgeError> {
    let almanac = MetaAlmanac::latest()
        .map_err(|e| NyxBridgeError::AlmanacLoad { source: Box::new(e) })?;
    Ok(Arc::new(almanac))
}

/// Return a process-wide shared full-physics almanac for use in tests.
///
/// This is the test-scoped companion to [`load_full_almanac`]. The first
/// caller takes the `OnceLock`'s internal lock, runs `load_full_almanac`,
/// and stores the resulting `Arc`; every subsequent caller — in the same
/// process, across any number of threads — returns a cheap `Arc::clone` of
/// the cached handle. `OnceLock::get_or_init` guarantees the initialiser
/// closure runs **exactly once** even under concurrent entry, which is
/// precisely the invariant `MetaAlmanac::latest()` needs.
///
/// This helper is gated behind the `test-support` Cargo feature and is
/// not compiled into production builds. Production code should call
/// [`load_full_almanac`] directly so failed loads surface as real
/// `Result::Err` values rather than panics.
///
/// # Panics
/// Panics if the underlying [`load_full_almanac`] call returns an error.
/// Tests cannot proceed without SPICE kernels, so fail-fast is the right
/// behavior; the `.expect` is the intended panic path, not an oversight.
/// Because this function lives behind `#[cfg(feature = "test-support")]`,
/// the `.expect` is not "library code" in the Codebase sense — it is
/// test-support code, and panics are conventional in test code.
#[cfg(feature = "test-support")]
#[must_use]
pub fn shared_almanac_for_tests() -> Arc<Almanac> {
    use std::sync::OnceLock;

    static CACHE: OnceLock<Arc<Almanac>> = OnceLock::new();
    Arc::clone(CACHE.get_or_init(|| {
        load_full_almanac().expect("shared test almanac must load")
    }))
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

    /// Reproducer for the parallel almanac-load race. Spawns at least 8
    /// threads (or `available_parallelism()`, whichever is larger) that all
    /// enter `shared_almanac_for_tests` simultaneously. All returns must
    /// point at the same underlying `Almanac` — `OnceLock::get_or_init`
    /// guarantees the initialiser runs exactly once even under concurrent
    /// entry, which fixes the race previously observed under
    /// `cargo test --workspace -- --include-ignored`.
    #[test]
    #[ignore = "downloads SPICE kernels on first run; requires --include-ignored"]
    fn shared_almanac_for_tests_is_thread_safe_under_parallel_entry() {
        use std::num::NonZeroUsize;
        use std::thread;

        let n = thread::available_parallelism()
            .map(NonZeroUsize::get)
            .unwrap_or(8)
            .max(8);

        thread::scope(|scope| {
            let handles: Vec<_> = (0..n)
                .map(|_| scope.spawn(shared_almanac_for_tests))
                .collect();
            let first = handles
                .into_iter()
                .enumerate()
                .map(|(idx, handle)| {
                    handle
                        .join()
                        .unwrap_or_else(|_| panic!("worker {idx} panicked"))
                })
                .reduce(|a, b| {
                    assert!(
                        Arc::ptr_eq(&a, &b),
                        "shared_almanac_for_tests must return pointer-equal Arcs",
                    );
                    a
                });
            assert!(first.is_some(), "at least one worker must return");
        });
    }

    /// Cache invariant: two sequential calls must return pointer-equal
    /// `Arc`s. Proves the `OnceLock` is actually caching rather than
    /// reloading on every call.
    #[test]
    #[ignore = "downloads SPICE kernels on first run; requires --include-ignored"]
    fn shared_almanac_for_tests_returns_same_arc_across_calls() {
        let first = shared_almanac_for_tests();
        let second = shared_almanac_for_tests();
        assert!(
            Arc::ptr_eq(&first, &second),
            "shared_almanac_for_tests must cache: expected pointer-equal Arcs",
        );
    }
}
