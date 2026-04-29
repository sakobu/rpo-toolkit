import { pipe } from '@railway-ts/pipelines/composition';
import { mapWith, type Result } from '@railway-ts/pipelines/result';

import type {
  ComputeTransferOutput,
  EnrichmentSuggestion,
  LambertConfig,
  LambertTransfer,
  PerchGeometry,
  ProximityConfig,
  SafetyRequirements,
  StateVector,
  TransferComputationInput,
  TransferResult,
  WasmError,
} from 'rpo-wasm';
import { compute_transfer_with_enrichment, solve_lambert_branches } from 'rpo-wasm';

import { callWasm } from './error';

export type ComputeTransferInput = {
  chief_eci: StateVector;
  deputy_eci: StateVector;
  perch: PerchGeometry;
  proximity: ProximityConfig;
  lambert_tof_s: number;
  lambert_config: LambertConfig;
  safety_requirements?: SafetyRequirements;
};

export type ComputeTransferResult = {
  transfer: TransferResult;
  enrichment: EnrichmentSuggestion;
};

/**
 * Compute a Lambert transfer in-browser via WASM.
 *
 * The in-tree Izzo solver runs in microseconds. The accompanying
 * `enrichment` is always present: `{ status: 'baseline', perch_roe }` when
 * no `safety_requirements` is supplied, otherwise `{ status: 'enriched',
 * safe_perch, requirements }` on success or a `WasmError` with
 * `code: 'formation'` on failure.
 */
export function computeTransfer(
  input: ComputeTransferInput,
): Result<ComputeTransferResult, WasmError> {
  const transferInput: TransferComputationInput = {
    chief: input.chief_eci,
    deputy: input.deputy_eci,
    perch: input.perch,
    proximity: input.proximity,
    lambert_tof_s: input.lambert_tof_s,
    lambert_config: input.lambert_config,
  };

  return pipe(
    callWasm<ComputeTransferOutput>(() =>
      compute_transfer_with_enrichment(transferInput, input.safety_requirements),
    ),
    mapWith((out) => ({
      transfer: out.transfer,
      enrichment: out.enrichment,
    })),
  );
}

/**
 * Enumerate every Lambert branch up to `maxRevs`, sorted by total Δv ascending.
 *
 * Surfaces the multi-solution capability of the in-tree Izzo solver. Will be
 * wired into a future "browse transfer options" UX; for now lives at the
 * boundary so consumers can experiment.
 */
export function solveLambertBranches(
  departure: StateVector,
  arrival: StateVector,
  maxRevs: number,
): Result<LambertTransfer[], WasmError> {
  return callWasm(() => solve_lambert_branches(departure, arrival, maxRevs));
}
