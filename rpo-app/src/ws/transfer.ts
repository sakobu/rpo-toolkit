import { pipe } from '@railway-ts/pipelines/composition';
import { mapWith, type Result } from '@railway-ts/pipelines/result';

import type {
  EnrichmentSuggestion,
  LambertConfig,
  PerchGeometry,
  ProximityConfig,
  SafetyRequirements,
  StateVector,
  TransferResult,
} from 'rpo-wasm';

import { nextRequestId, type ServerError } from '@/schemas/wsProtocol';

import { send } from './client';

export type ComputeTransferInput = {
  chief_eci: StateVector;
  deputy_eci: StateVector;
  perch: PerchGeometry;
  proximity: ProximityConfig;
  lambert_tof_s: number;
  lambert_config: LambertConfig;
  safety_requirements?: SafetyRequirements;
};

export type ComputeTransferOutput = {
  transfer: TransferResult;
  enrichment: EnrichmentSuggestion | null;
};

export async function computeTransfer(
  input: ComputeTransferInput,
): Promise<Result<ComputeTransferOutput, ServerError>> {
  const envelope = await send({
    type: 'compute_transfer',
    request_id: nextRequestId(),
    ...input,
  });
  return pipe(
    envelope,
    mapWith((msg) => ({ transfer: msg.result, enrichment: msg.enrichment ?? null })),
  );
}
