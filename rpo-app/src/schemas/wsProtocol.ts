import * as S from '@railway-ts/pipelines/schema';

import type {
  DragConfig,
  EnrichmentSuggestion,
  LambertConfig,
  MissionConfig,
  MissionCovarianceReport,
  MonteCarloConfig,
  MonteCarloReport,
  PerchGeometry,
  ProximityConfig,
  SafetyRequirements,
  SpacecraftConfig,
  StateVector,
  TransferResult,
  ValidationReport,
  WaypointMission,
} from 'rpo-wasm';

// ─── Wire tags ─────────────────────────────────────────────────────────────

export const CLIENT_MESSAGE_TYPES = [
  'compute_transfer',
  'extract_drag',
  'validate',
  'run_mc',
  'cancel',
] as const;

export const SERVER_MESSAGE_TYPES = [
  'transfer_result',
  'drag_result',
  'validation_result',
  'monte_carlo_result',
  'progress',
  'error',
  'cancelled',
  'heartbeat',
] as const;

export type ClientMessageType = (typeof CLIENT_MESSAGE_TYPES)[number];
export type ServerMessageType = (typeof SERVER_MESSAGE_TYPES)[number];

// ─── Error codes ───────────────────────────────────────────────────────────

// Server-sourced codes mirror `rpo-api/src/protocol.rs` ServerErrorCode.
// `connection_lost` is a client-only synthetic code for promises drained on
// socket close — it never appears on the wire.
export const SERVER_ERROR_CODES = [
  'lambert_failure',
  'nyx_bridge_error',
  'validation_error',
  'monte_carlo_error',
  'invalid_input',
  'cancelled',
] as const;

export type ServerErrorCode = (typeof SERVER_ERROR_CODES)[number] | 'connection_lost';

export type ServerError = {
  request_id?: number;
  code: ServerErrorCode;
  message: string;
  detail?: unknown;
};

// ─── Progress phase ────────────────────────────────────────────────────────

export const PROGRESS_PHASES = ['validate', 'mc'] as const;
export type ProgressPhase = (typeof PROGRESS_PHASES)[number];

// ─── Client → server message shapes ────────────────────────────────────────

export type ComputeTransferMsg = {
  type: 'compute_transfer';
  request_id: number;
  chief_eci: StateVector;
  deputy_eci: StateVector;
  perch: PerchGeometry;
  proximity: ProximityConfig;
  lambert_tof_s: number;
  lambert_config: LambertConfig;
  safety_requirements?: SafetyRequirements;
};

export type ExtractDragMsg = {
  type: 'extract_drag';
  request_id: number;
  chief_eci: StateVector;
  deputy_eci: StateVector;
  chief_config: SpacecraftConfig;
  deputy_config: SpacecraftConfig;
};

// `propagator` is an externally-tagged enum on the server: "j2" or { j2_drag: { drag: DragConfig } }.
export type PropagatorChoiceMsg = 'j2' | { j2_drag: { drag: DragConfig } };

export type ValidateMsg = {
  type: 'validate';
  request_id: number;
  mission: WaypointMission;
  chief_eci: StateVector;
  deputy_eci: StateVector;
  chief_config: SpacecraftConfig;
  deputy_config: SpacecraftConfig;
  samples_per_leg: number;
  cola_burns?: unknown[];
  analytical_cola?: unknown[];
  cola_target_distance_km?: number;
};

export type RunMcMsg = {
  type: 'run_mc';
  request_id: number;
  mission: WaypointMission;
  chief_eci: StateVector;
  deputy_eci: StateVector;
  chief_config: SpacecraftConfig;
  deputy_config: SpacecraftConfig;
  mission_config: MissionConfig;
  propagator: PropagatorChoiceMsg;
  drag_config?: DragConfig;
  monte_carlo: MonteCarloConfig;
  covariance_report?: MissionCovarianceReport;
};

export type CancelMsg = { type: 'cancel'; request_id: number };

export type ClientMessage =
  | ComputeTransferMsg
  | ExtractDragMsg
  | ValidateMsg
  | RunMcMsg
  | CancelMsg;

// ─── Server → client message shapes ────────────────────────────────────────

export type TransferResultMsg = {
  type: 'transfer_result';
  request_id: number;
  result: TransferResult;
  enrichment?: EnrichmentSuggestion;
};

export type DragResultMsg = {
  type: 'drag_result';
  request_id: number;
  drag: DragConfig;
};

export type ValidationResultMsg = {
  type: 'validation_result';
  request_id: number;
  report: ValidationReport;
};

export type MonteCarloResultMsg = {
  type: 'monte_carlo_result';
  request_id: number;
  report: MonteCarloReport;
};

export type ProgressMsg = {
  type: 'progress';
  request_id: number;
  phase: ProgressPhase;
  detail?: string;
  fraction?: number;
};

export type ErrorMsg = {
  type: 'error';
  request_id?: number;
  code: Exclude<ServerErrorCode, 'connection_lost'>;
  message: string;
  detail?: unknown;
};

export type CancelledMsg = { type: 'cancelled'; request_id: number };

export type HeartbeatMsg = { type: 'heartbeat'; seq: number };

export type ServerMessage =
  | TransferResultMsg
  | DragResultMsg
  | ValidationResultMsg
  | MonteCarloResultMsg
  | ProgressMsg
  | ErrorMsg
  | CancelledMsg
  | HeartbeatMsg;

// Maps a request-emitting client message to the success-response variant it
// waits for. Used by the client singleton to register resolvers.
export type ResponseForRequest<T extends ClientMessage> = T extends ComputeTransferMsg
  ? TransferResultMsg
  : T extends ExtractDragMsg
    ? DragResultMsg
    : T extends ValidateMsg
      ? ValidationResultMsg
      : T extends RunMcMsg
        ? MonteCarloResultMsg
        : T extends CancelMsg
          ? CancelledMsg
          : never;

// ─── Validation schemas (parse-at-boundary) ────────────────────────────────

// Nested rpo-wasm payloads pass through as `unknown` on the wire and are cast
// to their tsify-generated TS types. rpo-wasm is the authoritative source —
// duplicating its ~400-line surface as railway-ts schemas would only rot.
const unknownValue: S.Validator<unknown, unknown> = S.transform<unknown, unknown>((v) => v);

const transferResultSchema = S.object({
  type: S.required(S.literal('transfer_result')),
  request_id: S.required(S.number()),
  result: S.required(unknownValue),
  enrichment: S.optional(unknownValue),
});

const dragResultSchema = S.object({
  type: S.required(S.literal('drag_result')),
  request_id: S.required(S.number()),
  drag: S.required(unknownValue),
});

const validationResultSchema = S.object({
  type: S.required(S.literal('validation_result')),
  request_id: S.required(S.number()),
  report: S.required(unknownValue),
});

const monteCarloResultSchema = S.object({
  type: S.required(S.literal('monte_carlo_result')),
  request_id: S.required(S.number()),
  report: S.required(unknownValue),
});

const progressSchema = S.object({
  type: S.required(S.literal('progress')),
  request_id: S.required(S.number()),
  phase: S.required(S.stringEnum<ProgressPhase>([...PROGRESS_PHASES], 'unknown progress phase')),
  detail: S.optional(S.string()),
  fraction: S.optional(S.number()),
});

const errorSchema = S.object({
  type: S.required(S.literal('error')),
  request_id: S.optional(S.number()),
  code: S.required(
    S.stringEnum<Exclude<ServerErrorCode, 'connection_lost'>>(
      [...SERVER_ERROR_CODES],
      'unknown server error code',
    ),
  ),
  message: S.required(S.string()),
  detail: S.optional(unknownValue),
});

const cancelledSchema = S.object({
  type: S.required(S.literal('cancelled')),
  request_id: S.required(S.number()),
});

const heartbeatSchema = S.object({
  type: S.required(S.literal('heartbeat')),
  seq: S.required(S.number()),
});

export const serverMessageSchema = S.discriminatedUnion('type', {
  transfer_result: transferResultSchema,
  drag_result: dragResultSchema,
  validation_result: validationResultSchema,
  monte_carlo_result: monteCarloResultSchema,
  progress: progressSchema,
  error: errorSchema,
  cancelled: cancelledSchema,
  heartbeat: heartbeatSchema,
});

// ─── Request-id counter ────────────────────────────────────────────────────

let nextId = 1;

export function nextRequestId(): number {
  const id = nextId;
  nextId += 1;
  return id;
}
