import * as S from '@railway-ts/pipelines/schema';

import type { TransferDirection } from 'rpo-wasm';

import { getEngineConstants } from '@/wasm/constants';

export const PERCH_MODE_VALUES = ['v_bar', 'r_bar', 'custom'] as const;
export type PerchMode = (typeof PERCH_MODE_VALUES)[number];

export const PERCH_MODE_LABELS: Record<PerchMode, string> = {
  v_bar: 'V-bar',
  r_bar: 'R-bar',
  custom: 'Custom',
};

export const PERCH_OFFSET_MIN_KM = -50;
export const PERCH_OFFSET_MAX_KM = 50;

export const QNS_COMPONENTS = ['da', 'dlambda', 'dex', 'dey', 'dix', 'diy'] as const;
export type QnsComponent = (typeof QNS_COMPONENTS)[number];

export const QNS_LABELS: Record<QnsComponent, string> = {
  da: 'δa',
  dlambda: 'δλ',
  dex: 'δex',
  dey: 'δey',
  dix: 'δix',
  diy: 'δiy',
};

export const LAMBERT_TOF_MIN_S = 60;
export const LAMBERT_TOF_MAX_S = 24 * 3600;
export const LAMBERT_TOF_DEFAULT_S = 3600;

export const DIRECTION_VALUES = [
  'auto',
  'short_way',
  'long_way',
] as const satisfies readonly TransferDirection[];

export const DIRECTION_LABELS: Record<TransferDirection, string> = {
  auto: 'Auto',
  short_way: 'Short way',
  long_way: 'Long way',
};

export const REVOLUTIONS_MIN = 0;
export const REVOLUTIONS_MAX = 5;

export const LAMBERT_REQUEST_DEFAULTS = {
  perch_mode: 'v_bar',
  perch_offset_km: 1,
  lambert_tof_s: LAMBERT_TOF_DEFAULT_S,
  direction: 'short_way',
  revolutions: 0,
  perch_da: 0,
  perch_dlambda: 0,
  perch_dex: 0,
  perch_dey: 0,
  perch_dix: 0,
  perch_diy: 0,
} as const satisfies {
  perch_mode: PerchMode;
  perch_offset_km: number;
  lambert_tof_s: number;
  direction: TransferDirection;
  revolutions: number;
  perch_da: number;
  perch_dlambda: number;
  perch_dex: number;
  perch_dey: number;
  perch_dix: number;
  perch_diy: number;
};

// Numeric field with type/finiteness checks only — range bounds are applied
// conditionally below so hidden fields (perch_offset_km in custom mode, the
// perch_d* fields outside custom mode) don't block `form.isValid` and freeze
// `useFormAutoSubmission`.
const numericField = S.required(
  S.chain(S.parseNumber('Must be a number'), S.finite('Must be finite')),
);

function baseLambertRequestSchema() {
  return S.object({
    perch_mode: S.required(
      S.stringEnum<PerchMode>([...PERCH_MODE_VALUES], 'Pick a perch reference'),
    ),
    perch_offset_km: S.required(
      S.chain(S.parseNumber('Offset must be a number'), S.finite('Offset must be finite')),
    ),
    lambert_tof_s: S.required(
      S.chain(
        S.parseNumber('TOF must be a number'),
        S.finite('TOF must be finite'),
        S.between(
          LAMBERT_TOF_MIN_S,
          LAMBERT_TOF_MAX_S,
          `TOF must be ${LAMBERT_TOF_MIN_S}–${LAMBERT_TOF_MAX_S} s`,
        ),
      ),
    ),
    direction: S.required(
      S.stringEnum<TransferDirection>([...DIRECTION_VALUES], 'Pick a transfer direction'),
    ),
    revolutions: S.required(
      S.chain(
        S.parseNumber('Revolutions must be a number'),
        S.integer('Revolutions must be a whole number'),
        S.between(
          REVOLUTIONS_MIN,
          REVOLUTIONS_MAX,
          `Revolutions must be ${REVOLUTIONS_MIN}–${REVOLUTIONS_MAX}`,
        ),
      ),
    ),
    perch_da: numericField,
    perch_dlambda: numericField,
    perch_dex: numericField,
    perch_dey: numericField,
    perch_dix: numericField,
    perch_diy: numericField,
  });
}

export type LambertFormValues = S.InferSchemaType<ReturnType<typeof baseLambertRequestSchema>>;

const OFFSET_BOUNDS_MSG = `Offset must be ${PERCH_OFFSET_MIN_KM}–${PERCH_OFFSET_MAX_KM} km`;

export function lambertRequestSchema() {
  const qnsBound = getEngineConstants().linearization_perturbation_bound;
  const qnsBoundMsg = `Must be ±${qnsBound}`;
  return S.chain(
    baseLambertRequestSchema(),
    S.when<LambertFormValues>(
      (d) => d.perch_mode !== 'custom',
      S.refineAt(
        'perch_offset_km',
        (d: LambertFormValues) =>
          d.perch_offset_km >= PERCH_OFFSET_MIN_KM && d.perch_offset_km <= PERCH_OFFSET_MAX_KM,
        OFFSET_BOUNDS_MSG,
      ),
    ),
    S.when<LambertFormValues>(
      (d) => d.perch_mode === 'custom',
      S.chain(
        S.refineAt(
          'perch_da',
          (d: LambertFormValues) => Math.abs(d.perch_da) <= qnsBound,
          qnsBoundMsg,
        ),
        S.refineAt(
          'perch_dlambda',
          (d: LambertFormValues) => Math.abs(d.perch_dlambda) <= qnsBound,
          qnsBoundMsg,
        ),
        S.refineAt(
          'perch_dex',
          (d: LambertFormValues) => Math.abs(d.perch_dex) <= qnsBound,
          qnsBoundMsg,
        ),
        S.refineAt(
          'perch_dey',
          (d: LambertFormValues) => Math.abs(d.perch_dey) <= qnsBound,
          qnsBoundMsg,
        ),
        S.refineAt(
          'perch_dix',
          (d: LambertFormValues) => Math.abs(d.perch_dix) <= qnsBound,
          qnsBoundMsg,
        ),
        S.refineAt(
          'perch_diy',
          (d: LambertFormValues) => Math.abs(d.perch_diy) <= qnsBound,
          qnsBoundMsg,
        ),
      ),
    ),
  );
}
