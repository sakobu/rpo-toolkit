import * as S from '@railway-ts/pipelines/schema';

const km = S.chain(S.number('Must be a number'), S.finite('Must be finite'));
const vec3 = S.tupleOf(km, 3);

// hifitime accepts ISO date-time optionally followed by a timescale tag
// (e.g. "2024-01-01T00:00:00 UTC"). We trim + collapse whitespace so equality
// comparisons in the mission store don't fail on incidental formatting drift.
const EPOCH_RE =
  /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d+)?(Z|[+-]\d{2}:\d{2}| (UTC|TAI|TDB|TT|ET|GPS))?$/;

const epochField = S.required(
  S.chain(
    S.string('epoch must be a string'),
    S.transform<string, string>((s) => s.trim().replaceAll(/\s+/g, ' ')),
    S.nonEmpty('epoch is required'),
    S.pattern(EPOCH_RE, 'epoch must look like 2024-01-01T00:00:00 UTC'),
  ),
);

const baseSchema = S.object({
  epoch: epochField,
  position_eci_km: S.required(vec3),
  velocity_eci_km_s: S.required(vec3),
});

type StateVectorShape = S.InferSchemaType<typeof baseSchema>;

export const stateVectorSchema = S.chain(
  baseSchema,
  S.refineAt<StateVectorShape>(
    'position_eci_km',
    (d) => d.position_eci_km.some((x) => x !== 0),
    'Position vector cannot be zero',
  ),
  S.refineAt<StateVectorShape>(
    'velocity_eci_km_s',
    (d) => d.velocity_eci_km_s.some((x) => x !== 0),
    'Velocity vector cannot be zero',
  ),
);

export type StateVectorInput = S.InferSchemaType<typeof stateVectorSchema>;
