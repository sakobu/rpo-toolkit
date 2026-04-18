import * as S from '@railway-ts/pipelines/schema';

const km = S.chain(S.number('Must be a number'), S.finite('Must be finite'));
const vec3 = S.tupleOf(km, 3);

const baseSchema = S.object({
  epoch: S.required(S.chain(S.string('epoch must be a string'), S.nonEmpty('epoch is required'))),
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
