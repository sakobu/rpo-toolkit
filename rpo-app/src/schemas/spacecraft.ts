import * as S from '@railway-ts/pipelines/schema';

export const PRESET_NAMES = ['6U CubeSat', 'Servicer 500kg', 'Custom'] as const;
export type PresetName = (typeof PRESET_NAMES)[number];

// Physical bounds — caps catch unit typos without constraining realistic missions.
export const MASS_MIN_KG = 0.1;
export const MASS_MAX_KG = 50_000;
export const DRAG_AREA_MIN_M2 = 0.001;
export const DRAG_AREA_MAX_M2 = 1000;
export const SRP_AREA_MIN_M2 = 0.001;
export const SRP_AREA_MAX_M2 = 1000;
export const CD_MIN = 1.0;
export const CD_MAX = 4.0;
export const CR_MIN = 1.0;
export const CR_MAX = 2.0;

export const spacecraftSchema = S.object({
  preset: S.required(S.stringEnum<PresetName>([...PRESET_NAMES])),
  mass_kg: S.required(
    S.chain(
      S.parseNumber(),
      S.between(MASS_MIN_KG, MASS_MAX_KG, `Mass must be ${MASS_MIN_KG}–${MASS_MAX_KG} kg`),
    ),
  ),
  drag_area_m2: S.required(
    S.chain(
      S.parseNumber(),
      S.between(
        DRAG_AREA_MIN_M2,
        DRAG_AREA_MAX_M2,
        `Drag area must be ${DRAG_AREA_MIN_M2}–${DRAG_AREA_MAX_M2} m²`,
      ),
    ),
  ),
  cd: S.required(
    S.chain(S.parseNumber(), S.between(CD_MIN, CD_MAX, `Cd must be ${CD_MIN}–${CD_MAX}`)),
  ),
  srp_area_m2: S.required(
    S.chain(
      S.parseNumber(),
      S.between(
        SRP_AREA_MIN_M2,
        SRP_AREA_MAX_M2,
        `SRP area must be ${SRP_AREA_MIN_M2}–${SRP_AREA_MAX_M2} m²`,
      ),
    ),
  ),
  cr: S.required(
    S.chain(S.parseNumber(), S.between(CR_MIN, CR_MAX, `Cr must be ${CR_MIN}–${CR_MAX}`)),
  ),
});

export type SpacecraftConfig = S.InferSchemaType<typeof spacecraftSchema>;

type PresetValues = Omit<SpacecraftConfig, 'preset'>;

export const PRESETS = {
  '6U CubeSat': { mass_kg: 12, drag_area_m2: 0.06, cd: 2.2, srp_area_m2: 0.06, cr: 1.5 },
  'Servicer 500kg': { mass_kg: 500, drag_area_m2: 1.0, cd: 2.2, srp_area_m2: 1.0, cr: 1.5 },
} as const satisfies Record<Exclude<PresetName, 'Custom'>, PresetValues>;

export const NUMERIC_FIELDS: readonly (keyof PresetValues)[] = [
  'mass_kg',
  'drag_area_m2',
  'cd',
  'srp_area_m2',
  'cr',
] as const;

export const FIELD_LABELS: Record<keyof PresetValues, string> = {
  mass_kg: 'Mass (kg)',
  drag_area_m2: 'Drag area (m²)',
  cd: 'Cd',
  srp_area_m2: 'SRP area (m²)',
  cr: 'Cr',
};

export const FIELD_BOUNDS: Record<keyof PresetValues, { min: number; max: number }> = {
  mass_kg: { min: MASS_MIN_KG, max: MASS_MAX_KG },
  drag_area_m2: { min: DRAG_AREA_MIN_M2, max: DRAG_AREA_MAX_M2 },
  cd: { min: CD_MIN, max: CD_MAX },
  srp_area_m2: { min: SRP_AREA_MIN_M2, max: SRP_AREA_MAX_M2 },
  cr: { min: CR_MIN, max: CR_MAX },
};
