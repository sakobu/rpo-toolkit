import * as S from '@railway-ts/pipelines/schema';

export const PRESET_NAMES = ['6U CubeSat', 'Servicer 500kg', 'Custom'] as const;
export type PresetName = (typeof PRESET_NAMES)[number];

export const spacecraftSchema = S.object({
  preset: S.required(S.stringEnum<PresetName>([...PRESET_NAMES])),
  mass_kg: S.required(S.chain(S.parseNumber(), S.positive('Mass must be > 0'))),
  drag_area_m2: S.required(S.chain(S.parseNumber(), S.positive('Drag area must be > 0'))),
  cd: S.required(S.chain(S.parseNumber(), S.positive('Cd must be > 0'))),
  srp_area_m2: S.required(S.chain(S.parseNumber(), S.positive('SRP area must be > 0'))),
  cr: S.required(S.chain(S.parseNumber(), S.positive('Cr must be > 0'))),
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
