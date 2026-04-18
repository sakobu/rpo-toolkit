import { type ChangeEvent, useEffect } from 'react';

import { useForm } from '@railway-ts/use-form';

import { type Vehicle, VEHICLE_LABELS } from '@/domain/vehicle';
import {
  FIELD_LABELS,
  NUMERIC_FIELDS,
  PRESET_NAMES,
  type PresetName,
  PRESETS,
  type SpacecraftConfig,
  spacecraftSchema,
} from '@/schemas/spacecraft';
import { useConfig } from '@/stores/configuration';
import { FormField } from '@/ui/FormField';
import { Input } from '@/ui/Input';
import { Panel } from '@/ui/Panel';
import { Select } from '@/ui/Select';
import { SpacecraftThumbnail } from '@/viewport3d/spacecraft/SpacecraftThumbnail';

type SpacecraftPanelProps = {
  vehicle: Vehicle;
};

const DEFAULT_PRESET: PresetName = 'Servicer 500kg';

const initialValues: SpacecraftConfig = {
  preset: DEFAULT_PRESET,
  ...PRESETS[DEFAULT_PRESET],
};

type NumericField = (typeof NUMERIC_FIELDS)[number];

function isNumericField(field: string): field is NumericField {
  return (NUMERIC_FIELDS as readonly string[]).includes(field);
}

function isPresetName(value: string): value is PresetName {
  return (PRESET_NAMES as readonly string[]).includes(value);
}

export function SpacecraftPanel({ vehicle }: SpacecraftPanelProps) {
  const form = useForm<SpacecraftConfig>(spacecraftSchema, {
    initialValues,
    validationMode: 'live',
    onFieldChange: (field, _value, values) => {
      if (field === 'preset') return;
      if (!isNumericField(field)) return;
      if (values.preset === 'Custom') return;
      const stored = PRESETS[values.preset][field];
      const current = values[field];
      const currentNum = typeof current === 'string' ? Number(current) : current;
      if (currentNum !== stored) {
        form.setFieldValue('preset', 'Custom');
      }
    },
  });

  const setChiefConfig = useConfig((s) => s.setChiefConfig);
  const setDeputyConfig = useConfig((s) => s.setDeputyConfig);
  const setter = vehicle === 'chief' ? setChiefConfig : setDeputyConfig;

  useEffect(() => {
    setter({ values: form.values, isValid: form.isValid });
  }, [form.values, form.isValid, setter]);

  const handlePresetChange = (e: ChangeEvent<HTMLSelectElement>) => {
    const name = e.target.value;
    if (!isPresetName(name)) return;
    form.setFieldValue('preset', name);
    if (name !== 'Custom') {
      form.setValues({ preset: name, ...PRESETS[name] });
    }
  };

  const prefixId = (name: keyof SpacecraftConfig) => `${vehicle}-${name}`;

  return (
    <Panel title={VEHICLE_LABELS[vehicle]} subtitle="spacecraft">
      <div className="grid gap-3">
        <FormField label="Preset" name="preset" form={form} idPrefix={vehicle}>
          <Select
            id={prefixId('preset')}
            name={prefixId('preset')}
            value={form.values.preset}
            onBlur={() => form.setFieldTouched('preset')}
            onChange={handlePresetChange}
          >
            {PRESET_NAMES.map((name) => (
              <option key={name} value={name}>
                {name}
              </option>
            ))}
          </Select>
        </FormField>

        <div className="grid grid-cols-2 gap-3">
          {NUMERIC_FIELDS.map((key) => (
            <FormField
              key={key}
              label={FIELD_LABELS[key]}
              name={key}
              form={form}
              idPrefix={vehicle}
            >
              <Input type="number" step="any" {...form.getFieldProps(key)} id={prefixId(key)} />
            </FormField>
          ))}
        </div>

        <div className="mt-2">
          <SpacecraftThumbnail vehicle={vehicle} config={form.values} />
        </div>
      </div>
    </Panel>
  );
}
