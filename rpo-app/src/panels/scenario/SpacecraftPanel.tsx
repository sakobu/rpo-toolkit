import { useEffect, type ChangeEvent } from 'react';
import { useForm } from '@railway-ts/use-form';
import { validate } from '@railway-ts/pipelines/schema';
import { isOk } from '@railway-ts/pipelines/result';

import { Panel } from '../../components/primitives/Panel';
import { Field } from '../../components/primitives/Field';
import { Input } from '../../components/primitives/Input';
import { Select } from '../../components/primitives/Select';
import { SpacecraftThumbnail } from '../../components/spacecraft/SpacecraftThumbnail';
import {
  spacecraftSchema,
  PRESETS,
  PRESET_NAMES,
  NUMERIC_FIELDS,
  FIELD_LABELS,
  type PresetName,
  type SpacecraftConfig,
} from '../../schemas/spacecraft';
import { useScenario } from '../../stores/scenario';

type Vehicle = 'chief' | 'deputy';

type SpacecraftPanelProps = {
  vehicle: Vehicle;
};

const VEHICLE_LABELS: Record<Vehicle, string> = {
  chief: 'Chief',
  deputy: 'Deputy',
};

const DEFAULT_PRESET: PresetName = '6U CubeSat';

const initialValues: SpacecraftConfig = {
  preset: DEFAULT_PRESET,
  ...PRESETS[DEFAULT_PRESET],
};

export function SpacecraftPanel({ vehicle }: SpacecraftPanelProps) {
  const form = useForm<SpacecraftConfig>(spacecraftSchema, {
    initialValues,
    validationMode: 'live',
  });

  const setChief = useScenario((s) => s.setChief);
  const setDeputy = useScenario((s) => s.setDeputy);
  const setter = vehicle === 'chief' ? setChief : setDeputy;

  const preset = form.values.preset;

  useEffect(() => {
    if (preset === 'Custom') return;
    const presetValues = PRESETS[preset];
    for (const key of NUMERIC_FIELDS) {
      const current = form.values[key];
      const stored = presetValues[key];
      if (typeof current === 'string' ? Number(current) !== stored : current !== stored) {
        form.setFieldValue('preset', 'Custom');
        return;
      }
    }
  }, [form, preset]);

  useEffect(() => {
    const result = validate(form.values, spacecraftSchema);
    setter(isOk(result) ? result.value : null);
  }, [form.values, setter]);

  const handlePresetChange = (e: ChangeEvent<HTMLSelectElement>) => {
    const name = e.target.value as PresetName;
    form.setFieldValue('preset', name);
    if (name !== 'Custom') {
      form.setValues(PRESETS[name]);
    }
  };

  const scopedId = (field: string) => `${vehicle}-${field}`;

  const presetInvalid = Boolean(form.getFieldError('preset'));
  const presetId = scopedId('preset');

  return (
    <Panel title={VEHICLE_LABELS[vehicle]} subtitle="spacecraft">
      <div className="grid gap-3">
        <Field label="Preset" htmlFor={presetId} error={form.getFieldError('preset')}>
          <Select
            id={presetId}
            name={presetId}
            value={form.values.preset}
            onBlur={() => form.setFieldTouched('preset')}
            onChange={handlePresetChange}
            invalid={presetInvalid}
          >
            {PRESET_NAMES.map((name) => (
              <option key={name} value={name}>
                {name}
              </option>
            ))}
          </Select>
        </Field>

        <div className="grid grid-cols-2 gap-3">
          {NUMERIC_FIELDS.map((key) => {
            const id = scopedId(key);
            const error = form.getFieldError(key);
            return (
              <Field key={key} label={FIELD_LABELS[key]} htmlFor={id} error={error}>
                <Input
                  type="number"
                  step="any"
                  invalid={Boolean(error)}
                  {...form.getFieldProps(key)}
                  id={id}
                  name={id}
                />
              </Field>
            );
          })}
        </div>

        <SpacecraftThumbnail vehicle={vehicle} config={form.values} />
      </div>
    </Panel>
  );
}
