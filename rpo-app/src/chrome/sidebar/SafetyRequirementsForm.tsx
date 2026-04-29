import { useForm } from '@railway-ts/use-form';

import type { SafetyRequirements } from 'rpo-wasm';

import { useAchievableCap } from '@/hooks/useAchievableCap';
import {
  ALIGNMENT_LABELS,
  ALIGNMENT_VALUES,
  DEFAULT_SAFETY_REQUIREMENTS,
  MIN_SEPARATION_LOWER_KM,
  MIN_SEPARATION_STEP_KM,
  type SafetyRequirementsFormValues,
  safetyRequirementsSchema,
} from '@/schemas/safetyRequirements';
import { Button } from '@/ui/Button';
import { FormField } from '@/ui/FormField';
import { Select } from '@/ui/Select';
import { Slider } from '@/ui/Slider';
import { withBlur } from '@/utils/blur';

type Props = {
  initial: SafetyRequirements | null;
  onApply: (requirements: SafetyRequirements) => void;
  onCancel: () => void;
};

function initialValues(
  initial: SafetyRequirements | null,
  cap: number | null,
): SafetyRequirementsFormValues {
  const base = initial ?? DEFAULT_SAFETY_REQUIREMENTS;
  // Clamp prior values to the current orbit's cap so a chief change can never
  // leave the form holding a now-infeasible value.
  const clampedSep = cap === null ? base.min_separation_km : Math.min(base.min_separation_km, cap);
  return {
    min_separation_km: clampedSep,
    alignment: initial?.alignment ?? DEFAULT_SAFETY_REQUIREMENTS.alignment,
  };
}

export function SafetyRequirementsForm({ initial, onApply, onCancel }: Props) {
  const achievableCap = useAchievableCap();
  const chiefLoaded = achievableCap !== null;

  const form = useForm<SafetyRequirementsFormValues>(safetyRequirementsSchema, {
    initialValues: initialValues(initial, achievableCap),
    onSubmit: (values) => {
      onApply({
        min_separation_km: values.min_separation_km,
        alignment: values.alignment,
      });
    },
  });

  return (
    <form onSubmit={(e) => void form.handleSubmit(e)} className="mt-2 flex flex-col gap-2">
      <Slider
        label="Min R/C separation (km)"
        readout={
          <div className="text-right">
            <div>{form.values.min_separation_km.toFixed(2)} km</div>
            <div className="text-[9px] text-text-dim">
              {chiefLoaded
                ? `max for this orbit: ${achievableCap.toFixed(1)} km`
                : 'load chief to set safety requirements'}
            </div>
          </div>
        }
        min={MIN_SEPARATION_LOWER_KM}
        max={chiefLoaded ? achievableCap : MIN_SEPARATION_LOWER_KM}
        step={MIN_SEPARATION_STEP_KM}
        disabled={!chiefLoaded}
        {...form.getSliderProps('min_separation_km')}
        id="safety-min_separation_km"
      />
      <FormField label="Alignment" name="alignment" form={form} idPrefix="safety">
        <Select
          {...form.getSelectFieldProps('alignment')}
          id="safety-alignment"
          disabled={!chiefLoaded}
        >
          {ALIGNMENT_VALUES.map((v) => (
            <option key={v} value={v}>
              {ALIGNMENT_LABELS[v]}
            </option>
          ))}
        </Select>
      </FormField>
      <div className="flex items-center justify-end gap-2 pt-1">
        <Button variant="ghost" type="button" onClick={withBlur(onCancel)}>
          cancel
        </Button>
        <Button variant="primary" type="submit" disabled={!form.isValid || !chiefLoaded}>
          apply
        </Button>
      </div>
    </form>
  );
}
