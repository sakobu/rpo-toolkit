import { useForm } from '@railway-ts/use-form';

import type { SafetyRequirements } from 'rpo-wasm';

import { useAchievableCap } from '@/hooks/useAchievableCap';
import {
  ALIGNMENT_LABELS,
  ALIGNMENT_VALUES,
  DEFAULT_SAFETY_REQUIREMENTS,
  MIN_SEPARATION_LOWER_KM,
  MIN_SEPARATION_UPPER_KM,
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

function initialValues(initial: SafetyRequirements | null): SafetyRequirementsFormValues {
  if (initial === null) {
    return { ...DEFAULT_SAFETY_REQUIREMENTS };
  }
  return {
    min_separation_km: initial.min_separation_km,
    alignment: initial.alignment ?? DEFAULT_SAFETY_REQUIREMENTS.alignment,
  };
}

export function SafetyRequirementsForm({ initial, onApply, onCancel }: Props) {
  const achievableCap = useAchievableCap();

  const form = useForm<SafetyRequirementsFormValues>(safetyRequirementsSchema, {
    initialValues: initialValues(initial),
    onSubmit: (values) => {
      onApply({
        min_separation_km: values.min_separation_km,
        alignment: values.alignment,
      });
    },
  });

  const overlayLeftPct =
    achievableCap !== null
      ? ((achievableCap - MIN_SEPARATION_LOWER_KM) /
          (MIN_SEPARATION_UPPER_KM - MIN_SEPARATION_LOWER_KM)) *
        100
      : null;

  return (
    <form onSubmit={(e) => void form.handleSubmit(e)} className="mt-2 flex flex-col gap-2">
      <div className="relative">
        <Slider
          label="Min R/C separation (km)"
          readout={
            <div className="text-right">
              <div>{form.values.min_separation_km.toFixed(2)} km</div>
              <div className="text-[9px] text-text-dim">
                {achievableCap !== null
                  ? `safe to ~${achievableCap.toFixed(1)} km at this orbit`
                  : 'safe to —'}
              </div>
            </div>
          }
          min={MIN_SEPARATION_LOWER_KM}
          max={MIN_SEPARATION_UPPER_KM}
          step={0.1}
          {...form.getSliderProps('min_separation_km')}
          id="safety-min_separation_km"
        />
        {overlayLeftPct !== null && overlayLeftPct < 100 ? (
          <div
            aria-hidden
            className="pointer-events-none absolute right-0 bottom-1 h-1 rounded-xs bg-signal-hold-dim opacity-60"
            style={{ left: `${overlayLeftPct}%` }}
          />
        ) : null}
      </div>
      <FormField label="Alignment" name="alignment" form={form} idPrefix="safety">
        <Select {...form.getSelectFieldProps('alignment')} id="safety-alignment">
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
        <Button variant="primary" type="submit" disabled={!form.isValid}>
          apply
        </Button>
      </div>
    </form>
  );
}
