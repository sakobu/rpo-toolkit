import { useForm } from '@railway-ts/use-form';

import type { SafetyRequirements } from 'rpo-wasm';

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
import { Input } from '@/ui/Input';
import { Select } from '@/ui/Select';
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
  const form = useForm<SafetyRequirementsFormValues>(safetyRequirementsSchema, {
    initialValues: initialValues(initial),
    onSubmit: (values) => {
      onApply({
        min_separation_km: values.min_separation_km,
        alignment: values.alignment,
      });
    },
  });

  return (
    <form onSubmit={(e) => void form.handleSubmit(e)} className="mt-2 flex flex-col gap-2">
      <FormField
        label="Min R/C separation (km)"
        name="min_separation_km"
        form={form}
        idPrefix="safety"
      >
        <Input
          type="number"
          step="any"
          min={MIN_SEPARATION_LOWER_KM}
          max={MIN_SEPARATION_UPPER_KM}
          {...form.getFieldProps('min_separation_km')}
          id="safety-min_separation_km"
        />
      </FormField>
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
