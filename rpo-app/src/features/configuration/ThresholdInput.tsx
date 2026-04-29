import { type ChangeEvent, useMemo, useState } from 'react';

import { match } from '@railway-ts/pipelines/result';
import { validate } from '@railway-ts/pipelines/schema';

import { proximityConfigSchema, ROE_THRESHOLD_MIN } from '@/schemas/proximityConfig';
import { selectProximityConfig, usePlanner } from '@/stores/planner';
import { FieldErrorContext } from '@/ui/FieldErrorContext';
import { Input } from '@/ui/Input';
import { getEngineConstants } from '@/wasm/constants';

export function ThresholdInput() {
  const setConfig = usePlanner((s) => s.setProximityConfig);
  const schema = useMemo(() => proximityConfigSchema(), []);
  const {
    roe_threshold_default: defaultThreshold,
    linearization_perturbation_bound: maxThreshold,
  } = getEngineConstants();
  const [draft, setDraft] = useState<string>(() =>
    String(selectProximityConfig(usePlanner.getState().proximityConfig).roe_threshold),
  );
  const [error, setError] = useState<string | undefined>(undefined);

  const handleChange = (e: ChangeEvent<HTMLInputElement>) => {
    const raw = e.target.value;
    setDraft(raw);

    if (raw.trim() === '') {
      setError('δr/r threshold is required');
      return;
    }

    const parsed = Number(raw);
    match(validate({ roe_threshold: parsed }, schema), {
      ok: (config) => {
        setConfig(config);
        setError(undefined);
      },
      err: (errors) => {
        setError(errors[0]?.message ?? 'invalid threshold');
      },
    });
  };

  return (
    <div className="flex flex-col gap-1">
      <div className="flex flex-wrap items-baseline gap-3">
        <label
          htmlFor="roe-threshold"
          className="text-xs font-medium tracking-wide text-text-muted uppercase"
        >
          δr/r threshold
        </label>
        <FieldErrorContext value={error}>
          <Input
            id="roe-threshold"
            type="number"
            min={ROE_THRESHOLD_MIN}
            max={maxThreshold}
            step="0.0001"
            value={draft}
            onChange={handleChange}
            className="w-32"
          />
        </FieldErrorContext>
        {!error ? (
          <span className="font-mono text-xs text-text-dim">
            default {defaultThreshold} · D'Amico §2.3.4
          </span>
        ) : null}
      </div>
      {error ? (
        <span role="alert" className="font-mono text-xs text-signal-abort">
          {error}
        </span>
      ) : null}
    </div>
  );
}
