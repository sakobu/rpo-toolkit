import { useEffect, useMemo, useRef, useState } from 'react';
import { useNavigate } from 'react-router';
import { ArrowRight, TriangleAlert } from 'lucide-react';

import { match } from '@railway-ts/pipelines/result';
import { useForm, useFormAutoSubmission } from '@railway-ts/use-form';

import type { EnrichmentSuggestion, LambertConfig, PerchGeometry, TransferResult } from 'rpo-wasm';

import {
  DIRECTION_LABELS,
  DIRECTION_VALUES,
  LAMBERT_REQUEST_DEFAULTS,
  LAMBERT_TOF_MAX_S,
  LAMBERT_TOF_MIN_S,
  type LambertFormValues,
  lambertRequestSchema,
  PERCH_MODE_LABELS,
  PERCH_MODE_VALUES,
  PERCH_OFFSET_MAX_KM,
  PERCH_OFFSET_MIN_KM,
  QNS_COMPONENTS,
  QNS_LABELS,
  REVOLUTIONS_MAX,
  REVOLUTIONS_MIN,
} from '@/schemas/lambertRequest';
import { useConfig } from '@/stores/configuration';
import { selectProximityConfig, usePlanner } from '@/stores/planner';
import { Caps } from '@/ui/Caps';
import { FieldErrorContext } from '@/ui/FieldErrorContext';
import { FormField } from '@/ui/FormField';
import { Input } from '@/ui/Input';
import { SegControl } from '@/ui/SegControl';
import { Slider } from '@/ui/Slider';
import { withBlur } from '@/utils/blur';
import { formatDurationShort } from '@/utils/format';
import { computeTransfer } from '@/ws/transfer';

import { DeltaVReadout } from './DeltaVReadout';

function buildPerch(values: LambertFormValues): PerchGeometry {
  switch (values.perch_mode) {
    case 'v_bar':
      return { v_bar: { along_track_km: values.perch_offset_km } };
    case 'r_bar':
      return { r_bar: { radial_km: values.perch_offset_km } };
    case 'custom':
      return {
        custom: {
          da: values.perch_da,
          dlambda: values.perch_dlambda,
          dex: values.perch_dex,
          dey: values.perch_dey,
          dix: values.perch_dix,
          diy: values.perch_diy,
        },
      };
  }
}

export function TransferPanel() {
  const navigate = useNavigate();
  const transfer = usePlanner((s) => s.transfer);
  const enrichment = usePlanner((s) => s.enrichment);
  // Lambert failures live in local state, not on the form: using
  // `form.setServerErrors` would drop `form.isValid` to false and freeze
  // `useFormAutoSubmission` (it only fires on valid+dirty), so the user would
  // never be able to edit their way out of a bad TOF/revs combo.
  const [lambertError, setLambertError] = useState<string | null>(null);

  const schema = useMemo(() => lambertRequestSchema(), []);

  const form = useForm<LambertFormValues>(schema, {
    initialValues: { ...LAMBERT_REQUEST_DEFAULTS },
    validationMode: 'live',
    onSubmit: async (values) => {
      const { chiefState, deputyState } = useConfig.getState();
      const plannerState = usePlanner.getState();
      const proximityConfig = selectProximityConfig(plannerState);
      const { safetyRequirements } = plannerState;

      if (chiefState.status !== 'loaded' || deputyState.status !== 'loaded') {
        setLambertError('chief and deputy must be loaded');
        usePlanner.getState().clearTransfer();
        return;
      }

      const lambertConfig: LambertConfig = {
        direction: values.direction,
        revolutions: values.revolutions,
      };

      const result = await computeTransfer({
        chief_eci: chiefState.vector,
        deputy_eci: deputyState.vector,
        perch: buildPerch(values),
        proximity: proximityConfig,
        lambert_tof_s: values.lambert_tof_s,
        lambert_config: lambertConfig,
        ...(safetyRequirements ? { safety_requirements: safetyRequirements } : {}),
      });

      match(result, {
        ok: ({ transfer: tr, enrichment: enr }) => {
          usePlanner.getState().setTransfer(tr, enr);
          setLambertError(null);
        },
        err: (e) => {
          setLambertError(e.message);
          usePlanner.getState().clearTransfer();
        },
      });
    },
  });

  useFormAutoSubmission(form, 500);

  // Safety edits upstream must re-solve — otherwise the rendered Δv would
  // reflect an outdated enrichment (or its absence) after the user edits the
  // mission header. Submitting only on valid+not-submitting keeps this cheap
  // and avoids clobbering an in-flight request. `form` has unstable identity
  // across renders, so thread it through a ref to subscribe exactly once.
  const formRef = useRef(form);
  useEffect(() => {
    formRef.current = form;
  });

  useEffect(() => {
    const unsub = usePlanner.subscribe(
      (s) => s.safetyRequirements,
      () => {
        const f = formRef.current;
        if (f.isValid && !f.isSubmitting) void f.handleSubmit();
      },
    );
    return unsub;
  }, []);

  // Discrete toggles (Direction, Perch reference) need explicit submits.
  // `useFormAutoSubmission` gates on `isDirty`, which compares current values
  // against `initialValues` — clicking back to an initial value leaves the
  // form clean and the submit is skipped, freezing Δv on the previous result.
  // A handleSubmit() call inside the onClick doesn't work either: use-form
  // reads values from `formStateRef.current`, which only syncs to the new
  // state in a useEffect. Firing the submit from our own effect guarantees
  // the ref is current by the time handleSubmit runs.
  //
  // A value-based ref (not a did-mount flag) is required: StrictMode
  // double-invokes effects, so a bare `didMountRef` would fire a submit on
  // the second invocation — before the WebSocket connects.
  const lastSubmittedToggleRef = useRef({
    direction: form.values.direction,
    perch_mode: form.values.perch_mode,
  });
  useEffect(() => {
    if (
      lastSubmittedToggleRef.current.direction === form.values.direction &&
      lastSubmittedToggleRef.current.perch_mode === form.values.perch_mode
    ) {
      return;
    }
    lastSubmittedToggleRef.current = {
      direction: form.values.direction,
      perch_mode: form.values.perch_mode,
    };
    const f = formRef.current;
    if (f.isValid && !f.isSubmitting) void f.handleSubmit();
  }, [form.values.direction, form.values.perch_mode]);

  // Two-beat confirm on fallback enrichment. Keying the pending state to the
  // (enrichment, transfer) pair lets `confirmPending` derive to false when the
  // solver re-runs — no setState-in-effect needed to reset it.
  const [pendingFor, setPendingFor] = useState<{
    enrichment: EnrichmentSuggestion | null;
    transfer: TransferResult | null;
  } | null>(null);

  const confirmPending =
    pendingFor !== null && pendingFor.enrichment === enrichment && pendingFor.transfer === transfer;

  useEffect(() => {
    if (!confirmPending) return;
    const timer = setTimeout(() => setPendingFor(null), 5000);
    return () => clearTimeout(timer);
  }, [confirmPending]);

  const needsConfirm = enrichment?.perch.status === 'fallback';

  const handleAccept = () => {
    if (transfer === null) return;
    if (needsConfirm && !confirmPending) {
      setPendingFor({ enrichment, transfer });
      return;
    }
    usePlanner.getState().acceptTransfer();
    void navigate('/proximity');
  };

  const canAccept = transfer !== null && !form.isSubmitting && lambertError === null;

  // Multi-rev (revolutions > 0) routes through nyx Izzo with
  // `TransferKind::NRevs`, which has no long-way variant; direction is
  // dropped server-side. See `rpo-nyx/src/lambert.rs` and the
  // `multi_rev_ignores_direction` regression test.
  const directionLocked = form.values.revolutions > 0;

  return (
    <form onSubmit={(e) => void form.handleSubmit(e)} className="flex flex-col gap-2.5">
      <Caps>lambert transfer</Caps>

      <FormField label="Perch reference" name="perch_mode" form={form} idPrefix="lambert">
        <div className="flex gap-0.5 text-[10px]">
          {PERCH_MODE_VALUES.map((v) => (
            <SegControl
              key={v}
              active={form.values.perch_mode === v}
              onClick={() => {
                form.setFieldValue('perch_mode', v);
                form.setFieldTouched('perch_mode');
              }}
            >
              {PERCH_MODE_LABELS[v]}
            </SegControl>
          ))}
        </div>
      </FormField>

      {form.values.perch_mode === 'custom' ? (
        <div className="flex flex-col gap-1.5">
          <div className="flex items-center justify-between">
            <Caps>custom roe (δα · dimensionless)</Caps>
          </div>
          <div className="grid grid-cols-3 gap-1.5">
            {QNS_COMPONENTS.map((c) => (
              <FormField
                key={c}
                label={QNS_LABELS[c]}
                name={`perch_${c}` as const}
                form={form}
                idPrefix="lambert"
              >
                <Input
                  type="number"
                  step="any"
                  {...form.getFieldProps(`perch_${c}` as const)}
                  id={`lambert-perch_${c}`}
                />
              </FormField>
            ))}
          </div>
        </div>
      ) : (
        <FieldErrorContext value={form.getFieldError('perch_offset_km')}>
          <Slider
            label={`offset (${form.values.perch_mode === 'v_bar' ? 'along-track' : 'radial'})`}
            readout={`${form.values.perch_offset_km.toFixed(1)} km`}
            min={PERCH_OFFSET_MIN_KM}
            max={PERCH_OFFSET_MAX_KM}
            step={0.1}
            {...form.getSliderProps('perch_offset_km')}
          />
        </FieldErrorContext>
      )}

      <FieldErrorContext value={form.getFieldError('lambert_tof_s')}>
        <Slider
          label="lambert tof"
          readout={formatDurationShort(form.values.lambert_tof_s)}
          min={LAMBERT_TOF_MIN_S}
          max={LAMBERT_TOF_MAX_S}
          step={60}
          {...form.getSliderProps('lambert_tof_s')}
        />
      </FieldErrorContext>

      <FormField label="Direction" name="direction" form={form} idPrefix="lambert">
        <div className="flex flex-col gap-1">
          <div className="flex gap-0.5 text-[10px]">
            {DIRECTION_VALUES.map((v) => (
              <SegControl
                key={v}
                active={form.values.direction === v}
                disabled={directionLocked}
                onClick={() => {
                  form.setFieldValue('direction', v);
                  form.setFieldTouched('direction');
                }}
              >
                {DIRECTION_LABELS[v]}
              </SegControl>
            ))}
          </div>
          {directionLocked ? (
            <span className="font-mono text-[9px] tracking-wide text-text-dim normal-case">
              multi-rev uses the short-way branch
            </span>
          ) : null}
        </div>
      </FormField>

      <FieldErrorContext value={form.getFieldError('revolutions')}>
        <Slider
          label="revolutions"
          readout={`${form.values.revolutions} rev`}
          min={REVOLUTIONS_MIN}
          max={REVOLUTIONS_MAX}
          step={1}
          {...form.getSliderProps('revolutions')}
        />
      </FieldErrorContext>

      <DeltaVReadout transfer={transfer} enrichment={enrichment} submitting={form.isSubmitting} />

      <button
        type="button"
        onClick={withBlur(handleAccept)}
        disabled={!canAccept}
        className={`mt-1 flex cursor-pointer items-center justify-center gap-1.5 rounded-xs border px-2 py-1.5 font-mono text-[10px] tracking-wider uppercase transition-colors disabled:cursor-not-allowed disabled:border-border disabled:bg-surface-2 disabled:text-text-dim disabled:hover:bg-surface-2 ${
          confirmPending
            ? 'border-signal-hold bg-signal-hold-dim text-signal-hold hover:bg-signal-hold/10'
            : 'border-accent bg-accent-dim text-accent hover:bg-accent/10'
        }`}
      >
        {confirmPending ? 'confirm: accept fallback' : 'accept transfer'}
        <ArrowRight size={11} strokeWidth={1.75} />
      </button>

      {lambertError !== null ? (
        <span
          role="alert"
          className="flex min-h-4 items-center gap-1 font-mono text-[10px] text-signal-abort"
        >
          <TriangleAlert size={10} strokeWidth={1.75} />
          {lambertError}
        </span>
      ) : (
        <span aria-hidden className="min-h-4" />
      )}
    </form>
  );
}
