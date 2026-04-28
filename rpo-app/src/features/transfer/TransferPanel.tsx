import { useEffect, useMemo, useState } from 'react';
import { useNavigate } from 'react-router';
import { ArrowRight } from 'lucide-react';

import { match } from '@railway-ts/pipelines/result';
import { useForm } from '@railway-ts/use-form';

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
import { selectProximityConfig, selectTransferSubSurface, usePlanner } from '@/stores/planner';
import { Callout } from '@/ui/Callout';
import { Caps } from '@/ui/Caps';
import { FieldErrorContext } from '@/ui/FieldErrorContext';
import { FormField } from '@/ui/FormField';
import { Input } from '@/ui/Input';
import { SegControl } from '@/ui/SegControl';
import { Slider } from '@/ui/Slider';
import { withBlur } from '@/utils/blur';
import { formatDurationShort } from '@/utils/format';
import { computeTransfer } from '@/wasm/transfer';

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

type ComputedTransfer =
  | { kind: 'idle' }
  | { kind: 'error'; message: string }
  | { kind: 'ok'; transfer: TransferResult; enrichment: EnrichmentSuggestion | null };

export function TransferPanel() {
  const navigate = useNavigate();
  const transfer = usePlanner((s) => s.transfer);
  const enrichment = usePlanner((s) => s.enrichment);
  const safetyRequirements = usePlanner((s) => s.safetyRequirements);
  const proximityConfigRaw = usePlanner((s) => s.proximityConfig);
  const chiefState = useConfig((s) => s.chiefState);
  const deputyState = useConfig((s) => s.deputyState);

  const schema = useMemo(() => lambertRequestSchema(), []);

  const form = useForm<LambertFormValues>(schema, {
    initialValues: { ...LAMBERT_REQUEST_DEFAULTS },
    validationMode: 'live',
  });

  const {
    lambert_tof_s,
    direction,
    revolutions,
    perch_mode,
    perch_offset_km,
    perch_da,
    perch_dlambda,
    perch_dex,
    perch_dey,
    perch_dix,
    perch_diy,
  } = form.values;
  const formIsValid = form.isValid;

  // Lambert is sync via WASM (microseconds), so the candidate transfer is a
  // pure function of form values + upstream config. Derive it via useMemo;
  // a small effect below mirrors the result into the planner store for the
  // 3D viewport and other consumers.
  const computed = useMemo<ComputedTransfer>(() => {
    if (!formIsValid) return { kind: 'idle' };
    if (chiefState.status !== 'loaded' || deputyState.status !== 'loaded') {
      return { kind: 'error', message: 'chief and deputy must be loaded' };
    }
    const lambertConfig: LambertConfig = { direction, revolutions };
    const result = computeTransfer({
      chief_eci: chiefState.vector,
      deputy_eci: deputyState.vector,
      perch: buildPerch(form.values),
      proximity: selectProximityConfig(usePlanner.getState()),
      lambert_tof_s,
      lambert_config: lambertConfig,
      ...(safetyRequirements ? { safety_requirements: safetyRequirements } : {}),
    });
    return match(result, {
      ok: ({ transfer: tr, enrichment: enr }): ComputedTransfer => ({
        kind: 'ok',
        transfer: tr,
        enrichment: enr,
      }),
      err: (e): ComputedTransfer => ({ kind: 'error', message: e.message }),
    });
    // form.values is captured by buildPerch; the listed scalars are the
    // change-detection surface. proximityConfigRaw drives reactivity on the
    // selector default-fallback inside the memo.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [
    formIsValid,
    chiefState,
    deputyState,
    safetyRequirements,
    proximityConfigRaw,
    lambert_tof_s,
    direction,
    revolutions,
    perch_mode,
    perch_offset_km,
    perch_da,
    perch_dlambda,
    perch_dex,
    perch_dey,
    perch_dix,
    perch_diy,
  ]);

  const lambertError = computed.kind === 'error' ? computed.message : null;

  useEffect(() => {
    if (computed.kind === 'ok') {
      usePlanner.getState().setTransfer(computed.transfer, computed.enrichment);
    } else {
      usePlanner.getState().clearTransfer();
    }
  }, [computed]);

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

  // Sub-surface conics are non-physical — block ACCEPT TRANSFER until the
  // user adjusts their inputs (TOF, revolutions, direction) enough to lift
  // the transfer ellipse's perigee above `MIN_PERIAPSIS_ALTITUDE_KM`. The
  // arc and Δv readouts still render so the user can see why their inputs
  // are bad.
  const subSurface = usePlanner(selectTransferSubSurface);
  const canAccept = transfer !== null && lambertError === null && subSurface === null;
  const acceptDisabledReason =
    subSurface !== null
      ? 'transfer below 200 km altitude floor — adjust tof, revolutions, or direction'
      : undefined;

  return (
    <div className="flex flex-col gap-2.5">
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
        <div className="flex gap-0.5 text-[10px]">
          {DIRECTION_VALUES.map((v) => (
            <SegControl
              key={v}
              active={form.values.direction === v}
              onClick={() => {
                form.setFieldValue('direction', v);
                form.setFieldTouched('direction');
              }}
            >
              {DIRECTION_LABELS[v]}
            </SegControl>
          ))}
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

      <DeltaVReadout transfer={transfer} enrichment={enrichment} />

      <button
        type="button"
        onClick={withBlur(handleAccept)}
        disabled={!canAccept}
        aria-label={acceptDisabledReason}
        title={acceptDisabledReason}
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
        <Callout tone="abort">
          <div className="flex flex-col gap-1">
            <span className="tracking-wider uppercase">transfer unavailable</span>
            <span className="text-[10px] lowercase">{lambertError}</span>
          </div>
        </Callout>
      ) : null}
    </div>
  );
}
