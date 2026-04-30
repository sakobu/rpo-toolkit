import { useEffect, useMemo } from 'react';
import { useNavigate } from 'react-router';
import { ArrowRight } from 'lucide-react';
import { useShallow } from 'zustand/react/shallow';

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
  | { kind: 'ok'; transfer: TransferResult; enrichment: EnrichmentSuggestion };

export function TransferPanel() {
  const navigate = useNavigate();
  const { transferSlot, safetyRequirements, proximityConfigRaw } = usePlanner(
    useShallow((s) => ({
      transferSlot: s.transferSlot,
      safetyRequirements: s.safetyRequirements,
      proximityConfigRaw: s.proximityConfig,
    })),
  );
  const chiefState = useConfig((s) => s.chiefState);
  const deputyState = useConfig((s) => s.deputyState);

  const schema = useMemo(() => lambertRequestSchema(), []);

  const form = useForm<LambertFormValues>(schema, {
    initialValues: { ...LAMBERT_REQUEST_DEFAULTS },
    validationMode: 'live',
  });

  const formIsValid = form.isValid;

  const computed = useMemo<ComputedTransfer>(() => {
    if (!formIsValid) return { kind: 'idle' };

    if (chiefState.status !== 'loaded' || deputyState.status !== 'loaded') {
      return { kind: 'error', message: 'chief and deputy must be loaded' };
    }

    const lambertConfig: LambertConfig = {
      direction: form.values.direction,
      revolutions: form.values.revolutions,
    };

    const result = computeTransfer({
      chief_eci: chiefState.vector,
      deputy_eci: deputyState.vector,
      perch: buildPerch(form.values),
      proximity: selectProximityConfig(proximityConfigRaw),
      lambert_tof_s: form.values.lambert_tof_s,
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
  }, [formIsValid, chiefState, deputyState, safetyRequirements, proximityConfigRaw, form.values]);

  const lambertError = computed.kind === 'error' ? computed.message : null;

  useEffect(() => {
    if (computed.kind === 'ok') {
      usePlanner.getState().setTransfer(computed.transfer, computed.enrichment);
    } else {
      usePlanner.getState().clearTransfer();
    }
  }, [computed]);

  const handleAccept = () => {
    if (transferSlot === null) return;
    usePlanner.getState().acceptTransfer();
    void navigate('/proximity');
  };

  const subSurface = usePlanner(selectTransferSubSurface);
  const canAccept = transferSlot !== null && lambertError === null && subSurface === null;
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

      <DeltaVReadout slot={transferSlot} />

      <button
        type="button"
        onClick={withBlur(handleAccept)}
        disabled={!canAccept}
        aria-label={acceptDisabledReason}
        title={acceptDisabledReason}
        className="mt-1 flex cursor-pointer items-center justify-center gap-1.5 rounded-xs border border-accent bg-accent-dim px-2 py-1.5 font-mono text-[10px] tracking-wider text-accent uppercase transition-colors hover:bg-accent/10 disabled:cursor-not-allowed disabled:border-border disabled:bg-surface-2 disabled:text-text-dim disabled:hover:bg-surface-2"
      >
        accept transfer
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
