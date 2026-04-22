import { useState } from 'react';
import { Pencil, Plus, X } from 'lucide-react';
import { useShallow } from 'zustand/react/shallow';

import { EditConfigButton } from '@/chrome/EditConfigButton';
import { SafetyRequirementsForm } from '@/features/safety-requirements/SafetyRequirementsForm';
import {
  ALIGNMENT_LABELS,
  type AlignmentValue,
  DEFAULT_SAFETY_REQUIREMENTS,
} from '@/schemas/safetyRequirements';
import { useConfig } from '@/stores/configuration';
import { useMission } from '@/stores/mission';
import { Caps } from '@/ui/Caps';
import { InlineActionButton } from '@/ui/InlineActionButton';
import { KV } from '@/ui/KV';

export function MissionHeader() {
  const { chiefPreset, deputyPreset } = useConfig(
    useShallow((s) => ({
      chiefPreset: s.chiefConfig?.values.preset,
      deputyPreset: s.deputyConfig?.values.preset,
    })),
  );
  const classification = useMission((s) => s.classification);
  const threshold = useMission((s) => s.proximityConfig.roe_threshold);
  const safetyRequirements = useMission((s) => s.safetyRequirements);
  const setSafetyRequirements = useMission((s) => s.setSafetyRequirements);
  const [editingSafety, setEditingSafety] = useState(false);

  const phase = classification.status === 'ok' ? classification.phase : null;
  const phaseData = phase && ('proximity' in phase ? phase.proximity : phase.far_field);
  const separationKm = phaseData?.separation_km ?? null;
  const deltaROverR = phaseData?.delta_r_over_r ?? null;

  const chiefName = chiefPreset ?? 'Chief vehicle';
  const deputyName = deputyPreset ?? 'Deputy vehicle';

  const alignmentValue: AlignmentValue =
    safetyRequirements?.alignment ?? DEFAULT_SAFETY_REQUIREMENTS.alignment;

  return (
    <div className="border-b border-border px-4 py-3">
      <div className="mb-2 flex items-center justify-between gap-2">
        <Caps>mission</Caps>
        <EditConfigButton />
      </div>
      <div className="flex flex-col gap-1 rounded-xs border border-border bg-surface-2 px-2 py-1.5">
        <Caps>spacecraft</Caps>
        <KV k="chief" v={chiefName} />
        <KV k="deputy" v={deputyName} />
      </div>
      <div className="mt-1.5 flex flex-col gap-1 rounded-xs border border-border bg-surface-2 px-2 py-1.5">
        <Caps>classification</Caps>
        <KV k="separation" v={separationKm !== null ? `${separationKm.toFixed(1)} km` : '—'} />
        <KV k="δr/r" v={deltaROverR !== null ? deltaROverR.toExponential(2) : '—'} />
        <KV k="δr/r threshold" v={String(threshold)} />
      </div>
      <div className="mt-1.5 flex flex-col gap-1 rounded-xs border border-border bg-surface-2 px-2 py-1.5">
        <div className="flex items-center justify-between">
          <Caps>safety</Caps>
          {safetyRequirements === null ? (
            <InlineActionButton
              icon={<Plus size={9} strokeWidth={1.75} />}
              label="set"
              onClick={() => setEditingSafety(true)}
            />
          ) : (
            <div className="flex items-center gap-1">
              <InlineActionButton
                icon={<Pencil size={9} strokeWidth={1.75} />}
                label="edit"
                onClick={() => setEditingSafety(true)}
                aria-label="edit safety requirements"
              />
              <InlineActionButton
                icon={<X size={9} strokeWidth={1.75} />}
                label="clear"
                tone="abort"
                onClick={() => {
                  setSafetyRequirements(null);
                  setEditingSafety(false);
                }}
                aria-label="clear safety requirements"
              />
            </div>
          )}
        </div>
        {!editingSafety && safetyRequirements === null && (
          <span className="font-mono text-[9px] tracking-wide text-text-dim">
            not set · baseline only
          </span>
        )}
        {!editingSafety && safetyRequirements !== null && (
          <>
            <KV k="min R/C" v={`${safetyRequirements.min_separation_km} km`} />
            <KV k="alignment" v={ALIGNMENT_LABELS[alignmentValue].toLowerCase()} />
          </>
        )}
        {editingSafety && (
          <SafetyRequirementsForm
            initial={safetyRequirements}
            onApply={(r) => {
              setSafetyRequirements(r);
              setEditingSafety(false);
            }}
            onCancel={() => setEditingSafety(false)}
          />
        )}
      </div>
    </div>
  );
}
