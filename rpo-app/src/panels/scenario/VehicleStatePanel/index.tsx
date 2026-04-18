import { type ChangeEvent, type DragEvent, useState } from 'react';

import { match } from '@railway-ts/pipelines/result';

import { Panel } from '@/components/primitives/Panel';
import { type Vehicle, VEHICLE_LABELS } from '@/domain/vehicle';
import { EMPTY_SLOT, useScenario, type VehicleStateSlot } from '@/stores/scenario';

import { DropZone } from './DropZone';
import { LoadedPreview } from './LoadedPreview';
import { loadStateVectorFromFile } from './loadStateVector';

type VehicleStatePanelProps = {
  vehicle: Vehicle;
};

export function VehicleStatePanel({ vehicle }: VehicleStatePanelProps) {
  const slot = useScenario((s) => (vehicle === 'chief' ? s.chiefState : s.deputyState));
  const setSlot = useScenario((s) => (vehicle === 'chief' ? s.setChiefState : s.setDeputyState));
  const [dragActive, setDragActive] = useState(false);

  const handleFile = async (file: File) => {
    const result = await loadStateVectorFromFile(file);
    setSlot(
      match(result, {
        ok: (vector): VehicleStateSlot => ({ status: 'loaded', fileName: file.name, vector }),
        err: (message): VehicleStateSlot => ({ status: 'error', message }),
      }),
    );
  };

  const handleChange = (e: ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file) void handleFile(file);
    e.target.value = '';
  };

  const handleDrop = (e: DragEvent<HTMLLabelElement>) => {
    e.preventDefault();
    setDragActive(false);
    const file = e.dataTransfer.files?.[0];
    if (file) void handleFile(file);
  };

  const handleClear = () => setSlot(EMPTY_SLOT);

  return (
    <Panel title={VEHICLE_LABELS[vehicle]} subtitle="ECI J2000 state">
      {slot.status === 'loaded' ? (
        <LoadedPreview slot={slot} onClear={handleClear} />
      ) : (
        <DropZone
          vehicle={vehicle}
          dragActive={dragActive}
          onDragOver={(e) => {
            e.preventDefault();
            setDragActive(true);
          }}
          onDragLeave={() => setDragActive(false)}
          onDrop={handleDrop}
          onChange={handleChange}
        />
      )}
      {slot.status === 'error' ? (
        <p role="alert" className="mt-2 font-mono text-xs text-signal-abort">
          {slot.message}
        </p>
      ) : null}
    </Panel>
  );
}
