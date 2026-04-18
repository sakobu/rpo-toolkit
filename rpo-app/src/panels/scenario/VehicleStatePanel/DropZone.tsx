import type { ChangeEvent, DragEvent } from 'react';

import type { Vehicle } from '../vehicle';

type DropZoneProps = {
  vehicle: Vehicle;
  dragActive: boolean;
  onDragOver: (e: DragEvent<HTMLLabelElement>) => void;
  onDragLeave: () => void;
  onDrop: (e: DragEvent<HTMLLabelElement>) => void;
  onChange: (e: ChangeEvent<HTMLInputElement>) => void;
};

export function DropZone({
  vehicle,
  dragActive,
  onDragOver,
  onDragLeave,
  onDrop,
  onChange,
}: DropZoneProps) {
  return (
    <label
      htmlFor={`upload-${vehicle}`}
      className={`duration-fast flex cursor-pointer flex-col items-center justify-center rounded-sm border-2 border-dashed px-4 py-8 text-center transition-colors ${
        dragActive
          ? 'border-border-focus bg-surface-2'
          : 'border-border bg-surface-1 hover:bg-surface-2'
      }`}
      onDragOver={onDragOver}
      onDragLeave={onDragLeave}
      onDrop={onDrop}
    >
      <span className="text-sm text-text-muted">Drop state JSON or click to browse</span>
      <span className="mt-1 font-mono text-xs text-text-dim">
        {'{ epoch, position_eci_km, velocity_eci_km_s }'}
      </span>
      <input
        id={`upload-${vehicle}`}
        type="file"
        accept="application/json,.json"
        className="sr-only"
        onChange={onChange}
      />
    </label>
  );
}
