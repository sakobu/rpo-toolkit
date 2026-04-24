import { type InputHTMLAttributes, type ReactNode } from 'react';

import { Caps } from './Caps';
import { useFieldError } from './FieldErrorContext';

type SliderProps = Omit<InputHTMLAttributes<HTMLInputElement>, 'type'> & {
  label: ReactNode;
  readout: ReactNode;
  // `min`/`max` are required so the component can compute the fill percentage.
  // `value` arrives via spread from `form.getSliderProps(...)` — the railway-ts
  // helper returns only id/name/type/value/onChange/onBlur.
  min: number;
  max: number;
};

export function Slider({ label, readout, min, max, value, className, ...rest }: SliderProps) {
  const numericValue = typeof value === 'number' ? value : Number(value ?? 0);
  const span = max - min;
  const pct = span > 0 ? Math.min(100, Math.max(0, ((numericValue - min) / span) * 100)) : 0;
  const error = useFieldError();
  const invalid = Boolean(error);
  const fillColor = invalid ? 'bg-signal-abort' : 'bg-accent';
  const ringColor = invalid ? 'ring-signal-abort-dim' : 'ring-accent-dim';
  const readoutColor = invalid ? 'text-signal-abort' : 'text-accent';

  return (
    <div className={`flex flex-col gap-1 ${className ?? ''}`}>
      <div className="flex items-center justify-between">
        <Caps>{label}</Caps>
        <span className={`font-mono text-[11px] tabular-nums ${readoutColor}`}>{readout}</span>
      </div>
      <div className="relative flex h-4 items-center">
        <div className="absolute inset-x-0 inset-y-1.5 h-1 rounded-xs bg-surface-2" />
        <div
          className={`pointer-events-none absolute inset-y-1.5 h-1 rounded-xs ${fillColor}`}
          style={{ left: 0, width: `${pct}%` }}
        />
        <div
          className={`pointer-events-none absolute size-2.5 rounded-full ring-[3px] ${fillColor} ${ringColor}`}
          style={{ left: `calc(${pct}% - 5px)` }}
        />
        <input
          type="range"
          min={min}
          max={max}
          value={value}
          className="relative h-4 w-full cursor-pointer opacity-0"
          {...rest}
        />
      </div>
      {error ? (
        <span role="alert" className="font-mono text-xs text-signal-abort">
          {error}
        </span>
      ) : null}
    </div>
  );
}
