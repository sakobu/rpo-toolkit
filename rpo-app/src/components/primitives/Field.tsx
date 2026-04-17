import { type ReactNode } from 'react';

type FieldProps = {
  label: ReactNode;
  htmlFor: string;
  error?: string;
  hint?: ReactNode;
  children: ReactNode;
};

export function Field({ label, htmlFor, error, hint, children }: FieldProps) {
  return (
    <div className="flex flex-col gap-1">
      <label
        htmlFor={htmlFor}
        className="text-xs font-medium tracking-wide text-text-muted uppercase"
      >
        {label}
      </label>
      {children}
      {error ? (
        <span role="alert" className="font-mono text-xs text-signal-abort">
          {error}
        </span>
      ) : hint ? (
        <span className="font-mono text-xs text-text-dim">{hint}</span>
      ) : null}
    </div>
  );
}
