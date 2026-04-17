import { type SelectHTMLAttributes } from 'react';

import { useFieldError } from './FieldErrorContext';

type SelectProps = SelectHTMLAttributes<HTMLSelectElement>;

export function Select({ className, children, ...rest }: SelectProps) {
  const invalid = Boolean(useFieldError());
  const border = invalid ? 'border-signal-abort' : 'border-border';

  return (
    <select
      className={`duration-fast appearance-none rounded-sm border bg-surface-2 px-2 py-1 pr-7 text-text transition-colors ${border} ${className ?? ''}`}
      {...rest}
    >
      {children}
    </select>
  );
}
