import { type SelectHTMLAttributes } from 'react';

type SelectProps = SelectHTMLAttributes<HTMLSelectElement> & {
  invalid?: boolean;
};

export function Select({ invalid, className, children, ...rest }: SelectProps) {
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
