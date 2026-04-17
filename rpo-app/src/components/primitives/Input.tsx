import { type InputHTMLAttributes } from 'react';

type InputProps = InputHTMLAttributes<HTMLInputElement> & {
  invalid?: boolean;
};

export function Input({ invalid, className, type = 'text', ...rest }: InputProps) {
  const numeric = type === 'number';
  const border = invalid ? 'border-signal-abort' : 'border-border';

  return (
    <input
      type={type}
      className={`duration-fast rounded-sm border bg-surface-2 px-2 py-1 text-text transition-colors ${border} ${numeric ? 'font-mono tabular-nums' : ''} ${className ?? ''}`}
      {...rest}
    />
  );
}
