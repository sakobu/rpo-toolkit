import type { HTMLAttributes } from 'react';

type CapsProps = HTMLAttributes<HTMLSpanElement> & {
  color?: string;
};

export function Caps({ color = 'text-text-dim', className = '', children, ...rest }: CapsProps) {
  return (
    <span
      className={`font-mono text-[10px] tracking-wider uppercase ${color} ${className}`}
      {...rest}
    >
      {children}
    </span>
  );
}
