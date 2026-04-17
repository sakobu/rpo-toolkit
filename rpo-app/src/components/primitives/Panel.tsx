import { type ReactNode } from 'react';

type PanelProps = {
  title?: ReactNode;
  subtitle?: ReactNode;
  children: ReactNode;
  className?: string;
};

export function Panel({ title, subtitle, children, className }: PanelProps) {
  return (
    <section
      className={`rounded-md border border-border bg-surface-1 p-4 shadow-panel ${className ?? ''}`}
    >
      {(title || subtitle) && (
        <header className="mb-3 flex items-baseline justify-between border-b border-border/60 pb-2">
          {title && <h2 className="text-sm font-semibold tracking-tight text-text">{title}</h2>}
          {subtitle && (
            <span className="font-mono text-xs tracking-wider text-text-dim uppercase">
              {subtitle}
            </span>
          )}
        </header>
      )}
      {children}
    </section>
  );
}
