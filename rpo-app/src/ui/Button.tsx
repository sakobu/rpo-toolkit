import { type ButtonHTMLAttributes, type Ref } from 'react';

type ButtonVariant = 'primary' | 'ghost';

type ButtonProps = Omit<ButtonHTMLAttributes<HTMLButtonElement>, 'type'> & {
  variant?: ButtonVariant;
  type?: 'button' | 'submit' | 'reset';
  ref?: Ref<HTMLButtonElement>;
};

const BASE = 'duration-fast font-mono text-xs tracking-wider uppercase transition-colors';

const VARIANT_CLASSES: Record<ButtonVariant, { enabled: string; disabled: string }> = {
  primary: {
    enabled:
      'cursor-pointer rounded-sm border border-accent bg-accent/10 px-4 py-2 text-accent hover:bg-accent/20',
    disabled:
      'cursor-not-allowed rounded-sm border border-border bg-surface-2 px-4 py-2 text-text-dim',
  },
  ghost: {
    enabled: 'cursor-pointer text-text-dim hover:text-text-muted',
    disabled: 'cursor-not-allowed text-text-dim/50',
  },
};

export function Button({
  variant = 'primary',
  type = 'button',
  disabled = false,
  className,
  children,
  ref,
  ...rest
}: ButtonProps) {
  const variantClasses = VARIANT_CLASSES[variant];
  const stateClass = disabled ? variantClasses.disabled : variantClasses.enabled;
  return (
    <button
      ref={ref}
      type={type}
      disabled={disabled}
      className={`${BASE} ${stateClass} ${className ?? ''}`}
      {...rest}
    >
      {children}
    </button>
  );
}
