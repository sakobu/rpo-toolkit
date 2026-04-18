import { type ButtonHTMLAttributes, type Ref } from 'react';

type IconButtonProps = Omit<ButtonHTMLAttributes<HTMLButtonElement>, 'type' | 'aria-label'> & {
  type?: 'button' | 'submit' | 'reset';
  'aria-label': string;
  ref?: Ref<HTMLButtonElement>;
};

const BASE =
  'flex h-5 w-5 cursor-pointer items-center justify-center rounded-xs text-text-dim hover:text-text disabled:cursor-not-allowed disabled:text-text-dim/50';

export function IconButton({
  type = 'button',
  className,
  children,
  ref,
  ...rest
}: IconButtonProps) {
  return (
    <button ref={ref} type={type} className={`${BASE} ${className ?? ''}`} {...rest}>
      {children}
    </button>
  );
}
