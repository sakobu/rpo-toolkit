import { type ReactNode } from 'react';
import type { ExtractFieldPaths, UseFormReturn } from '@railway-ts/use-form';

import { FieldErrorContext } from './FieldErrorContext';

type BaseProps = {
  label: ReactNode;
  hint?: ReactNode;
  children: ReactNode;
};

type SmartProps<TValues extends Record<string, unknown>> = BaseProps & {
  name: ExtractFieldPaths<TValues>;
  form: UseFormReturn<TValues>;
  htmlFor?: never;
  error?: never;
};

type ManualProps = BaseProps & {
  htmlFor: string;
  error?: string;
  name?: never;
  form?: never;
};

type FormFieldProps<TValues extends Record<string, unknown>> = SmartProps<TValues> | ManualProps;

export function FormField<TValues extends Record<string, unknown>>(props: FormFieldProps<TValues>) {
  const { label, hint, children } = props;
  const htmlFor = props.form ? props.form.getFieldId(props.name) : props.htmlFor;
  const error = props.form ? props.form.getFieldError(props.name) : props.error;

  return (
    <div className="flex flex-col gap-1">
      <label
        htmlFor={htmlFor}
        className="text-xs font-medium tracking-wide text-text-muted uppercase"
      >
        {label}
      </label>
      <FieldErrorContext value={error}>{children}</FieldErrorContext>
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
