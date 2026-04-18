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
  // Namespace the rendered htmlFor when multiple forms of the same shape live on one page
  // (e.g. Deputy + Chief SpacecraftPanel). `@railway-ts/use-form` returns the field name
  // verbatim from `getFieldId`, so without a prefix both forms collide on `id`.
  idPrefix?: string;
  htmlFor?: never;
  error?: never;
};

type ManualProps = BaseProps & {
  htmlFor: string;
  error?: string;
  name?: never;
  form?: never;
  idPrefix?: never;
};

type FormFieldProps<TValues extends Record<string, unknown>> = SmartProps<TValues> | ManualProps;

export function FormField<TValues extends Record<string, unknown>>(props: FormFieldProps<TValues>) {
  const { label, hint, children } = props;
  const rawId = props.form ? props.form.getFieldId(props.name) : props.htmlFor;
  const htmlFor = props.idPrefix ? `${props.idPrefix}-${rawId}` : rawId;
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
