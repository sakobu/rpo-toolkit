import { createContext, useContext } from 'react';

export const FieldErrorContext = createContext<string | undefined>(undefined);

export function useFieldError(): string | undefined {
  return useContext(FieldErrorContext);
}
