import type { MouseEvent } from 'react';

/**
 * Wraps a click handler to blur the element afterward, preventing the focus
 * ring from lingering after clicks on toggle buttons.
 */
export function withBlur<T extends () => void>(fn: T) {
  return (e: MouseEvent<HTMLButtonElement>) => {
    fn();
    e.currentTarget.blur();
  };
}
