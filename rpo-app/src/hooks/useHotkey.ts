import { useEffect } from 'react';

type HotkeyOptions = {
  shift?: boolean;
};

export function useHotkey(key: string, callback: () => void, options?: HotkeyOptions): void {
  const requireShift = !!options?.shift;
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (
        e.target instanceof HTMLInputElement ||
        e.target instanceof HTMLTextAreaElement ||
        e.target instanceof HTMLSelectElement
      ) {
        return;
      }
      if (e.key.toLowerCase() !== key.toLowerCase()) return;
      if (e.shiftKey !== requireShift) return;
      callback();
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [key, callback, requireShift]);
}
