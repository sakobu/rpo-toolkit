import { useState } from 'react';
import { GripHorizontal, Minus, Plus } from 'lucide-react';

import { useHotkey } from '@/hooks/useHotkey';
import { useUI } from '@/stores/ui';
import { withBlur } from '@/utils/blur';

export default function HUD() {
  const hudVisible = useUI((s) => s.hudVisible);
  const toggleHUD = useUI((s) => s.toggleHUD);
  const [minimized, setMinimized] = useState(false);

  useHotkey('h', toggleHUD);

  if (!hudVisible) return null;

  return (
    <aside className="pointer-events-auto fixed top-4 right-4 z-30 w-78 rounded-sm border border-border bg-surface-1/80 backdrop-blur-sm">
      <header className="flex items-center justify-between border-b border-border px-3 py-2">
        <div className="flex items-center gap-2 text-text-muted">
          <GripHorizontal size={14} strokeWidth={1.5} />
          <span className="font-mono text-[10px] tracking-wider uppercase">hud</span>
        </div>
        <button
          type="button"
          onClick={withBlur(() => setMinimized((prev) => !prev))}
          className="flex h-5 w-5 cursor-pointer items-center justify-center rounded-xs text-text-dim hover:text-text"
          aria-label={minimized ? 'expand hud' : 'minimize hud'}
        >
          {minimized ? <Plus size={12} strokeWidth={1.5} /> : <Minus size={12} strokeWidth={1.5} />}
        </button>
      </header>
      <div
        className={`duration-base overflow-hidden transition-[max-height] ease-out ${
          minimized ? 'max-h-0' : 'max-h-128'
        }`}
      >
        <div className="px-3 py-3 font-mono text-xs text-text-dim">
          mission dashboard — coming soon
        </div>
      </div>
    </aside>
  );
}
