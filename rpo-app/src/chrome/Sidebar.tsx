import { Link } from 'react-router';
import { ChevronLeft } from 'lucide-react';

import { useHotkey } from '@/hooks/useHotkey';
import { useUI } from '@/stores/ui';
import { withBlur } from '@/utils/blur';

export default function Sidebar() {
  const sidebarOpen = useUI((s) => s.sidebarOpen);
  const toggleSidebar = useUI((s) => s.toggleSidebar);

  useHotkey('s', toggleSidebar);

  return (
    <>
      <aside
        className={`duration-base pointer-events-auto fixed top-0 left-0 z-30 flex h-screen w-70 flex-col border-r border-border bg-surface-1 transition-transform ease-out ${
          sidebarOpen ? 'translate-x-0' : '-translate-x-full'
        }`}
      >
        <header className="border-b border-border px-4 py-3">
          <h1 className="text-sm font-semibold tracking-tight text-text">RPO Toolkit</h1>
          <p className="font-mono text-[10px] tracking-wider text-text-dim uppercase">
            mission planner
          </p>
        </header>
        <div className="flex-1 overflow-y-auto px-4 py-4 font-mono text-xs text-text-dim">
          configuration · classification · transfer — coming soon
        </div>
        <footer className="border-t border-border px-4 py-3">
          <Link
            to="/"
            className="font-mono text-[10px] tracking-wider text-text-dim uppercase hover:text-text-muted"
          >
            ← setup
          </Link>
        </footer>
      </aside>
      <button
        type="button"
        onClick={withBlur(toggleSidebar)}
        className={`duration-base pointer-events-auto fixed top-4 z-40 flex h-8 w-8 cursor-pointer items-center justify-center rounded-sm border border-border bg-surface-1/80 text-text-muted backdrop-blur-sm transition-[left] ease-out hover:text-text ${
          sidebarOpen ? 'left-66' : 'left-4'
        }`}
        aria-label={sidebarOpen ? 'collapse sidebar' : 'expand sidebar'}
      >
        <ChevronLeft
          size={16}
          strokeWidth={1.5}
          className={`duration-base transition-transform ease-out ${
            sidebarOpen ? '' : 'rotate-180'
          }`}
        />
      </button>
    </>
  );
}
