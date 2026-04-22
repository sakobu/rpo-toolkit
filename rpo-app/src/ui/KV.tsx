import { Caps } from '@/ui/Caps';

type KVProps = {
  k: string;
  v: string;
  color?: string;
};

export function KV({ k, v, color = 'text-text' }: KVProps) {
  return (
    <div className="flex items-baseline justify-between gap-3">
      <Caps>{k}</Caps>
      <span className={`font-mono text-[11px] ${color}`}>{v}</span>
    </div>
  );
}
