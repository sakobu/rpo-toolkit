import { GripHorizontal } from 'lucide-react';

import { useResizeDrag } from '@/hooks/useResizeDrag';

type Props = {
  initialHeight: number;
  minHeight: number;
  maxHeight: number;
  onResize: (height: number) => void;
};

export default function ResizeHandle({ initialHeight, minHeight, maxHeight, onResize }: Props) {
  const { isDragging, handlePointerDown } = useResizeDrag({
    initialHeight,
    minHeight,
    maxHeight,
    onResize,
  });

  return (
    <div
      onPointerDown={handlePointerDown}
      className={`flex h-6 cursor-ns-resize items-center justify-center border-b border-border ${
        isDragging ? 'bg-surface-3' : 'bg-surface-2 hover:bg-surface-3'
      }`}
      aria-label="resize mission panel"
    >
      <GripHorizontal size={14} strokeWidth={1.5} className="text-text-dim" />
    </div>
  );
}
