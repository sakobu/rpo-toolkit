import {
  type PointerEvent as ReactPointerEvent,
  useCallback,
  useEffect,
  useRef,
  useState,
} from 'react';

type Options = {
  initialHeight: number;
  minHeight: number;
  maxHeight: number;
  onResize: (height: number) => void;
};

type ResizeDrag = {
  isDragging: boolean;
  handlePointerDown: (e: ReactPointerEvent) => void;
};

export function useResizeDrag({
  initialHeight,
  minHeight,
  maxHeight,
  onResize,
}: Options): ResizeDrag {
  const [isDragging, setIsDragging] = useState(false);
  const startY = useRef(0);
  const startHeight = useRef(initialHeight);
  const elementRef = useRef<HTMLElement | null>(null);

  const onResizeRef = useRef(onResize);
  useEffect(() => {
    onResizeRef.current = onResize;
  });

  const handlePointerDown = useCallback(
    (e: ReactPointerEvent) => {
      e.preventDefault();
      const target = e.currentTarget;
      if (!(target instanceof HTMLElement)) return;
      target.setPointerCapture(e.pointerId);
      elementRef.current = target;
      setIsDragging(true);
      startY.current = e.clientY;
      startHeight.current = initialHeight;
      document.body.style.cursor = 'ns-resize';
      document.body.style.userSelect = 'none';
    },
    [initialHeight],
  );

  useEffect(() => {
    if (!isDragging || !elementRef.current) return;
    const element = elementRef.current;

    const handlePointerMove = (e: PointerEvent) => {
      const deltaY = startY.current - e.clientY;
      const newHeight = Math.min(maxHeight, Math.max(minHeight, startHeight.current + deltaY));
      onResizeRef.current(newHeight);
    };

    const handlePointerUp = (e: PointerEvent) => {
      element.releasePointerCapture(e.pointerId);
      setIsDragging(false);
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
    };

    element.addEventListener('pointermove', handlePointerMove);
    element.addEventListener('pointerup', handlePointerUp);
    element.addEventListener('pointercancel', handlePointerUp);

    return () => {
      element.removeEventListener('pointermove', handlePointerMove);
      element.removeEventListener('pointerup', handlePointerUp);
      element.removeEventListener('pointercancel', handlePointerUp);
    };
  }, [isDragging, minHeight, maxHeight]);

  return { isDragging, handlePointerDown };
}
