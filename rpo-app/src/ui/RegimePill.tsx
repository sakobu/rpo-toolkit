import { Chip } from '@/ui/Chip';

export type Regime = 'FAR-FIELD' | 'PROXIMITY';

type RegimePillProps = {
  regime: Regime;
};

export function RegimePill({ regime }: RegimePillProps) {
  return <Chip tone={regime === 'PROXIMITY' ? 'go' : 'hold'}>{regime}</Chip>;
}
