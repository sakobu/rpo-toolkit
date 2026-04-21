import { curry, pipe } from '@railway-ts/pipelines/composition';

import type { Vehicle } from '@/domain/vehicle';
import type { SpacecraftConfig } from '@/schemas/spacecraft';

import type { SpacecraftProps } from '../types';

import {
  buildDeployedArms,
  buildDeployedPanels,
  buildMainBody,
  buildStowedPanels,
  buildTrackingMarkers,
} from './builders';
import { DEPLOYED_PANEL_THRESHOLD_M2, type Tint, TINTS } from './constants';
import { bodyScaleFromDrag, panelScaleFromSrp } from './scaling';

interface BuildContext {
  readonly config: SpacecraftConfig;
  readonly tint: Tint;
  readonly bodyScale: number;
  readonly deployed: boolean;
}

const seedContext = (vehicle: Vehicle, config: SpacecraftConfig): BuildContext => ({
  config,
  tint: TINTS[vehicle],
  bodyScale: bodyScaleFromDrag(config.drag_area_m2),
  deployed: config.srp_area_m2 > DEPLOYED_PANEL_THRESHOLD_M2,
});

const assembleProps = (ctx: BuildContext): SpacecraftProps => ({
  scale: ctx.bodyScale,
  mainBody: buildMainBody(ctx.tint),
  arms: ctx.deployed ? buildDeployedArms() : [],
  solarPanels: ctx.deployed
    ? buildDeployedPanels(panelScaleFromSrp(ctx.config.srp_area_m2))
    : buildStowedPanels(),
  trackingMarkers: buildTrackingMarkers(ctx.tint),
});

export const configToSpacecraftProps = (
  config: SpacecraftConfig,
  vehicle: Vehicle,
): SpacecraftProps => pipe(config, curry(seedContext)(vehicle), assembleProps);
