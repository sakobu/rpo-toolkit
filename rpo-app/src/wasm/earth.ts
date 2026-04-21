import type { Result } from '@railway-ts/pipelines/result';

import type { EcefState, GeodeticCoord, StateVector, Vec3, WasmError } from 'rpo-wasm';
import {
  earth_rotation_angle_rad,
  ecef_to_eci_position_km,
  ecef_to_geodetic,
  eci_to_ecef_position_km,
  eci_to_ecef_state,
  geodetic_to_ecef_km,
} from 'rpo-wasm';

import { callWasm } from './error';

export function earthRotationAngleRad(epoch: string): Result<number, WasmError> {
  return callWasm(() => earth_rotation_angle_rad(epoch));
}

export function eciToEcefPositionKm(rEciKm: Vec3, epoch: string): Result<Vec3, WasmError> {
  return callWasm(() => eci_to_ecef_position_km(rEciKm, epoch));
}

export function ecefToEciPositionKm(rEcefKm: Vec3, epoch: string): Result<Vec3, WasmError> {
  return callWasm(() => ecef_to_eci_position_km(rEcefKm, epoch));
}

export function eciToEcefState(state: StateVector): Result<EcefState, WasmError> {
  return callWasm(() => eci_to_ecef_state(state));
}

export function geodeticToEcefKm(
  latitudeRad: number,
  longitudeRad: number,
  altitudeKm: number,
): Result<Vec3, WasmError> {
  return callWasm(() => geodetic_to_ecef_km(latitudeRad, longitudeRad, altitudeKm));
}

export function ecefToGeodetic(rEcefKm: Vec3): Result<GeodeticCoord, WasmError> {
  return callWasm(() => ecef_to_geodetic(rEcefKm));
}
