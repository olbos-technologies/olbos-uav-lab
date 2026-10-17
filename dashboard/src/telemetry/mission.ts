import type { Tone } from './types';

export const PHASES = [
  { state: 'PRE_FLIGHT_CHECK', label: 'Preflight' },
  { state: 'READY', label: 'Ready' },
  { state: 'TAKEOFF', label: 'Takeoff' },
  { state: 'NAVIGATION', label: 'Navigate' },
  { state: 'LANDING', label: 'Land' },
  { state: 'COMPLETE', label: 'Complete' },
] as const;

export const EXCEPTIONAL = new Set(['ABORT', 'EMERGENCY', 'COMMUNICATION_LOSS', 'RETURN_TO_LAUNCH']);

const CRITICAL = new Set([
  'COMMAND_REJECTED',
  'GEOFENCE',
  'INVALID_POSITION',
  'INVALID_COMMAND',
  'INVALID_SPAWN',
  'INVALID_CLOCK',
  'LOW_BATTERY',
  'MISSION_DEADLINE',
  'LEG_TIMEOUT',
  'UNEXPECTED_VEHICLE_STATE',
]);

export function alertTone(alert?: string): Tone {
  if (!alert || alert === 'UNASSESSED') return 'neutral';
  if (alert === 'NORMAL') return 'ok';
  return CRITICAL.has(alert) ? 'critical' : 'warn';
}

export function outcomeTone(outcome?: string): Tone {
  switch (outcome) {
    case 'COMPLETE':
      return 'ok';
    case 'ABORTED_LANDED':
      return 'warn';
    case 'FAILED':
      return 'critical';
    case 'RUNNING':
      return 'info';
    default:
      return 'neutral';
  }
}

export const humanize = (code?: string) =>
  code ? code.toLowerCase().replace(/_/g, ' ').replace(/^\w/, (c) => c.toUpperCase()) : '—';

export const fmt = (x: number | null | undefined, digits = 1) =>
  typeof x === 'number' && Number.isFinite(x) ? x.toFixed(digits) : '—';
