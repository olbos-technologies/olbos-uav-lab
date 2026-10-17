// Telemetry contract shared by the replay and live bridges (docs/interfaces/contracts.md).
export type Sample = {
  source: string;
  transport?: string;
  time_s: number;
  east?: number;
  north?: number;
  altitude?: number;
  battery?: number | null;
  mode?: string;
  alert?: string;
  age_s?: number;
  armed?: boolean;
  position_valid?: boolean;
  // Present when the shared MissionController produced the sample (harness replay).
  state?: string;
  outcome?: string;
  waypoint?: number;
  waypoints?: number;
};

export type Tone = 'ok' | 'info' | 'warn' | 'critical' | 'neutral';

export type TelemetryEvent = {
  id: number;
  time_s: number;
  label: string;
  detail: string;
  tone: Tone;
};

const num = (v: unknown) => (typeof v === 'number' && Number.isFinite(v) ? v : undefined);
const str = (v: unknown) => (typeof v === 'string' && v.length <= 64 ? v : undefined);
const bool = (v: unknown) => (typeof v === 'boolean' ? v : undefined);

// Copies only known, well-typed fields so malformed bridge output never reaches rendering.
export function parseSample(raw: string): Sample {
  const d: unknown = JSON.parse(raw);
  if (!d || typeof d !== 'object') throw new Error('Telemetry must be an object');
  const r = d as Record<string, unknown>;
  const time_s = num(r.time_s);
  const source = str(r.source);
  if (time_s === undefined || source === undefined) throw new Error('Missing time_s/source');
  return {
    source,
    time_s,
    transport: str(r.transport),
    east: num(r.east),
    north: num(r.north),
    altitude: num(r.altitude),
    battery: r.battery === null ? null : num(r.battery),
    mode: str(r.mode),
    alert: str(r.alert),
    age_s: num(r.age_s),
    armed: bool(r.armed),
    position_valid: bool(r.position_valid),
    state: str(r.state),
    outcome: str(r.outcome),
    waypoint: num(r.waypoint),
    waypoints: num(r.waypoints),
  };
}
