import { useState } from 'react';
import type { Sample } from '../telemetry/types';
import { GeoMap } from './GeoMap';

const GEOFENCE_M = 40;
const EXTENTS = [15, 25, 45, 90];
const VIEW_KEY = 'olbos-trajectory-view';
type View = 'local' | 'map';

function storedView(): View {
  try {
    return localStorage.getItem(VIEW_KEY) === 'map' ? 'map' : 'local';
  } catch {
    return 'local';
  }
}

// Local view works offline; the map view fetches tiles from the configured provider on demand.
export function TrajectoryMap({ rows, latest }: { rows: Sample[]; latest?: Sample }) {
  const [view, setView] = useState<View>(storedView);
  const choose = (v: View) => {
    setView(v);
    try {
      localStorage.setItem(VIEW_KEY, v);
    } catch {
      /* storage unavailable: choice lasts for this session */
    }
  };
  return (
    <article className="panel map">
      <header className="panel-head">
        <h2>Trajectory</h2>
        <div className="segmented" role="group" aria-label="Trajectory view">
          <button className={view === 'local' ? 'on' : ''} aria-pressed={view === 'local'} onClick={() => choose('local')}>
            Local
          </button>
          <button className={view === 'map' ? 'on' : ''} aria-pressed={view === 'map'} onClick={() => choose('map')}>
            Map
          </button>
        </div>
      </header>
      {view === 'map' ? <GeoMap rows={rows} latest={latest} fence={GEOFENCE_M} /> : <LocalView rows={rows} latest={latest} />}
      <footer className="panel-foot">
        Telemetry trace · last {rows.length} samples · {GEOFENCE_M} m fence · no physics rendered
      </footer>
    </article>
  );
}

function LocalView({ rows, latest }: { rows: Sample[]; latest?: Sample }) {
  const points = rows.filter(
    (r): r is Sample & { east: number; north: number } => r.east !== undefined && r.north !== undefined,
  );
  // Auto-zoom to the flown area so short routes stay legible; never crop the trace.
  const reach = points.reduce((m, p) => Math.max(m, Math.abs(p.east), Math.abs(p.north)), 0);
  const extent = EXTENTS.find((e) => reach * 1.2 <= e) ?? EXTENTS.at(-1)!;
  const step = extent <= 15 ? 2.5 : extent <= 25 ? 5 : extent <= 45 ? 10 : 20;
  const ticks: number[] = [];
  for (let v = -Math.floor(extent / step) * step; v <= extent; v += step) ticks.push(v);
  const trail = points.map((p) => `${p.east.toFixed(2)},${(-p.north).toFixed(2)}`).join(' ');
  const here = latest?.east !== undefined && latest.north !== undefined ? latest : undefined;
  const start = points[0];
  const s = extent / 45;
  return (
    <div className="map-frame">
        <span className="map-scale muted">ENU · ±{extent} m</span>
        <svg viewBox={`${-extent} ${-extent} ${extent * 2} ${extent * 2}`} role="img" aria-label="Local flight trajectory, east right, north up">
          {ticks.map((t) => (
            <g key={t} className={t === 0 ? 'axis' : 'gridline'}>
              <line x1={t} y1={-extent} x2={t} y2={extent} strokeWidth={0.08 * s * (t === 0 ? 2 : 1)} />
              <line x1={-extent} y1={t} x2={extent} y2={t} strokeWidth={0.08 * s * (t === 0 ? 2 : 1)} />
              {t !== 0 && Math.abs(t) < extent && (
                <text x={t} y={extent - 1 * s} fontSize={2.1 * s} textAnchor="middle">
                  {t}
                </text>
              )}
            </g>
          ))}
          <circle className="fence" r={GEOFENCE_M} strokeWidth={0.3 * s} strokeDasharray={`${1.2 * s} ${0.8 * s}`} />
          <text className="compass" x={0.8 * s} y={-extent + 2.8 * s} fontSize={2.4 * s}>N</text>
          {trail && <polyline className="trail" points={trail} strokeWidth={0.55 * s} />}
          {start && <circle className="start" cx={start.east} cy={-start.north} r={0.9 * s} strokeWidth={0.3 * s} />}
          {here && (
            <g transform={`translate(${here.east} ${-here.north!})`}>
              <circle className="pulse" r={2.4 * s} />
              <circle className="vehicle" r={1.1 * s} strokeWidth={0.35 * s} />
              <text className="vehicle-label" x={2.2 * s} y={-1.8 * s} fontSize={2.2 * s} strokeWidth={0.5 * s}>
                {here.altitude !== undefined ? `${here.altitude.toFixed(1)} m` : ''}
              </text>
            </g>
          )}
        </svg>
        {!points.length && <div className="map-empty">Awaiting position telemetry</div>}
    </div>
  );
}
