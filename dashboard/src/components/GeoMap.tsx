import { useEffect, useRef, useState, type KeyboardEvent, type PointerEvent } from 'react';
import type { Sample } from '../telemetry/types';

// Draws local ENU telemetry over raster map tiles around a chosen display origin.
// This is a visual projection, not GPS: the telemetry carries no geographic position.
type Origin = { lat: number; lon: number };
type Point = { x: number; y: number };

const DEFAULT_ORIGIN: Origin = { lat: 36.154, lon: -95.9928 };
const ORIGIN_KEY = 'olbos-map-origin';
const TILE = 256;
const DEFAULT_ZOOM = 18;
const MIN_ZOOM = 13;
const MAX_ZOOM = 20;
const METERS_PER_DEGREE = 111_320;
const EARTH_CIRCUMFERENCE = 40_075_016.686;
const DEFAULT_TILES = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';
const configured = import.meta.env.VITE_MAP_TILE_URL;
const TILE_TEMPLATE =
  configured && /^https:\/\/.+\{z\}.*\{x\}.*\{y\}/.test(configured) ? configured : DEFAULT_TILES;

function worldPixel(lat: number, lon: number, zoom: number): Point {
  const size = TILE * 2 ** zoom;
  const sin = Math.sin((Math.max(-85.05112878, Math.min(85.05112878, lat)) * Math.PI) / 180);
  return {
    x: ((lon + 180) / 360) * size,
    y: (0.5 - Math.log((1 + sin) / (1 - sin)) / (4 * Math.PI)) * size,
  };
}

function localToGeo(east: number, north: number, origin: Origin): Origin {
  return {
    lat: origin.lat + north / METERS_PER_DEGREE,
    lon: origin.lon + east / (METERS_PER_DEGREE * Math.cos((origin.lat * Math.PI) / 180)),
  };
}

function storedOrigin(): Origin {
  try {
    const v = JSON.parse(localStorage.getItem(ORIGIN_KEY) ?? 'null');
    if (Number.isFinite(v?.lat) && Number.isFinite(v?.lon) && Math.abs(v.lat) <= 85 && Math.abs(v.lon) <= 180)
      return { lat: v.lat, lon: v.lon };
  } catch {
    /* storage unavailable or corrupt: fall back to default */
  }
  return DEFAULT_ORIGIN;
}

// Text drafts let users clear the field or type a leading '-' without the map jumping to 0.
function OriginField({ label, value, limit, onCommit }: { label: string; value: number; limit: number; onCommit: (v: number) => void }) {
  const [draft, setDraft] = useState(value.toFixed(6));
  useEffect(() => setDraft(value.toFixed(6)), [value]);
  const parsed = draft.trim() === '' ? NaN : Number(draft);
  const valid = Number.isFinite(parsed) && Math.abs(parsed) <= limit;
  const commit = () => (valid ? onCommit(parsed) : setDraft(value.toFixed(6)));
  return (
    <label className={valid ? '' : 'invalid'}>
      {label}
      <input
        inputMode="decimal"
        value={draft}
        aria-invalid={!valid}
        onChange={(e) => setDraft(e.target.value)}
        onBlur={commit}
        onKeyDown={(e: KeyboardEvent) => e.key === 'Enter' && commit()}
      />
    </label>
  );
}

export function GeoMap({ rows, latest, fence }: { rows: Sample[]; latest?: Sample; fence: number }) {
  const [origin, setOrigin] = useState<Origin>(storedOrigin);
  const [zoom, setZoom] = useState(DEFAULT_ZOOM);
  const [pan, setPan] = useState<Point>({ x: 0, y: 0 });
  const [size, setSize] = useState({ width: 0, height: 0 });
  const stage = useRef<HTMLDivElement | null>(null);
  const drag = useRef<{ x: number; y: number; pan: Point } | null>(null);

  useEffect(() => {
    const el = stage.current;
    if (!el) return;
    const observer = new ResizeObserver(([entry]) => {
      if (entry) setSize({ width: entry.contentRect.width, height: entry.contentRect.height });
    });
    observer.observe(el);
    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    try {
      localStorage.setItem(ORIGIN_KEY, JSON.stringify(origin));
    } catch {
      /* storage unavailable: origin lasts for this session only */
    }
  }, [origin]);

  // Pan is in screen pixels at the current zoom, so it scales with zoom to keep the same place in view.
  const zoomBy = (delta: number) => {
    const next = Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, zoom + delta));
    const k = 2 ** (next - zoom);
    setPan((p) => ({ x: p.x * k, y: p.y * k }));
    setZoom(next);
  };

  const center = worldPixel(origin.lat, origin.lon, zoom);
  const ox = size.width / 2 + pan.x;
  const oy = size.height / 2 + pan.y;
  const tiles: { key: string; url: string; left: number; top: number }[] = [];
  if (size.width && size.height) {
    const count = 2 ** zoom;
    const minX = Math.floor((center.x - ox) / TILE);
    const maxX = Math.floor((center.x - ox + size.width - 1) / TILE);
    const minY = Math.max(0, Math.floor((center.y - oy) / TILE));
    const maxY = Math.min(count - 1, Math.floor((center.y - oy + size.height - 1) / TILE));
    for (let y = minY; y <= maxY; y++)
      for (let x = minX; x <= maxX; x++) {
        const wrapped = ((x % count) + count) % count;
        tiles.push({
          key: `${zoom}/${x}/${y}`,
          url: TILE_TEMPLATE.replace('{z}', String(zoom)).replace('{x}', String(wrapped)).replace('{y}', String(y)),
          left: x * TILE - center.x + ox,
          top: y * TILE - center.y + oy,
        });
      }
  }

  const toScreen = (east: number, north: number): Point => {
    const g = localToGeo(east, north, origin);
    const p = worldPixel(g.lat, g.lon, zoom);
    return { x: p.x - center.x + ox, y: p.y - center.y + oy };
  };
  const points = rows
    .filter((r) => r.east !== undefined && r.north !== undefined)
    .map((r) => toScreen(r.east!, r.north!));
  const here = latest?.east !== undefined && latest.north !== undefined ? toScreen(latest.east, latest.north!) : undefined;
  const metersPerPixel = (EARTH_CIRCUMFERENCE * Math.cos((origin.lat * Math.PI) / 180)) / (TILE * 2 ** zoom);

  const startDrag = (e: PointerEvent<HTMLDivElement>) => {
    if (e.button !== 0) return;
    e.currentTarget.setPointerCapture(e.pointerId);
    drag.current = { x: e.clientX, y: e.clientY, pan };
  };
  const moveDrag = (e: PointerEvent<HTMLDivElement>) => {
    const d = drag.current;
    if (d) setPan({ x: d.pan.x + e.clientX - d.x, y: d.pan.y + e.clientY - d.y });
  };
  const endDrag = () => {
    drag.current = null;
  };
  const stop = (e: PointerEvent) => e.stopPropagation();

  return (
    <div className="geo">
      <div
        className="geo-stage"
        ref={stage}
        onPointerDown={startDrag}
        onPointerMove={moveDrag}
        onPointerUp={endDrag}
        onPointerCancel={endDrag}
        role="region"
        aria-label="Map with the local telemetry route drawn around the display origin. Drag to pan."
      >
        {tiles.map((t) => (
          <img key={t.key} className="geo-tile" src={t.url} alt="" draggable={false} style={{ left: t.left, top: t.top }} decoding="async" />
        ))}
        <svg className="geo-overlay" aria-hidden="true">
          <circle className="geo-fence" cx={ox} cy={oy} r={fence / metersPerPixel} />
          <path className="geo-origin" d={`M${ox - 7} ${oy}h14M${ox} ${oy - 7}v14`} />
          {points.length > 1 && <polyline className="geo-route" points={points.map((p) => `${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' ')} />}
          {here && (
            <g transform={`translate(${here.x} ${here.y})`}>
              <circle className="pulse" r={14} />
              <circle className="geo-vehicle" r={6} />
              {latest?.altitude !== undefined && (
                <text className="geo-label" x={10} y={-10}>
                  {latest.altitude.toFixed(1)} m
                </text>
              )}
            </g>
          )}
        </svg>
        {!points.length && <div className="map-empty geo-empty">Awaiting position telemetry</div>}
        <div className="geo-controls" onPointerDown={stop}>
          <button type="button" className="ghost" aria-label="Zoom in" onClick={() => zoomBy(1)} disabled={zoom >= MAX_ZOOM}>
            +
          </button>
          <button type="button" className="ghost" aria-label="Zoom out" onClick={() => zoomBy(-1)} disabled={zoom <= MIN_ZOOM}>
            −
          </button>
          <button
            type="button"
            className="ghost"
            onClick={() => {
              setPan({ x: 0, y: 0 });
              setZoom(DEFAULT_ZOOM);
            }}
          >
            Center
          </button>
        </div>
        <a className="geo-attribution" href="https://www.openstreetmap.org/copyright" target="_blank" rel="noreferrer" onPointerDown={stop}>
          © OpenStreetMap contributors
        </a>
      </div>
      <div className="geo-footer">
        <div className="geo-origin-fields">
          <OriginField label="Origin lat" value={origin.lat} limit={85} onCommit={(lat) => setOrigin((o) => ({ ...o, lat }))} />
          <OriginField label="Origin lon" value={origin.lon} limit={180} onCommit={(lon) => setOrigin((o) => ({ ...o, lon }))} />
        </div>
        <span className="muted">
          {points.length} samples · zoom {zoom}
        </span>
      </div>
      <p className="geo-disclaimer">
        Local east/north offsets drawn around this display origin. Not GPS; never use for navigation.
      </p>
    </div>
  );
}
