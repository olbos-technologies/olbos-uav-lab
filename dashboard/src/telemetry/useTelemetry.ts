import { useCallback, useEffect, useRef, useState } from 'react';
import { alertTone, humanize, outcomeTone } from './mission';
import { parseSample, type Sample, type TelemetryEvent } from './types';

export type Link = 'connecting' | 'open' | 'closed';

const STALE_MS = 2000;
const MAX_EVENTS = 60;

function changes(prev: Sample | undefined, next: Sample, nextId: () => number): TelemetryEvent[] {
  const out: TelemetryEvent[] = [];
  const add = (label: string, detail: string, tone: TelemetryEvent['tone']) =>
    out.push({ id: nextId(), time_s: next.time_s, label, detail, tone });
  if (!prev) add('Telemetry', `${next.source} via ${next.transport ?? 'unknown transport'}`, 'info');
  if (next.waypoint !== undefined && prev?.waypoint !== undefined && next.waypoint > prev.waypoint)
    add('Waypoint', `Reached ${next.waypoint} of ${next.waypoints ?? '?'}`, 'ok');
  if (next.state && next.state !== prev?.state) add('State', humanize(next.state), 'info');
  if (next.alert && next.alert !== prev?.alert && (prev || next.alert !== 'NORMAL'))
    add('Alert', humanize(next.alert), alertTone(next.alert));
  if (next.outcome && next.outcome !== 'RUNNING' && next.outcome !== prev?.outcome)
    add('Outcome', humanize(next.outcome), outcomeTone(next.outcome));
  return out;
}

// Read-only telemetry subscription: validates every frame, reconnects with backoff and
// starts a new session when the source clock restarts (e.g. replay reconnect).
export function useTelemetry(url: string, capacity = 600) {
  const [rows, setRows] = useState<Sample[]>([]);
  const [events, setEvents] = useState<TelemetryEvent[]>([]);
  const [link, setLink] = useState<Link>('connecting');
  const [error, setError] = useState('');
  const [lastAt, setLastAt] = useState(0);
  const [now, setNow] = useState(() => Date.now());
  const [retryAt, setRetryAt] = useState(0);
  const previous = useRef<Sample | undefined>(undefined);
  const eventId = useRef(0);

  useEffect(() => {
    let disposed = false;
    let socket: WebSocket | undefined;
    let retry: number | undefined;
    let attempt = 0;
    const connect = () => {
      setLink('connecting');
      try {
        socket = new WebSocket(url);
      } catch {
        setError('Invalid telemetry URL');
        setLink('closed');
        return;
      }
      socket.onopen = () => {
        attempt = 0;
        setLink('open');
        setError('');
      };
      socket.onmessage = (e) => {
        let sample: Sample;
        try {
          sample = parseSample(String(e.data));
        } catch {
          setError('Malformed telemetry rejected');
          return;
        }
        const prev = previous.current;
        const restarted = prev !== undefined && sample.time_s < prev.time_s;
        const base = restarted ? undefined : prev;
        // Newest first; reversed here because state updaters must stay pure (StrictMode reruns them).
        const found = changes(base, sample, () => ++eventId.current).reverse();
        setRows((v) => (restarted ? [sample] : [...v.slice(-(capacity - 1)), sample]));
        if (restarted) setEvents([]);
        if (found.length) setEvents((v) => [...found, ...(restarted ? [] : v)].slice(0, MAX_EVENTS));
        previous.current = sample;
        setLastAt(Date.now());
      };
      socket.onerror = () => setError('Telemetry bridge unavailable');
      socket.onclose = () => {
        if (disposed) return;
        setLink('closed');
        const delay = Math.min(10_000, 500 * 2 ** attempt++);
        setRetryAt(Date.now() + delay);
        retry = window.setTimeout(connect, delay);
      };
    };
    connect();
    const clock = window.setInterval(() => setNow(Date.now()), 500);
    return () => {
      disposed = true;
      window.clearInterval(clock);
      window.clearTimeout(retry);
      socket?.close();
    };
  }, [url, capacity]);

  const clear = useCallback(() => {
    setRows([]);
    setEvents([]);
    previous.current = undefined;
  }, []);

  const latest = rows.at(-1);
  const stale = !lastAt || now - lastAt > STALE_MS || (latest?.age_s ?? 0) > STALE_MS / 1000;
  const retryIn = link === 'closed' ? Math.max(0, Math.ceil((retryAt - now) / 1000)) : 0;
  return { rows, events, latest, link, error, stale, retryIn, clear };
}
