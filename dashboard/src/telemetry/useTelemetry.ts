import { useCallback, useEffect, useRef, useState } from 'react';
import { alertTone, humanize, outcomeTone } from './mission';
import { parseSample, type Sample, type TelemetryEvent } from './types';

export type Link = 'connecting' | 'open' | 'closed';
export type DemoCommand = 'start' | 'land' | 'abort' | 'inject_fault' | 'reset';

const STALE_MS = 2000;
export const MAX_EVENTS = 60;

// Events implied by moving from one sample to the next; shared by live telemetry and file replay.
export function changes(prev: Sample | undefined, next: Sample, nextId: () => number): TelemetryEvent[] {
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
  const [commandStatus, setCommandStatus] = useState('');
  const [commandPending, setCommandPending] = useState(false);
  const previous = useRef<Sample | undefined>(undefined);
  const eventId = useRef(0);
  const socketRef = useRef<WebSocket | undefined>(undefined);
  const pendingCommandId = useRef('');
  const commandTimer = useRef<number | undefined>(undefined);

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
        socketRef.current = socket;
        setLink('open');
        setError('');
      };
      socket.onmessage = (e) => {
        let sample: Sample;
        try {
          const frame: unknown = JSON.parse(String(e.data));
          if (frame && typeof frame === 'object' && (frame as Record<string, unknown>).type === 'ack') {
            const ack = frame as Record<string, unknown>;
            if (typeof ack.request_id === 'string' && ack.request_id === pendingCommandId.current) {
              window.clearTimeout(commandTimer.current);
              pendingCommandId.current = '';
              setCommandPending(false);
              setCommandStatus(typeof ack.message === 'string' ? ack.message : 'Command response received.');
            }
            return;
          }
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
        if (socketRef.current === socket) socketRef.current = undefined;
        window.clearTimeout(commandTimer.current);
        pendingCommandId.current = '';
        setCommandPending(false);
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
      window.clearTimeout(commandTimer.current);
      socketRef.current = undefined;
      socket?.close();
    };
  }, [url, capacity]);

  const clear = useCallback(() => {
    setRows([]);
    setEvents([]);
    previous.current = undefined;
  }, []);

  const sendCommand = useCallback((command: DemoCommand) => {
    const socket = socketRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      setCommandStatus('Connect to the local simulation demo first.');
      return;
    }
    const requestId = `${Date.now()}-${Math.random().toString(16).slice(2)}`;
    try {
      socket.send(JSON.stringify({ type: 'command', command, request_id: requestId }));
      pendingCommandId.current = requestId;
      setCommandPending(true);
      setCommandStatus(`Sending ${command.replace('_', ' ')}…`);
      window.clearTimeout(commandTimer.current);
      commandTimer.current = window.setTimeout(() => {
        if (pendingCommandId.current !== requestId) return;
        pendingCommandId.current = '';
        setCommandPending(false);
        setCommandStatus('No response from the simulation bridge.');
      }, 3000);
    } catch {
      setCommandPending(false);
      setCommandStatus('Could not send command to the simulation bridge.');
    }
  }, []);

  const latest = rows.at(-1);
  const stale = !lastAt || now - lastAt > STALE_MS || (latest?.age_s ?? 0) > STALE_MS / 1000;
  const retryIn = link === 'closed' ? Math.max(0, Math.ceil((retryAt - now) / 1000)) : 0;
  return { rows, events, latest, link, error, stale, retryIn, clear, sendCommand, commandPending, commandStatus };
}
