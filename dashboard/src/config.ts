const DEFAULT_URL = 'ws://127.0.0.1:8765';
const LOOPBACK = new Set(['127.0.0.1', 'localhost', '[::1]']);

// `?ws=ws://127.0.0.1:9000` overrides the bridge; non-loopback hosts are ignored by design.
export function telemetryUrl(search = window.location.search): string {
  const requested = new URLSearchParams(search).get('ws');
  if (!requested) return DEFAULT_URL;
  try {
    const u = new URL(requested);
    return (u.protocol === 'ws:' || u.protocol === 'wss:') && LOOPBACK.has(u.hostname) ? u.toString() : DEFAULT_URL;
  } catch {
    return DEFAULT_URL;
  }
}
