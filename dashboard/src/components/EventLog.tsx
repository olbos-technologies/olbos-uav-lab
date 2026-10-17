import type { TelemetryEvent } from '../telemetry/types';

export function EventLog({ events }: { events: TelemetryEvent[] }) {
  return (
    <article className="panel events">
      <header className="panel-head">
        <h2>Event log</h2>
        <span className="muted">{events.length} recent</span>
      </header>
      {events.length ? (
        <ol>
          {events.map((e) => (
            <li key={e.id} className={`tone-${e.tone}`}>
              <time>{e.time_s.toFixed(1)}s</time>
              <span className="dot" />
              <span className="event-label">{e.label}</span>
              <span className="event-detail">{e.detail}</span>
            </li>
          ))}
        </ol>
      ) : (
        <p className="muted empty">State changes, waypoints and alerts appear here.</p>
      )}
    </article>
  );
}
