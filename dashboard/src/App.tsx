import { EventLog } from './components/EventLog';
import { MetricCard } from './components/MetricCard';
import { MissionTimeline } from './components/MissionTimeline';
import { SafetyPanel } from './components/SafetyPanel';
import { TelemetryChart } from './components/TelemetryChart';
import { TrajectoryMap } from './components/TrajectoryMap';
import { telemetryUrl } from './config';
import { fmt, humanize } from './telemetry/mission';
import { palettes, useTheme } from './telemetry/theme';
import { useTelemetry, type Link } from './telemetry/useTelemetry';
import type { Sample } from './telemetry/types';

const url = telemetryUrl();

function linkLabel(link: Link, stale: boolean, retryIn: number) {
  if (link === 'closed') return { text: retryIn ? `Reconnecting in ${retryIn}s` : 'Reconnecting', tone: 'critical' };
  if (link === 'connecting') return { text: 'Connecting', tone: 'neutral' };
  return stale ? { text: 'Stale / no data', tone: 'warn' } : { text: 'Live', tone: 'ok' };
}

function exportLog(rows: Sample[]) {
  const blob = new Blob([rows.map((r) => JSON.stringify(r)).join('\n')], { type: 'application/x-ndjson' });
  const href = URL.createObjectURL(blob);
  const a = Object.assign(document.createElement('a'), { href, download: 'telemetry-window.jsonl' });
  a.click();
  URL.revokeObjectURL(href);
}

export function App() {
  const {
    rows,
    events,
    latest: d,
    link,
    error,
    stale,
    retryIn,
    clear,
    sendCommand,
    commandPending,
    commandStatus,
  } = useTelemetry(url);
  const theme = useTheme();
  const palette = palettes[theme.resolved];
  const status = linkLabel(link, stale, retryIn);
  const battery = typeof d?.battery === 'number' ? d.battery : undefined;
  const elapsed = rows.length ? rows[rows.length - 1].time_s - rows[0].time_s : undefined;

  return (
    <div className="shell">
      <header className="topbar">
        <div className="brand">
          <svg viewBox="0 0 32 32" aria-hidden="true">
            <path d="M16 3 29 16 16 29 3 16Z" fill="none" stroke="currentColor" strokeWidth="2" />
            <circle cx="16" cy="16" r="4" fill="currentColor" />
          </svg>
          <div>
            <strong>Olbos</strong> UAV Lab
            <small>Flight operations · simulation only</small>
          </div>
        </div>
        <div className="topbar-actions">
          <span className={`pill tone-${status.tone}`} role="status">
            <span className="status-dot" />
            {status.text}
          </span>
          <button className="ghost" onClick={theme.cycle} title="Toggle colour theme">
            Theme: {theme.choice}
          </button>
          <button className="ghost" onClick={clear} disabled={!rows.length}>
            Clear
          </button>
          <button onClick={() => exportLog(rows)} disabled={!rows.length}>
            Export JSONL
          </button>
        </div>
      </header>

      <div className="source-strip">
        <span>
          Source <b>{d?.source ?? 'awaiting telemetry'}</b>
        </span>
        <span>
          Transport <b>{d?.transport ?? '—'}</b>
        </span>
        <span>
          Bridge <b>{url}</b>
        </span>
        <span>
          {d?.source === 'SIMULATION_DEMO'
            ? 'Interactive local simulation only · no PX4 connection'
            : 'Read-only monitoring'}
        </span>
        {error && <span className="tone-warn">{error}</span>}
      </div>

      <main>
        <MissionTimeline rows={rows} latest={d} />

        <section className="metrics">
          <MetricCard label="Altitude" value={fmt(d?.altitude)} unit="m" hint="Local ENU, above origin" tone="info" />
          <MetricCard
            label="Battery"
            value={battery === undefined ? '—' : fmt(battery * 100, 0)}
            unit="%"
            hint="Remaining charge"
            tone={battery === undefined ? 'neutral' : battery < 0.2 ? 'critical' : battery < 0.35 ? 'warn' : 'ok'}
            meter={battery}
          />
          <MetricCard label="Position" value={`${fmt(d?.east)} / ${fmt(d?.north)}`} unit="m" hint="East / north" />
          <MetricCard
            label="Flight mode"
            value={<span className="mode">{d?.state ? humanize(d.state) : d?.mode ?? '—'}</span>}
            hint={elapsed === undefined ? 'Observed, not requested' : `Window ${elapsed.toFixed(1)} s · observed`}
          />
        </section>

        <section className="main-grid">
          <TrajectoryMap rows={rows} latest={d} />
          <div className="side">
            <SafetyPanel
              latest={d}
              stale={stale}
              sendCommand={sendCommand}
              commandPending={commandPending}
              commandStatus={commandStatus}
            />
            <EventLog events={events} />
          </div>
        </section>

        <section className="charts">
          <TelemetryChart title="Altitude" unit="m" rows={rows} value={(r) => r.altitude} color={palette.accent} fill={palette.accentFill} palette={palette} />
          <TelemetryChart
            title="Battery"
            unit="%"
            rows={rows}
            value={(r) => (typeof r.battery === 'number' ? r.battery * 100 : undefined)}
            domain={[0, 100]}
            color={palette.ok}
            fill={palette.accentFill}
            palette={palette}
          />
        </section>
      </main>

      <footer className="footer">
        <span>Olbos UAV Lab · research prototype</span>
        <span>Simulation results do not establish real-flight safety</span>
      </footer>
    </div>
  );
}
