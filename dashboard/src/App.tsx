import { useRef, useState, type DragEvent } from 'react';
import { EventLog } from './components/EventLog';
import { MetricCard } from './components/MetricCard';
import { MissionTimeline } from './components/MissionTimeline';
import { ReplayBar } from './components/ReplayBar';
import { SafetyPanel } from './components/SafetyPanel';
import { TelemetryChart } from './components/TelemetryChart';
import { TrajectoryMap } from './components/TrajectoryMap';
import { telemetryUrl } from './config';
import { fmt, humanize } from './telemetry/mission';
import { palettes, useTheme } from './telemetry/theme';
import { MAX_TRACE_BYTES, parseTrace, useReplay, type Trace } from './telemetry/useReplay';
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
  const [trace, setTrace] = useState<Trace | null>(null);
  const [fileError, setFileError] = useState('');
  const fileInput = useRef<HTMLInputElement>(null);
  // The live connection stays open during replay: the local simulation demo keeps one session per
  // connection, so closing it would discard the user's demo flight. Only the display follows the trace.
  const live = useTelemetry(url);
  const replay = useReplay(trace);
  const { rows, events, latest: d, stale } = replay ?? live;
  const theme = useTheme();
  const palette = palettes[theme.resolved];
  const status = replay
    ? { text: `Replay · ${(replay.t - replay.start).toFixed(1)} s`, tone: 'info' }
    : linkLabel(live.link, live.stale, live.retryIn);
  const error = replay ? fileError : fileError || live.error;

  const openFile = async (file?: File) => {
    if (!file) return;
    try {
      if (file.size > MAX_TRACE_BYTES) throw new Error('Trace larger than 20 MB');
      setTrace(parseTrace(file.name, await file.text()));
      setFileError('');
    } catch (e) {
      setFileError(`${file.name}: ${e instanceof Error ? e.message : 'unreadable'}`);
    }
  };
  const onDrop = (e: DragEvent) => {
    e.preventDefault();
    void openFile(e.dataTransfer.files[0]);
  };
  const battery = typeof d?.battery === 'number' ? d.battery : undefined;
  const elapsed = rows.length ? rows[rows.length - 1].time_s - rows[0].time_s : undefined;

  return (
    <div className="shell" onDragOver={(e) => e.preventDefault()} onDrop={onDrop}>
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
          <button className="ghost" onClick={() => fileInput.current?.click()} title="Replay a .jsonl trace (or drop one on the page)">
            Open trace
          </button>
          <input
            ref={fileInput}
            type="file"
            accept=".jsonl,.json,.txt,application/x-ndjson"
            hidden
            onChange={(e) => {
              void openFile(e.target.files?.[0]);
              e.target.value = '';
            }}
          />
          {!replay && (
            <button className="ghost" onClick={live.clear} disabled={!rows.length}>
              Clear
            </button>
          )}
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
          Transport <b>{replay ? 'FILE' : (d?.transport ?? '—')}</b>
        </span>
        <span>
          {replay ? 'File' : 'Bridge'} <b>{replay ? trace!.name : url}</b>
        </span>
        <span>
          {replay
            ? 'Replay of a recorded trace · no live connection'
            : d?.source === 'SIMULATION_DEMO'
              ? 'Interactive local simulation only · no PX4 connection'
              : 'Read-only monitoring'}
        </span>
        {error && <span className="tone-warn">{error}</span>}
      </div>

      <main>
        {replay && (
          <ReplayBar
            name={trace!.name}
            rejected={trace!.rejected}
            t={replay.t}
            start={replay.start}
            end={replay.end}
            playing={replay.playing}
            speed={replay.speed}
            onToggle={replay.toggle}
            onSeek={replay.seek}
            onSpeed={replay.setSpeed}
            onExit={() => setTrace(null)}
          />
        )}
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
              sendCommand={live.sendCommand}
              // Demo commands are disabled during replay so a click never acts on a hidden live session.
              commandPending={!!replay || live.commandPending}
              commandStatus={replay ? '' : live.commandStatus}
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
