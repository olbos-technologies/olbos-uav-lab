import { EXCEPTIONAL, PHASES, humanize, outcomeTone } from '../telemetry/mission';
import type { Sample } from '../telemetry/types';

const phaseIndex = (state?: string) => PHASES.findIndex((p) => p.state === state);

export function MissionTimeline({ rows, latest }: { rows: Sample[]; latest?: Sample }) {
  if (!latest?.state)
    return (
      <section className="panel timeline" aria-label="Mission progress">
        <div className="timeline-empty">
          <span className="eyebrow">Mission state</span>
          <p>
            {latest
              ? `Not published by this source. Raw flight mode: ${latest.mode ?? '—'}`
              : 'Awaiting telemetry'}
          </p>
        </div>
      </section>
    );
  // Furthest nominal phase reached; exceptional states mark that phase instead of a new step.
  const reached = rows.reduce((m, r) => Math.max(m, phaseIndex(r.state)), -1);
  const current = Math.max(phaseIndex(latest.state), reached);
  const exceptional = EXCEPTIONAL.has(latest.state);
  const terminal = latest.outcome && latest.outcome !== 'RUNNING';
  const failedStep = terminal && latest.outcome !== 'COMPLETE';
  const total = latest.waypoints ?? 0;
  const done = Math.min(latest.waypoint ?? 0, total);
  return (
    <section className="panel timeline" aria-label="Mission progress">
      <ol className="steps">
        {PHASES.map((p, i) => {
          const cls =
            i < current || (i === current && latest.outcome === 'COMPLETE')
              ? 'done'
              : i === current
                ? exceptional || failedStep
                  ? 'alert'
                  : 'active'
                : '';
          return (
            <li key={p.state} className={cls} aria-current={i === current ? 'step' : undefined}>
              <span className="node" />
              <span className="step-label">{p.label}</span>
            </li>
          );
        })}
      </ol>
      <div className="timeline-meta">
        <span className={`badge tone-${outcomeTone(latest.outcome)}`}>{humanize(latest.outcome)}</span>
        {exceptional && <span className="badge tone-critical">{humanize(latest.state)}</span>}
        {total > 0 && (
          <div className="waypoints" title={`${done} of ${total} waypoints reached`}>
            <span>
              Waypoints <b>{done}</b>/{total}
            </span>
            <div className="bar">
              <i style={{ width: `${(done / total) * 100}%` }} />
            </div>
          </div>
        )}
      </div>
    </section>
  );
}
