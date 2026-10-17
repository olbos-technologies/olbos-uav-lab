import { alertTone, humanize } from '../telemetry/mission';
import type { Sample } from '../telemetry/types';

export function SafetyPanel({ latest, stale }: { latest?: Sample; stale: boolean }) {
  const tone = stale ? 'warn' : alertTone(latest?.alert);
  const headline = stale ? 'Telemetry stale' : latest?.alert ? humanize(latest.alert) : 'Unassessed';
  return (
    <article className={`panel safety tone-${tone}`}>
      <header className="panel-head">
        <h2>Safety & health</h2>
        <span className="status-dot" />
      </header>
      <p className="safety-headline">{headline}</p>
      <p className="muted">Vehicle safety decisions remain in the C++ supervisor and PX4.</p>
      <dl>
        <div>
          <dt>Machine learning</dt>
          <dd>Model output unavailable · offline synthetic experiments only</dd>
        </div>
        <div>
          <dt>Mission control</dt>
          <dd>
            Command interface not enabled
            <span className="cmd-row">
              <button disabled>Start</button>
              <button disabled>Land</button>
              <button disabled>Abort</button>
            </span>
          </dd>
        </div>
      </dl>
    </article>
  );
}
