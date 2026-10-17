import { alertTone, humanize } from '../telemetry/mission';
import type { Sample } from '../telemetry/types';
import type { DemoCommand } from '../telemetry/useTelemetry';

export function SafetyPanel({
  latest,
  stale,
  sendCommand,
  commandPending,
  commandStatus,
}: {
  latest?: Sample;
  stale: boolean;
  sendCommand: (command: DemoCommand) => void;
  commandPending: boolean;
  commandStatus: string;
}) {
  const tone = stale ? 'warn' : alertTone(latest?.alert);
  const headline = stale ? 'Telemetry stale' : latest?.alert ? humanize(latest.alert) : 'Unassessed';
  const demoConnected = latest?.source === 'SIMULATION_DEMO' && !stale;
  const modelStatus = demoConnected ? latest?.ml_status : undefined;
  const modelTone = modelStatus === 'NORMAL' ? 'ok' : modelStatus ? 'warn' : 'neutral';
  const armed = demoConnected && latest?.armed === true;
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
          <dd>
            {modelStatus ? (
              <>
                <span className={`badge tone-${modelTone}`}>{humanize(modelStatus)}</span>
                <p className="muted ml-detail">
                  Synthetic model confidence {Math.round((latest?.ml_probability ?? 0) * 100)}% · {latest?.ml_model}
                </p>
              </>
            ) : (
              'No advisory output · start the local simulation demo'
            )}
            <small className="demo-disclaimer">Synthetic data only. Advisory display; it cannot control the vehicle.</small>
          </dd>
        </div>
        <div>
          <dt>Mission control</dt>
          <dd>
            {demoConnected ? 'Local simulation connected · no PX4 connection' : 'Simulation controls unavailable'}
            <span className="cmd-row">
              <button disabled={!demoConnected || armed || commandPending} onClick={() => sendCommand('start')}>Start</button>
              <button disabled={!armed || commandPending} onClick={() => sendCommand('land')}>Land</button>
              <button disabled={!armed || commandPending} onClick={() => sendCommand('abort')}>Abort</button>
            </span>
            <span className="cmd-row">
              <button className="ghost" disabled={!armed || commandPending} onClick={() => sendCommand('inject_fault')}>
                Test anomaly
              </button>
              <button className="ghost" disabled={!demoConnected || armed || commandPending} onClick={() => sendCommand('reset')}>
                Reset
              </button>
            </span>
            {commandStatus && <small className="command-status" role="status">{commandStatus}</small>}
          </dd>
        </div>
      </dl>
    </article>
  );
}
