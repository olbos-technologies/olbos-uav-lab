import type { ReactNode } from 'react';
import type { Tone } from '../telemetry/types';

export function MetricCard(props: { label: string; value: ReactNode; unit?: string; hint: string; tone?: Tone; meter?: number }) {
  const { label, value, unit, hint, tone = 'neutral', meter } = props;
  return (
    <article className={`panel metric tone-${tone}`}>
      <span className="eyebrow">{label}</span>
      <div className="metric-value">
        {value}
        {unit && <small>{unit}</small>}
      </div>
      {meter !== undefined && (
        <div className="bar" role="meter" aria-valuemin={0} aria-valuemax={100} aria-valuenow={Math.round(meter * 100)} aria-label={label}>
          <i style={{ width: `${Math.max(0, Math.min(1, meter)) * 100}%` }} />
        </div>
      )}
      <span className="hint">{hint}</span>
    </article>
  );
}
