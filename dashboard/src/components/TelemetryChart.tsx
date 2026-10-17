import { Area, AreaChart, CartesianGrid, ResponsiveContainer, Tooltip, XAxis, YAxis } from 'recharts';
import type { Palette } from '../telemetry/theme';
import type { Sample } from '../telemetry/types';

type Props = {
  title: string;
  unit: string;
  rows: Sample[];
  value: (s: Sample) => number | undefined;
  domain?: [number, number];
  color: string;
  fill: string;
  palette: Palette;
};

export function TelemetryChart({ title, unit, rows, value, domain, color, fill, palette }: Props) {
  const data = rows.map((r) => ({ t: r.time_s, v: value(r) }));
  return (
    <article className="panel chart">
      <header className="panel-head">
        <h2>{title}</h2>
        <span className="muted">{unit} · latest window</span>
      </header>
      <ResponsiveContainer width="100%" height={170}>
        <AreaChart data={data} margin={{ top: 6, right: 6, bottom: 0, left: -18 }}>
          <defs>
            <linearGradient id={`g-${title}`} x1="0" y1="0" x2="0" y2="1">
              <stop offset="0%" stopColor={fill} stopOpacity={1} />
              <stop offset="100%" stopColor={fill} stopOpacity={0} />
            </linearGradient>
          </defs>
          <CartesianGrid stroke={palette.grid} vertical={false} />
          <XAxis dataKey="t" type="number" domain={['dataMin', 'dataMax']} tickFormatter={(v) => `${Number(v).toFixed(0)}s`} minTickGap={50} stroke={palette.axis} tickLine={false} fontSize={11} />
          <YAxis domain={domain ?? ['auto', 'auto']} stroke={palette.axis} tickLine={false} axisLine={false} fontSize={11} />
          <Tooltip
            contentStyle={{ background: palette.surface, border: `1px solid ${palette.border}`, borderRadius: 8, color: palette.text, fontSize: 12 }}
            labelFormatter={(v) => `t = ${Number(v).toFixed(2)} s`}
            formatter={(v) => [`${Number(v).toFixed(2)} ${unit}`, title]}
          />
          <Area type="monotone" dataKey="v" stroke={color} strokeWidth={2} fill={`url(#g-${title})`} isAnimationActive={false} connectNulls />
        </AreaChart>
      </ResponsiveContainer>
    </article>
  );
}
