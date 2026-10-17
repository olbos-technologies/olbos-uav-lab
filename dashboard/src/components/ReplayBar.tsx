const SPEEDS = [0.5, 1, 2, 5, 10];

type Props = {
  name: string;
  rejected: number;
  t: number;
  start: number;
  end: number;
  playing: boolean;
  speed: number;
  onToggle: () => void;
  onSeek: (t: number) => void;
  onSpeed: (s: number) => void;
  onExit: () => void;
};

export function ReplayBar({ name, rejected, t, start, end, playing, speed, onToggle, onSeek, onSpeed, onExit }: Props) {
  return (
    <section className="panel replay" aria-label="Trace replay">
      <button onClick={onToggle} aria-label={playing ? 'Pause' : 'Play'} className="replay-play">
        {playing ? '❚❚' : '▶'}
      </button>
      <div className="replay-track">
        <div className="replay-meta">
          <b title={name}>{name}</b>
          <span className="muted">
            {(t - start).toFixed(1)} / {(end - start).toFixed(1)} s
            {rejected > 0 && ` · ${rejected} invalid line${rejected === 1 ? '' : 's'} skipped`}
          </span>
        </div>
        <input
          type="range"
          min={start}
          max={end}
          step={0.05}
          value={t}
          onChange={(e) => onSeek(Number(e.target.value))}
          aria-label="Replay position"
        />
      </div>
      <label className="replay-speed">
        <span className="muted">Speed</span>
        <select value={speed} onChange={(e) => onSpeed(Number(e.target.value))}>
          {SPEEDS.map((s) => (
            <option key={s} value={s}>
              {s}×
            </option>
          ))}
        </select>
      </label>
      <button className="ghost" onClick={onExit}>
        Back to live
      </button>
    </section>
  );
}
