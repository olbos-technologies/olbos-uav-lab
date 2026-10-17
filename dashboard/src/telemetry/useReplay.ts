import { useEffect, useState } from 'react';
import { parseSample, type Sample, type TelemetryEvent } from './types';
import { changes, MAX_EVENTS } from './useTelemetry';

export type Trace = { name: string; samples: Sample[]; events: TelemetryEvent[]; rejected: number };

export const MAX_TRACE_BYTES = 20 * 1024 * 1024;

// Parses a JSONL trace (uav_scenarios traces, harness output or a dashboard export).
// Lines that fail validation are counted and skipped rather than aborting the whole file.
export function parseTrace(name: string, text: string): Trace {
  const samples: Sample[] = [];
  let rejected = 0;
  for (const line of text.split(/\r?\n/)) {
    if (!line.trim()) continue;
    try {
      const sample = parseSample(line);
      samples.push({ ...sample, transport: sample.transport ?? 'FILE' });
    } catch {
      rejected++;
    }
  }
  if (!samples.length) throw new Error('No valid telemetry lines in file');
  samples.sort((a, b) => a.time_s - b.time_s);
  const events: TelemetryEvent[] = [];
  let id = 0;
  samples.forEach((s, i) => events.push(...changes(samples[i - 1], s, () => ++id)));
  return { name, samples, events, rejected };
}

function lastAtOrBefore(samples: Sample[], t: number) {
  let lo = 0;
  let hi = samples.length - 1;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (samples[mid].time_s <= t) lo = mid;
    else hi = mid - 1;
  }
  return lo;
}

// Plays a loaded trace on the trace's own clock: play/pause, speed and seek.
export function useReplay(trace: Trace | null, capacity = 600) {
  const start = trace?.samples[0].time_s ?? 0;
  const end = trace?.samples.at(-1)!.time_s ?? 0;
  const [t, setT] = useState(start);
  const [playing, setPlaying] = useState(true);
  const [speed, setSpeed] = useState(1);

  useEffect(() => {
    setT(start);
    setPlaying(true);
  }, [trace, start]);

  useEffect(() => {
    if (!trace || !playing) return;
    let last = performance.now();
    const timer = window.setInterval(() => {
      const now = performance.now();
      const dt = ((now - last) / 1000) * speed;
      last = now;
      setT((v) => Math.min(end, v + dt));
    }, 50);
    return () => window.clearInterval(timer);
  }, [trace, playing, speed, end]);

  useEffect(() => {
    if (playing && t >= end) setPlaying(false);
  }, [t, end, playing]);

  if (!trace) return null;
  const index = lastAtOrBefore(trace.samples, t);
  const rows = trace.samples.slice(Math.max(0, index + 1 - capacity), index + 1);
  const latest = trace.samples[index];
  // Newest first, matching the live event log.
  const events = trace.events.filter((e) => e.time_s <= latest.time_s).reverse().slice(0, MAX_EVENTS);
  return {
    rows,
    events,
    latest,
    stale: (latest.age_s ?? 0) > 2,
    t,
    start,
    end,
    playing,
    speed,
    setSpeed,
    toggle: () => {
      if (!playing && t >= end) setT(start);
      setPlaying((p) => !p);
    },
    seek: (v: number) => setT(Math.max(start, Math.min(end, v))),
  };
}
