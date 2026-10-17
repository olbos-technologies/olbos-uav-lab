import { useEffect, useState } from 'react';

export type ThemeChoice = 'system' | 'light' | 'dark';
const KEY = 'olbos-theme';

function stored(): ThemeChoice {
  try {
    const v = localStorage.getItem(KEY);
    return v === 'light' || v === 'dark' ? v : 'system';
  } catch {
    return 'system';
  }
}

export function useTheme() {
  const [choice, setChoice] = useState<ThemeChoice>(stored);
  const [systemDark, setSystemDark] = useState(
    () => window.matchMedia?.('(prefers-color-scheme: dark)').matches ?? true,
  );
  useEffect(() => {
    const q = window.matchMedia?.('(prefers-color-scheme: dark)');
    const on = (e: MediaQueryListEvent) => setSystemDark(e.matches);
    q?.addEventListener('change', on);
    return () => q?.removeEventListener('change', on);
  }, []);
  const resolved = choice === 'system' ? (systemDark ? 'dark' : 'light') : choice;
  useEffect(() => {
    document.documentElement.dataset.theme = resolved;
  }, [resolved]);
  const cycle = () => {
    const next: ThemeChoice = choice === 'system' ? 'light' : choice === 'light' ? 'dark' : 'system';
    setChoice(next);
    try {
      localStorage.setItem(KEY, next);
    } catch {
      /* storage unavailable: keep in-memory choice */
    }
  };
  return { choice, resolved, cycle };
}

export type Palette = { accent: string; accentFill: string; ok: string; warn: string; grid: string; axis: string; surface: string; border: string; text: string };

export const palettes: Record<'light' | 'dark', Palette> = {
  dark: { accent: '#4cc2ff', accentFill: 'rgba(76,194,255,.18)', ok: '#4ade9b', warn: '#f5b942', grid: '#1b2b3d', axis: '#6b829a', surface: '#0f1b2a', border: '#22364c', text: '#e6eef6' },
  light: { accent: '#0a78c2', accentFill: 'rgba(10,120,194,.14)', ok: '#13875a', warn: '#b76e00', grid: '#e3e9f0', axis: '#6a7b8d', surface: '#ffffff', border: '#d7e0ea', text: '#0f1d2b' },
};
