"""Render uav_scenarios' safety-report.json as Markdown and a self-contained HTML page.

Usage: python3 scripts/safety_report.py artifacts/scenarios/safety-report.json
Writes safety-report.md and safety-report.html next to the input. Standard library only.
"""
import html
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

DISCLAIMER = ('Kinematic simulation of the shared MissionController. The plant approximates a PX4 multicopter '
              '(position lag, velocity limits, partial wind rejection, Land descent, auto-disarm); it is not '
              'a physics model, PX4 SITL or flight evidence.')


def fmt(value, unit='', digits=2):
    return '—' if value is None else f'{value:.{digits}f}{unit}'


def observations(s):
    """Physical outcomes worth flagging even when the controller behaved correctly."""
    notes = []
    if s['max_distance_m'] is not None and s['geofence_m'] and s['max_distance_m'] > s['geofence_m']:
        notes.append(f"Airframe reached {s['max_distance_m']:.1f} m from home, beyond the "
                     f"{s['geofence_m']:.0f} m fence; the controller cannot overcome wind beyond control authority.")
    if (s['touchdown_speed_mps'] or 0) > 1.5:
        notes.append(f"Touchdown at {s['touchdown_speed_mps']:.1f} m/s: impact, not a controlled landing.")
    if s['truth_armed'] and not s['truth_landed'] and s['ever_armed']:
        notes.append('Vehicle still airborne and armed when the controller gave up; autopilot failsafe must own it.')
    return notes


def markdown(report, generated):
    rows = report['scenarios']
    lines = [
        '# Olbos UAV Lab — fault-injection safety report', '',
        f"Generated {generated}. **{report['passed']}/{len(rows)} scenarios passed.**", '',
        f'> {DISCLAIMER}', '',
        '| Result | Scenario | Category | Outcome | First alert | Detected after | Max distance | Touchdown |',
        '|---|---|---|---|---|---|---|---|',
    ]
    for s in rows:
        lines.append(f"| {'PASS' if s['pass'] else '**FAIL**'} | `{s['name']}` | {s['category']} | {s['outcome']} | "
                     f"{s['first_alert'] or '—'} | {fmt(s['detect_latency_s'], ' s')} | "
                     f"{fmt(s['max_distance_m'], ' m', 1)} | {fmt(s['touchdown_speed_mps'], ' m/s', 1)} |")
    lines += ['', '## Scenario details', '']
    for s in rows:
        lines += [f"### {'PASS' if s['pass'] else 'FAIL'} · {s['name']}", '', s['description'], '']
        for c in s['checks']:
            lines.append(f"- {'✅' if c['pass'] else '❌'} `{c['name']}` — {c['detail']}")
        for note in observations(s):
            lines.append(f'- ⚠️ {note}')
        lines.append('')
    return '\n'.join(lines)


CSS = """
:root{--bg:#f5f7fa;--card:#fff;--text:#122130;--muted:#5d6f80;--line:#dde5ed;--ok:#13875a;--bad:#d23737;--warn:#a86500;color-scheme:light dark}
@media (prefers-color-scheme:dark){:root{--bg:#08111b;--card:#0f1b28;--text:#e5eef6;--muted:#8399ad;--line:#20354a;--ok:#4ade9b;--bad:#ff6b6b;--warn:#f5b942}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.55 system-ui,-apple-system,'Segoe UI',sans-serif}
main{max-width:1100px;margin:0 auto;padding:32px 16px 48px}h1{font-size:24px;margin:0 0 4px}h2{font-size:16px;margin:32px 0 12px}
.muted{color:var(--muted)}.summary{display:flex;gap:12px;flex-wrap:wrap;margin:20px 0}
.stat{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px 18px;min-width:150px}
.stat b{display:block;font-size:26px;font-variant-numeric:tabular-nums}.note{border-left:3px solid var(--warn);background:var(--card);padding:10px 14px;border-radius:6px;color:var(--muted)}
.table{overflow-x:auto;background:var(--card);border:1px solid var(--line);border-radius:10px}
table{width:100%;border-collapse:collapse;font-size:13px}th,td{text-align:left;padding:8px 9px;border-bottom:1px solid var(--line);white-space:nowrap}
th{color:var(--muted);font-weight:600;font-size:11.5px;text-transform:uppercase;letter-spacing:.06em}tr:last-child td{border-bottom:0}
.tag{display:inline-block;padding:2px 9px;border-radius:999px;font-weight:600;font-size:12px}.pass{color:var(--ok);background:color-mix(in srgb,var(--ok) 13%,transparent)}
.fail{color:var(--bad);background:color-mix(in srgb,var(--bad) 13%,transparent)}code{font:12.5px ui-monospace,Menlo,monospace}
details{background:var(--card);border:1px solid var(--line);border-radius:10px;margin:8px 0;padding:10px 14px}summary{cursor:pointer;font-weight:600}
ul{margin:10px 0 4px;padding-left:20px}li{margin:3px 0}.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}
"""


def page(report, generated):
    rows = report['scenarios']
    e = html.escape
    failed = report['failed']
    table = ''.join(
        f"<tr><td><span class='tag {'pass' if s['pass'] else 'fail'}'>{'PASS' if s['pass'] else 'FAIL'}</span></td>"
        f"<td><code>{e(s['name'])}</code></td><td>{e(s['category'])}</td><td>{e(s['outcome'])}</td>"
        f"<td>{e(s['first_alert'] or '—')}</td><td>{fmt(s['detect_latency_s'], ' s')}</td>"
        f"<td>{fmt(s['max_distance_m'], ' m', 1)}</td><td>{fmt(s['touchdown_speed_mps'], ' m/s', 1)}</td></tr>"
        for s in rows)
    details = ''.join(
        f"<details{' open' if not s['pass'] or observations(s) else ''}><summary>"
        f"<span class='{'ok' if s['pass'] else 'bad'}'>{'PASS' if s['pass'] else 'FAIL'}</span> · {e(s['name'])}</summary>"
        f"<p class='muted'>{e(s['description'])}</p><ul>"
        + ''.join(f"<li class='{'ok' if c['pass'] else 'bad'}'>{'✓' if c['pass'] else '✗'} <code>{e(c['name'])}</code> "
                  f"<span class='muted'>— {e(c['detail'])}</span></li>" for c in s['checks'])
        + ''.join(f"<li class='warn'>⚠ {e(n)}</li>" for n in observations(s))
        + '</ul></details>' for s in rows)
    flagged = sum(1 for s in rows if observations(s))
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>UAV Safety Report</title><style>{CSS}</style></head><body><main>
<h1>Fault-injection safety report</h1><div class="muted">Olbos UAV Lab · generated {e(generated)}</div>
<div class="summary"><div class="stat"><span class="muted">Scenarios</span><b>{len(rows)}</b></div>
<div class="stat"><span class="muted">Passed</span><b class="ok">{report['passed']}</b></div>
<div class="stat"><span class="muted">Failed</span><b class="{'bad' if failed else 'ok'}">{failed}</b></div>
<div class="stat"><span class="muted">Physical warnings</span><b class="{'warn' if flagged else 'ok'}">{flagged}</b></div></div>
<p class="note">{e(DISCLAIMER)}</p>
<h2>Results</h2><div class="table"><table><thead><tr><th>Result</th><th>Scenario</th><th>Category</th><th>Outcome</th>
<th>First alert</th><th>Detected after</th><th>Max distance</th><th>Touchdown</th></tr></thead><tbody>{table}</tbody></table></div>
<h2>Checks per scenario</h2>{details}</main></body></html>"""


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    source = Path(sys.argv[1])
    report = json.loads(source.read_text())
    generated = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')
    (source.parent / 'safety-report.md').write_text(markdown(report, generated) + '\n')
    (source.parent / 'safety-report.html').write_text(page(report, generated))
    print(f"{report['passed']}/{len(report['scenarios'])} passed -> {source.parent / 'safety-report.html'}")


if __name__ == '__main__':
    main()
