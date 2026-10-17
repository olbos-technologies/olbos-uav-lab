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
    if (s.get('estimate_error_m') or 0) > 1 and s['outcome'] == 'COMPLETE':
        notes.append(f"Mission reported COMPLETE while the airframe was up to {s['estimate_error_m']:.1f} m from where "
                     'it believed it was. Mission-level monitoring cannot see estimator faults; PX4 EKF2 '
                     'innovation checks (or an independent position source) must catch them.')
    if s['truth_armed'] and not s['truth_landed'] and s['ever_armed']:
        notes.append('Vehicle still airborne and armed when the controller gave up; autopilot failsafe must own it.')
    return notes


STATUS = {  # fixed status palette; icon + label always accompany the colour
    'good': ('#0ca30c', '✓', 'Completed'),
    'warning': ('#fab219', '▲', 'Aborted, landed'),
    'critical': ('#d03b3b', '✕', 'Failed or unsafe'),
}


def status(point):
    if not point['safe'] or point['outcome'] == 'FAILED':
        return 'critical'
    return 'good' if point['outcome'] == 'COMPLETE' else 'warning'


def sweep_summary(sw):
    """One plain-language sentence: how far the mission keeps completing, and what happens beyond."""
    pts, unit = sw['points'], sw['unit']
    clean = [p['value'] for p in pts if p['outcome'] == 'COMPLETE']
    limit = None
    for p in pts:
        if p['outcome'] != 'COMPLETE':
            break
        limit = p['value']
    beyond = [p for p in pts if limit is None or p['value'] > limit]
    alerts = sorted({p['first_alert'] for p in beyond if p['first_alert']})
    unsafe = sum(1 for p in pts if not p['safe'])
    worst = max(pts, key=lambda p: p['max_distance_m'] or 0)
    text = (f"Completes up to {limit:g} {unit}" if limit is not None else 'Never completes') + \
        (f"; beyond that the controller stops it ({', '.join(alerts)})" if alerts else '') + '.'
    late = [p for p in pts if p['outcome'] == 'COMPLETE' and p['estimate_error_m'] and p['estimate_error_m'] > 1]
    if late:
        text += (f" Up to {max(p['value'] for p in late):g} {unit} goes unnoticed: the airframe lands up to "
                 f"{max(p['estimate_error_m'] for p in late):.1f} m from where it believes it is.")
    if worst['max_distance_m'] and worst['geofence_m'] and worst['max_distance_m'] > worst['geofence_m']:
        text += f" Worst drift {worst['max_distance_m']:.0f} m from home (fence {worst['geofence_m']:.0f} m)."
    text += ' All points kept the safety invariants.' if not unsafe else f' {unsafe} point(s) BROKE a safety invariant.'
    return text, len(clean)


def sweep_chart(sw):
    """Dot plot: fault magnitude vs maximum distance from home, coloured by outcome status."""
    e = html.escape
    pts = sw['points']
    w, h, left, right, top, bottom = 920, 230, 44, 14, 22, 36
    xs = [p['value'] for p in pts]
    fence = pts[0]['geofence_m'] or 0
    ymax = max([p['max_distance_m'] or 0 for p in pts] + [fence]) * 1.12
    x0, x1 = min(xs), max(xs)
    sx = lambda v: left + (v - x0) / ((x1 - x0) or 1) * (w - left - right)
    sy = lambda v: top + (1 - v / (ymax or 1)) * (h - top - bottom)
    step = 10 if ymax <= 60 else 20
    grid = ''.join(
        f"<line class='grid' x1='{left}' x2='{w - right}' y1='{sy(v):.1f}' y2='{sy(v):.1f}'/>"
        f"<text class='tick' x='{left - 6}' y='{sy(v) + 4:.1f}' text-anchor='end'>{v:g}</text>"
        for v in range(0, int(ymax) + 1, step))
    xticks = ''.join(
        f"<text class='tick' x='{sx(v):.1f}' y='{h - bottom + 16}' text-anchor='middle'>{v:g}</text>"
        for i, v in enumerate(xs) if i % max(1, len(xs) // 10) == 0 or i == len(xs) - 1)
    fence_line = (f"<line class='fence' x1='{left}' x2='{w - right}' y1='{sy(fence):.1f}' y2='{sy(fence):.1f}'/>"
                  f"<text class='fence-label' x='{left + 6}' y='{sy(fence) - 5:.1f}'>"
                  f"fence {fence:g} m</text>") if fence else ''
    dots = ''
    for p in pts:
        st = status(p)
        tip = (f"{sw['parameter']} {p['value']:g} {sw['unit']}: {STATUS[st][2]} ({p['outcome']}), "
               f"alert {p['first_alert'] or 'none'}, max distance {p['max_distance_m']:.1f} m")
        dots += (f"<g class='dot'><title>{e(tip)}</title>"
                 f"<circle class='hit' cx='{sx(p['value']):.1f}' cy='{sy(p['max_distance_m'] or 0):.1f}' r='11'/>"
                 f"<circle class='st-{st}' cx='{sx(p['value']):.1f}' cy='{sy(p['max_distance_m'] or 0):.1f}' r='5'/></g>")
    return (f"<svg class='sweep-chart' viewBox='0 0 {w} {h}' role='img' "
            f"aria-label='{e(sw['parameter'])} versus maximum distance from home'>"
            f"{grid}{fence_line}<line class='axis' x1='{left}' x2='{w - right}' y1='{sy(0):.1f}' y2='{sy(0):.1f}'/>"
            f"{xticks}<text class='axis-title' x='{(left + w - right) / 2}' y='{h - 4}' text-anchor='middle'>"
            f"{e(sw['parameter'])} ({e(sw['unit'])})</text>"
            f"<text class='axis-title' x='4' y='12'>max distance from home (m)</text>{dots}</svg>")


def markdown(report, generated):
    rows = report['scenarios']
    lines = [
        '# Olbos UAV Lab — fault-injection safety report', '',
        f"Generated {generated}. **{report['passed']}/{len(rows)} scenarios passed.**", '',
        f'> {DISCLAIMER}', '',
        f"Mission: {report['config']['source']} ({report['config']['waypoints']} waypoints, "
        f"tracking limit {report['config']['tracking_limit_m']:g} m / {report['config']['tracking_seconds']:g} s, "
        f"stale after {report['config']['stale_seconds']:g} s). Faults injected at {report['config']['fault_time_s']:g} s.", '',
        '| Result | Scenario | Category | Outcome | First alert | Detected after | Max distance | Touchdown |',
        '|---|---|---|---|---|---|---|---|',
    ]
    for s in rows:
        lines.append(f"| {'PASS' if s['pass'] else '**FAIL**'} | `{s['name']}` | {s['category']} | {s['outcome']} | "
                     f"{s['first_alert'] or '—'} | {fmt(s['detect_latency_s'], ' s')} | "
                     f"{fmt(s['max_distance_m'], ' m', 1)} | {fmt(s['touchdown_speed_mps'], ' m/s', 1)} |")
    if report.get('sweeps'):
        lines += ['', '## Safety envelope sweeps', '']
        for sw in report['sweeps']:
            lines.append(f"- **{sw['parameter']}** — {sweep_summary(sw)[0]}")
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
:root{--bg:#f5f7fa;--card:#fff;--text:#122130;--muted:#5d6f80;--line:#dde5ed;--grid:#e8edf2;--ok:#13875a;--bad:#d23737;--warn:#a86500;color-scheme:light dark}
@media (prefers-color-scheme:dark){:root{--bg:#08111b;--card:#0f1b28;--text:#e5eef6;--muted:#8399ad;--line:#20354a;--grid:#1b2c3d;--ok:#4ade9b;--bad:#ff6b6b;--warn:#f5b942}}
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
ul{margin:10px 0 4px;padding-left:20px}
h3{font-size:14px;margin:0 0 4px}.sweep{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:16px;margin:12px 0}
.sweep>p{margin:6px 0}.sweep-chart{display:block;width:100%;max-width:920px;height:auto;margin:8px 0}
.sweep-chart .grid{stroke:var(--grid);stroke-width:1}.sweep-chart .axis{stroke:var(--muted);stroke-width:1}
.sweep-chart .tick,.sweep-chart .axis-title,.sweep-chart .fence-label{fill:var(--muted);font-size:11px}
.sweep-chart .fence{stroke:var(--muted);stroke-width:1.5;stroke-dasharray:5 4}
.sweep-chart circle:not(.hit){stroke:var(--card);stroke-width:2}.sweep-chart .hit{fill:transparent}
.sweep-chart .dot:hover circle:not(.hit){r:7}.st-good{fill:#0ca30c}.st-warning{fill:#fab219}.st-critical{fill:#d03b3b}
.legend{display:flex;gap:16px;flex-wrap:wrap;font-size:12.5px;color:var(--muted);margin:6px 0 4px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px;vertical-align:-1px}
.legend .fence-key{width:18px;height:0;border-radius:0;border-top:1.5px dashed var(--muted);vertical-align:3px}li{margin:3px 0}.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}
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
    legend = ''.join(f"<span><i style='background:{c}'></i>{icon} {label}</span>" for c, icon, label in STATUS.values())
    sweeps = ''
    if report.get('sweeps'):
        sweeps = ('<h2>Safety envelope</h2><p class="muted">Each fault swept across a range on this mission. '
                  'Hover a point for details.</p>'
                  f'<div class="legend">{legend}<span><i class="fence-key"></i>geofence</span></div>')
        for sw in report['sweeps']:
            text, _ = sweep_summary(sw)
            table = ''.join(
                f"<tr><td>{p['value']:g}</td><td>{STATUS[status(p)][1]} {e(p['outcome'])}</td><td>{e(p['first_alert'] or '—')}</td>"
                f"<td>{fmt(p['detect_latency_s'], ' s')}</td><td>{fmt(p['max_distance_m'], ' m', 1)}</td>"
                f"<td>{fmt(p['estimate_error_m'], ' m', 1)}</td></tr>" for p in sw['points'])
            sweeps += (f"<section class='sweep'><h3>{e(sw['parameter']).capitalize()}</h3>"
                       f"<p class='muted'>{e(sw['description'])}</p><p>{e(text)}</p>{sweep_chart(sw)}"
                       f"<details><summary>Table view</summary><div class='table'><table><thead><tr>"
                       f"<th>{e(sw['unit'])}</th><th>Outcome</th><th>First alert</th><th>Detected after</th>"
                       f"<th>Max distance</th><th>Position error</th></tr></thead><tbody>{table}</tbody></table></div>"
                       f"</details></section>")
    cfg = report.get('config', {})
    mission = (f"Mission: {e(str(cfg.get('source', '')))} · {cfg.get('waypoints', '?')} waypoints · fence {cfg.get('geofence_m', 0):g} m · "
               f"tracking {cfg.get('tracking_limit_m', 0):g} m / {cfg.get('tracking_seconds', 0):g} s · "
               f"faults at {cfg.get('fault_time_s', 0):g} s") if cfg else ''
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>UAV Safety Report</title><style>{CSS}</style></head><body><main>
<h1>Fault-injection safety report</h1><div class="muted">Olbos UAV Lab · generated {e(generated)}</div>
<div class="muted">{mission}</div>
<div class="summary"><div class="stat"><span class="muted">Scenarios</span><b>{len(rows)}</b></div>
<div class="stat"><span class="muted">Passed</span><b class="ok">{report['passed']}</b></div>
<div class="stat"><span class="muted">Failed</span><b class="{'bad' if failed else 'ok'}">{failed}</b></div>
<div class="stat"><span class="muted">Physical warnings</span><b class="{'warn' if flagged else 'ok'}">{flagged}</b></div>
<div class="stat"><span class="muted">Unsafe sweep points</span><b class="{'bad' if report.get('unsafe_sweep_points') else 'ok'}">{report.get('unsafe_sweep_points', 0)}</b></div></div>
<p class="note">{e(DISCLAIMER)}</p>
<h2>Results</h2><div class="table"><table><thead><tr><th>Result</th><th>Scenario</th><th>Category</th><th>Outcome</th>
<th>First alert</th><th>Detected after</th><th>Max distance</th><th>Touchdown</th></tr></thead><tbody>{table}</tbody></table></div>
{sweeps}<h2>Checks per scenario</h2>{details}</main></body></html>"""


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
