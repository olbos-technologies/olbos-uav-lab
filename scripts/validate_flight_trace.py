#!/usr/bin/env python3
"""Read-only JSONL trace acceptance checker. Kinematic evidence is NOT PX4 SITL evidence."""
import argparse
import json
import math
from pathlib import Path

REQUIRED = ("source", "time_s", "east", "north", "altitude", "battery", "state", "outcome")
NUMERIC = ("time_s", "east", "north", "altitude", "battery")
TERMINAL = {"COMPLETE", "ABORTED_LANDED", "FAILED"}


def validate_trace(path: Path, expected_source="KINEMATIC_HARNESS", expected_outcome="COMPLETE"):
    errors, rows = [], []
    with path.open(encoding="utf-8") as handle:
        for line_no, raw in enumerate(handle, 1):
            if not raw.strip():
                continue
            try:
                row = json.loads(raw)
            except json.JSONDecodeError as exc:
                errors.append(f"line {line_no}: invalid JSON ({exc.msg})")
                continue
            if not isinstance(row, dict):
                errors.append(f"line {line_no}: expected JSON object")
                continue
            missing = [key for key in REQUIRED if key not in row]
            if missing:
                errors.append(f"line {line_no}: missing {missing}")
                continue
            if row["source"] != expected_source:
                errors.append(f'line {line_no}: unexpected source {row["source"]!r}')
            for key in NUMERIC:
                val = row[key]
                if type(val) not in (int, float) or not math.isfinite(val):
                    errors.append(f"line {line_no}: invalid numeric {key}")
            rows.append(row)
    if not rows:
        errors.append("no valid samples")
        return {"passed": False, "sample_count": 0, "errors": errors}
    previous, terminal_indices = None, []
    for i, row in enumerate(rows):
        t = row["time_s"]
        if type(t) in (int, float) and math.isfinite(t):
            if previous is not None and t <= previous:
                errors.append(f"sample {i}: non-increasing timestamp")
            previous = t
        battery = row["battery"]
        if type(battery) in (int, float) and math.isfinite(battery) and not 0 <= battery <= 1:
            errors.append(f"sample {i}: battery outside [0,1]")
        altitude = row["altitude"]
        if type(altitude) in (int, float) and math.isfinite(altitude) and altitude < -0.05:
            errors.append(f"sample {i}: negative altitude")
        if row["outcome"] in TERMINAL:
            terminal_indices.append(i)
    if not terminal_indices:
        errors.append("no terminal outcome observed")
    elif terminal_indices != [len(rows) - 1]:
        errors.append("terminal outcome must occur exactly once, at last sample")
    if rows[-1]["outcome"] != expected_outcome:
        errors.append(f'last outcome {rows[-1]["outcome"]!r} != {expected_outcome!r}')
    return {
        "passed": not errors,
        "evidence_class": "KINEMATIC_HARNESS_ONLY" if expected_source == "KINEMATIC_HARNESS" else "UNVERIFIED_SOURCE",
        "sample_count": len(rows),
        "last_outcome": rows[-1]["outcome"],
        "elapsed_s": rows[-1]["time_s"],
        "errors": errors,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--source", default="KINEMATIC_HARNESS")
    parser.add_argument("--outcome", default="COMPLETE")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    try:
        result = validate_trace(args.trace, args.source, args.outcome)
    except OSError as exc:
        result = {"passed": False, "sample_count": 0, "errors": [str(exc)]}
    output = json.dumps(result, indent=2, allow_nan=False)
    print(output)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(output + "\n", encoding="utf-8")
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
