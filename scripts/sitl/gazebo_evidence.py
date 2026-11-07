#!/usr/bin/env python3
import argparse
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

OUTCOMES = {0: "RUNNING", 1: "COMPLETE", 2: "ABORTED_LANDED", 3: "FAILED"}
EXPECTED = {
    ("smoke", 1): ("COMPLETE", "NORMAL"),
    ("smoke", 2): ("COMPLETE", "NORMAL"),
    ("waypoint", 1): ("COMPLETE", "NORMAL"),
    ("waypoint", 2): ("COMPLETE", "NORMAL"),
    ("abort", 1): ("ABORTED_LANDED", "OPERATOR_ABORT"),
    ("abort", 2): ("ABORTED_LANDED", "OPERATOR_ABORT"),
    ("gnss_loss", 1): ("ABORTED_LANDED", "INVALID_POSITION"),
    ("position_jump", 1): ("ABORTED_LANDED", "TRACKING_ERROR"),
    ("stale_telemetry", 1): ("FAILED", "LANDING_UNVERIFIED"),
    ("geofence", 1): ("ABORTED_LANDED", "GEOFENCE"),
}
FAULT_REASONS = {
    "gnss_loss": "INVALID_POSITION",
    "position_jump": "TRACKING_ERROR",
    "stale_telemetry": "STALE_TELEMETRY",
    "geofence": "GEOFENCE",
}
STATE_RE = re.compile(
    r"\bSTATE (?P<state>[A-Z_]+) outcome=(?P<code>[0-3]) reason=(?P<reason>[A-Z_]+)"
)
KV_RE = re.compile(r"([a-z0-9_]+)=([A-Za-z0-9_.-]+)")


def read_text(path):
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def parse_controller_log(path, scenario):
    text = read_text(path)
    states = list(STATE_RE.finditer(text))
    state = states[-1].groupdict() if states else {}
    evidence_line = next((line for line in reversed(text.splitlines()) if "EVIDENCE " in line), "")
    evidence = dict(KV_RE.findall(evidence_line))
    fault_line = next(
        (line for line in text.splitlines() if f"INJECTED_FAULT scenario={scenario} " in line), ""
    )
    fault_marker = dict(KV_RE.findall(fault_line))
    outcome = OUTCOMES.get(int(state.get("code", -1)), "UNKNOWN")
    return {
        "state": state.get("state", "UNKNOWN"),
        "outcome": outcome,
        "reason": state.get("reason", "UNKNOWN"),
        "land_request_observed": bool(re.search(r"\bREQUEST command=21\b", text)),
        "terminal_evidence": evidence,
        "fault_marker": fault_marker,
    }


def record_run(args):
    status_text = read_text(args.status)
    land_text = read_text(args.land)
    arming_match = re.search(r"\barming_state:\s*(\d+)", status_text)
    landed_match = re.search(r"\blanded:\s*(true|false)", land_text, re.IGNORECASE)
    parsed = parse_controller_log(args.log, args.scenario)
    evidence = parsed["terminal_evidence"]
    abort_text = read_text(args.abort)
    trace_lines = read_text(args.trace).splitlines() if args.trace else []
    valid_trace_samples = []
    for line in trace_lines:
        try:
            sample = json.loads(line)
        except json.JSONDecodeError:
            continue
        if sample.get("tag") == "GAZEBO_SITL" and sample.get("scenario") in {
            "none",
            args.scenario,
        }:
            valid_trace_samples.append(sample)
    fault_reason = FAULT_REASONS.get(args.scenario)
    fault_seen_in_trace = False
    streaming_after_fault = False
    for sample in valid_trace_samples:
        if sample.get("reason") == fault_reason:
            fault_seen_in_trace = True
        if fault_seen_in_trace and sample.get("stream") is True:
            streaming_after_fault = True
    return {
        "mission": args.scenario,
        "repeat": args.repeat,
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        **parsed,
        "vehicle_status_arming_state": int(arming_match.group(1)) if arming_match else None,
        "vehicle_land_detected": landed_match.group(1).lower() == "true" if landed_match else None,
        "actual_landed_disarmed": (
            arming_match is not None
            and int(arming_match.group(1)) == 1
            and landed_match is not None
            and landed_match.group(1).lower() == "true"
        ),
        "abort_service_accepted": "success: true" in abort_text,
        "trace_file": str(Path(args.trace).name) if args.trace else None,
        "trace_samples": len(valid_trace_samples),
        "fault_seen_in_trace": fault_seen_in_trace,
        "streaming_after_fault": streaming_after_fault,
    }


def validate_run(run, expected, errors):
    key = (run.get("mission"), run.get("repeat"))
    if key not in EXPECTED:
        errors.append(f"unexpected run: {key}")
        return
    expected_outcome, expected_reason = expected
    if (run.get("outcome"), run.get("reason")) != expected:
        errors.append(
            f"{key}: expected {expected_outcome}/{expected_reason}, "
            f"got {run.get('outcome')}/{run.get('reason')}"
        )
    if run.get("actual_landed_disarmed") is not True:
        errors.append(f"{key}: PX4 VehicleStatus/VehicleLandDetected did not confirm disarmed and landed")
    if run.get("land_request_observed") is not True:
        errors.append(f"{key}: controller Land request was not observed")
    if not run.get("recorded_at_utc"):
        errors.append(f"{key}: missing run timestamp")
    if not run.get("trace_file") or run.get("trace_samples", 0) < 10:
        errors.append(f"{key}: missing raw/controller timing and position trace")
    evidence = run.get("terminal_evidence") or {}
    if evidence.get("tag") != "GAZEBO_SITL":
        errors.append(f"{key}: missing GAZEBO_SITL terminal provenance")
    if evidence.get("px4_armed") != "0" or evidence.get("px4_landed") != "1":
        errors.append(f"{key}: terminal PX4 state was not observed disarmed and landed")
    if evidence.get("px4_position_valid") not in {"0", "1"}:
        errors.append(f"{key}: missing raw PX4 position validity")
    if evidence.get("controller_position_valid") not in {"0", "1"}:
        errors.append(f"{key}: missing controller-observed position validity")
    if run.get("mission") == "abort" and run.get("abort_service_accepted") is not True:
        errors.append(f"{key}: operator abort service was not observed as accepted")
    if run.get("mission") in {"gnss_loss", "position_jump", "stale_telemetry", "geofence"}:
        marker = run.get("fault_marker") or {}
        if marker.get("source") != "ROS_ADAPTER_OBSERVATION":
            errors.append(f"{key}: injected fault source was not recorded at the ROS adapter boundary")
        if marker.get("raw_px4_untouched") != "true":
            errors.append(f"{key}: raw PX4 estimator observations were not explicitly kept untouched")
        if evidence.get("fault") != run.get("mission") or evidence.get("fault_injected") != "1":
            errors.append(f"{key}: terminal fault provenance is missing or mismatched")
        if run.get("fault_seen_in_trace") is not True:
            errors.append(f"{key}: injected fault was not observed in the timing trace")
        if run.get("streaming_after_fault") is True:
            errors.append(f"{key}: controller continued setpoint streaming after detecting the fault")
        try:
            px4_east = float(evidence["px4_position_east"])
            controller_east = float(evidence["controller_position_east"])
            raw_age = float(evidence["px4_telemetry_age"])
            controller_age = float(evidence["controller_telemetry_age"])
        except (KeyError, ValueError):
            errors.append(f"{key}: missing raw/controller timing or position evidence")
        else:
            east_offset = controller_east - px4_east
            if run.get("mission") == "gnss_loss":
                if evidence.get("px4_position_valid") != "1" or evidence.get(
                    "controller_position_valid"
                ) != "0":
                    errors.append(f"{key}: raw and controller GNSS validity were not distinguished")
            elif run.get("mission") == "position_jump" and abs(east_offset - 6.0) > 0.02:
                errors.append(f"{key}: controller position did not retain the 6 m injected jump")
            elif run.get("mission") == "geofence" and east_offset < 44.9:
                errors.append(f"{key}: controller geofence offset was not recorded")
            elif run.get("mission") == "stale_telemetry" and (raw_age > 2 or controller_age <= 2):
                errors.append(f"{key}: raw telemetry freshness was not distinguished from stale controller data")
    elif evidence.get("fault_injected") != "0":
        errors.append(f"{key}: nominal/operator-abort run unexpectedly injected a sensor fault")


def build_report(document, errors):
    rows = []
    for run in sorted(document.get("runs", []), key=lambda row: (row.get("mission", ""), row.get("repeat", 0))):
        evidence = run.get("terminal_evidence") or {}
        key = (run.get("mission"), run.get("repeat"))
        state = (
            "PASS"
            if key in EXPECTED
            and (run.get("outcome"), run.get("reason"))
            == EXPECTED[key]
            and run.get("actual_landed_disarmed")
            and evidence.get("tag") == "GAZEBO_SITL"
            and evidence.get("px4_armed") == "0"
            and evidence.get("px4_landed") == "1"
            and run.get("trace_samples", 0) >= 10
            and not any(error.startswith(f"{key}:") for error in errors)
            else "FAIL"
        )
        rows.append(
            f"| {run.get('mission')} #{run.get('repeat')} | {run.get('recorded_at_utc', 'UNKNOWN')} | "
            f"{run.get('outcome')} / {run.get('reason')} | {run.get('land_request_observed')} | "
            f"{run.get('actual_landed_disarmed')} | {state} |"
        )
    errors_text = "\n".join(f"- {error}" for error in errors) or "- None"
    return "\n".join(
        [
            "# Gazebo x500 SITL evidence",
            "",
            f"- Claim tag: `{document.get('claim_tag', 'UNKNOWN')}`",
            f"- Simulator: `{document.get('simulator', 'UNKNOWN')}`",
            f"- Report generated (UTC): `{document.get('generated_at_utc', 'UNKNOWN')}`",
            "- PX4 source: `PX4-Autopilot v1.16.0`; ROS 2: `Humble`; Gazebo: `Harmonic`.",
            "- Sensor faults are injected into the ROS adapter's controller observation only. The raw PX4 estimator topics remain separately recorded; this is not Gazebo sensor-model or PX4-estimator fault injection.",
            "- `FAILED / LANDING_UNVERIFIED` is an explicit rejected controller completion, even if the independent PX4 state observer later confirms landing and disarm.",
            "",
            "- Recorded parameters: `UXRCE_DDS_DOM_ID=42`, `COM_OF_LOSS_T=1`, `COM_OBL_RC_ACT=4`, `COM_RC_IN_MODE=4` (see `params.txt` and `versions.txt`).",
            "",
            "| Run | Timestamp (UTC) | Controller outcome / reason | Land requested | PX4 landed and disarmed | Invariant |",
            "|---|---|---|---|---|---|",
            *rows,
            "",
            "## Rejected or unknown states",
            errors_text,
            "",
            "Full per-run ROS logs, 20 Hz raw/controller position traces, PX4 ULogs, parameter settings, and resolved package/source manifest are retained beside this report.",
            "",
        ]
    )


def validate(args):
    run_paths = sorted(Path(args.input).glob("runs/*.json"))
    runs = [json.loads(path.read_text(encoding="utf-8")) for path in run_paths]
    errors = []
    params = read_text(Path(args.input) / "params.txt")
    readback = read_text(Path(args.input) / "params-readback.txt")
    for name, value in (
        ("UXRCE_DDS_DOM_ID", "42"),
        ("COM_OF_LOSS_T", "1"),
        ("COM_OBL_RC_ACT", "4"),
        ("COM_RC_IN_MODE", "4"),
    ):
        setting = f"{name}={value}"
        if f"configured {setting}" not in params:
            errors.append(f"missing recorded PX4 parameter: {setting}")
        if not any(
            name in line
            and re.search(rf"(?<![\d.]){value}(?:\.0+)?(?![\d.])", line)
            for line in readback.splitlines()
        ):
            errors.append(f"PX4 parameter readback did not confirm {setting}")
    versions = read_text(Path(args.input) / "versions.txt")
    for source in ("claim_tag=GAZEBO_SITL", "simulator=gz_x500", "px4_ref=v1.16.0", "agent_ref=v2.4.3"):
        if source not in versions:
            errors.append(f"missing source/version provenance: {source}")
    if len(runs) != len(EXPECTED):
        errors.append(f"expected {len(EXPECTED)} mission runs, found {len(runs)}")
    seen = set()
    for run in runs:
        key = (run.get("mission"), run.get("repeat"))
        if key in seen:
            errors.append(f"duplicate run: {key}")
        seen.add(key)
        validate_run(run, EXPECTED.get(key, ("UNKNOWN", "UNKNOWN")), errors)
    missing = set(EXPECTED) - seen
    for key in sorted(missing):
        errors.append(f"missing run: {key}")
    document = {
        "claim_tag": "GAZEBO_SITL",
        "simulator": "gz_x500",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "parameter_manifest": "params.txt",
        "source_manifest": "versions.txt",
        "runs": runs,
        "invariant_errors": errors,
    }
    out = Path(args.output)
    out.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    Path(args.summary).write_text(build_report(document, errors), encoding="utf-8")
    print(build_report(document, errors), end="")
    return 1 if errors else 0


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    record = subparsers.add_parser("record")
    record.add_argument("--scenario", required=True)
    record.add_argument("--repeat", type=int, required=True)
    record.add_argument("--log", required=True)
    record.add_argument("--status", required=True)
    record.add_argument("--land", required=True)
    record.add_argument("--abort", default="")
    record.add_argument("--trace", default="")
    record.add_argument("--output", required=True)
    check = subparsers.add_parser("validate")
    check.add_argument("--input", required=True)
    check.add_argument("--output", required=True)
    check.add_argument("--summary", required=True)
    args = parser.parse_args()
    if args.command == "record":
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(json.dumps(record_run(args), indent=2) + "\n", encoding="utf-8")
        return 0
    return validate(args)


if __name__ == "__main__":
    sys.exit(main())
