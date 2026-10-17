"""Loopback-only interactive kinematic dashboard demo.

This is a UI and telemetry demo. It does not connect to PX4, ROS, or a vehicle.
Commands are applied only to the in-memory simulation below.
"""
import argparse
import asyncio
import json
import time
from pathlib import Path
from typing import Callable

from websockets.asyncio.server import serve

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "artifacts" / "health_rf.joblib"
ALLOWED_ORIGINS = {"http://127.0.0.1:5173", "http://localhost:5173"}
FAULT_LABELS = {
    0: "NORMAL",
    1: "LOW_BATTERY_PATTERN",
    2: "VIBRATION_PATTERN",
    3: "POSITION_ERROR_PATTERN",
}
WAYPOINTS = ((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 0.0))
STEP_S = 0.2


def load_predictor() -> Callable[[list[float]], tuple[str, float]]:
    """Load only the model produced by this repository's synthetic trainer."""
    try:
        import joblib
    except ImportError as exc:
        raise RuntimeError(
            "Install ML dependencies with: python3 -m pip install -r machine_learning/requirements.txt"
        ) from exc
    if not MODEL_PATH.is_file():
        raise RuntimeError(
            "Synthetic model is missing. Run: python3 machine_learning/training/train.py"
        )
    model = joblib.load(MODEL_PATH)

    def predict(features: list[float]) -> tuple[str, float]:
        probabilities = model.predict_proba([features])[0]
        best = int(probabilities.argmax())
        label = FAULT_LABELS.get(int(model.classes_[best]), "UNKNOWN_PATTERN")
        return label, float(probabilities[best])

    return predict


class SimulationDemo:
    """Small, deterministic plant for exercising dashboard controls and display only."""

    def __init__(self, predictor: Callable[[list[float]], tuple[str, float]]):
        self.predictor = predictor
        self.clock_s = 0.0
        self.mission_s = 0.0
        self.state = "IDLE"
        self.outcome = "RUNNING"
        self.alert = "NORMAL"
        self.armed = False
        self.battery = 1.0
        self.fault_mode = "normal"
        self.land_started_s: float | None = None
        self.land_outcome = "COMPLETE"

    @property
    def active(self) -> bool:
        return self.armed and self.state not in {"LANDING", "COMPLETE", "ABORTED_LANDED", "FAILED"}

    def command(self, name: str) -> tuple[bool, str]:
        """Accept a fixed command vocabulary; no command leaves this process."""
        if name == "start":
            if self.armed:
                return False, "Mission is already active."
            self.mission_s = 0.0
            self.state, self.outcome, self.alert = "TAKEOFF", "RUNNING", "NORMAL"
            self.armed, self.battery = True, 1.0
            self.fault_mode = "normal"
            self.land_started_s = None
            return True, "Simulation mission started."
        if name == "land":
            if not self.armed:
                return False, "The simulated vehicle is already landed."
            self._begin_landing("COMPLETE")
            return True, "Simulated landing requested."
        if name == "abort":
            if not self.armed:
                return False, "There is no active simulated mission to abort."
            self.alert = "OPERATOR_ABORT"
            self._begin_landing("ABORTED_LANDED")
            return True, "Simulation abort accepted; simulated landing started."
        if name == "inject_fault":
            if not self.armed:
                return False, "Start the simulated mission before injecting a test fault."
            self.fault_mode = "vibration"
            return True, "Synthetic vibration fault injected for model demonstration."
        if name == "reset":
            if self.armed:
                return False, "Land or abort before resetting the simulation."
            self.mission_s = 0.0
            self.state, self.outcome, self.alert = "IDLE", "RUNNING", "NORMAL"
            self.fault_mode, self.battery = "normal", 1.0
            self.land_started_s = None
            return True, "Simulation reset."
        return False, "Unknown command."

    def _begin_landing(self, outcome: str) -> None:
        self.state = "LANDING"
        self.land_started_s = self.mission_s
        self.land_outcome = outcome

    def _position(self) -> tuple[float, float, float, int]:
        if self.state == "IDLE":
            return 0.0, 0.0, 0.0, 0
        if self.state in {"LANDING", "COMPLETE", "ABORTED_LANDED"}:
            if self.state != "LANDING" or self.land_started_s is None:
                return 0.0, 0.0, 0.0, len(WAYPOINTS)
            elapsed = self.mission_s - self.land_started_s
            altitude = max(0.0, 5.0 - elapsed * 2.5)
            if elapsed >= 2.0:
                self.armed = False
                self.state = self.land_outcome
                self.outcome = self.land_outcome
                altitude = 0.0
            return self.east, self.north, altitude, self.waypoint
        if self.mission_s < 2.0:
            self.state = "TAKEOFF"
            return 0.0, 0.0, min(5.0, self.mission_s * 2.5), 0
        self.state = "NAVIGATION"
        route_s = self.mission_s - 2.0
        leg_duration = 4.0
        leg = min(int(route_s // leg_duration), len(WAYPOINTS) - 1)
        fraction = min(1.0, (route_s - leg * leg_duration) / leg_duration)
        a = WAYPOINTS[leg]
        b = WAYPOINTS[min(leg + 1, len(WAYPOINTS) - 1)]
        self.east = a[0] + (b[0] - a[0]) * fraction
        self.north = a[1] + (b[1] - a[1]) * fraction
        self.waypoint = min(leg, len(WAYPOINTS))
        if route_s >= (len(WAYPOINTS) - 1) * leg_duration:
            self._begin_landing("COMPLETE")
            return self.east, self.north, 5.0, len(WAYPOINTS)
        return self.east, self.north, 5.0, self.waypoint

    def tick(self, dt: float = STEP_S) -> dict:
        dt = min(max(float(dt), 0.0), 0.5)
        self.clock_s += dt
        if self.armed:
            self.mission_s += dt
            self.battery = max(0.0, self.battery - dt * 0.0005)
        self.east = getattr(self, "east", 0.0)
        self.north = getattr(self, "north", 0.0)
        self.waypoint = getattr(self, "waypoint", 0)
        east, north, altitude, waypoint = self._position()

        # Features follow train.py's exact ordering and units. Faults are injected only
        # by the local demo control and are explicitly synthetic.
        features = [5000.0, 16.0, 8.0, 9.81, 0.03, 0.1, 0.2, 30.0]
        if self.fault_mode == "vibration":
            features[0] -= 500.0
            features[5] += 0.4
        elif self.fault_mode == "low_battery":
            features[1] -= 2.0
            features[2] += 4.0
        elif self.fault_mode == "position_error":
            features[4] += 0.1
            features[6] += 2.0
        ml_status, ml_probability = self.predictor(features)
        return {
            "source": "SIMULATION_DEMO",
            "transport": "LOCAL_WEBSOCKET",
            "time_s": round(self.clock_s, 3),
            "east": round(east, 3),
            "north": round(north, 3),
            "altitude": round(altitude, 3),
            "battery": round(self.battery, 4),
            "mode": "SIMULATION",
            "alert": self.alert,
            "age_s": 0.0,
            "armed": self.armed,
            "position_valid": True,
            "state": self.state,
            "outcome": self.outcome,
            "waypoint": waypoint,
            "waypoints": len(WAYPOINTS),
            "ml_status": ml_status,
            "ml_probability": round(ml_probability, 4),
            "ml_model": "synthetic-random-forest-v1",
            "ml_scope": "SYNTHETIC DEMO ONLY",
        }


def handle_message(sim: SimulationDemo, raw: str) -> dict:
    try:
        request = json.loads(raw)
    except (TypeError, json.JSONDecodeError):
        return {"type": "ack", "accepted": False, "message": "Invalid JSON."}
    if not isinstance(request, dict) or request.get("type") != "command":
        return {"type": "ack", "accepted": False, "message": "Expected a command request."}
    request_id = request.get("request_id")
    command_name = request.get("command")
    if not isinstance(request_id, str) or len(request_id) > 80:
        return {"type": "ack", "accepted": False, "message": "Invalid request ID."}
    if not isinstance(command_name, str):
        command_name = ""
    accepted, message = sim.command(command_name)
    return {"type": "ack", "request_id": request_id, "command": command_name,
            "accepted": accepted, "message": message}


async def client(websocket, predictor: Callable[[list[float]], tuple[str, float]]) -> None:
    sim = SimulationDemo(predictor)

    async def publish() -> None:
        while True:
            await websocket.send(json.dumps(sim.tick(), allow_nan=False))
            await asyncio.sleep(STEP_S)

    sender = asyncio.create_task(publish())
    try:
        async for raw in websocket:
            reply = handle_message(sim, raw)
            await websocket.send(json.dumps(reply, allow_nan=False))
    finally:
        sender.cancel()
        try:
            await sender
        except asyncio.CancelledError:
            pass


async def main(host: str, port: int) -> None:
    predictor = load_predictor()
    async with serve(lambda ws: client(ws, predictor), host, port,
                     origins=ALLOWED_ORIGINS, max_size=1024, max_queue=4,
                     write_limit=4096, ping_interval=20, ping_timeout=20):
        print(f"Simulation-only dashboard bridge: ws://{host}:{port}", flush=True)
        print("No ROS, PX4, or physical vehicle connection is available in this server.", flush=True)
        await asyncio.Future()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", choices=["127.0.0.1", "localhost"])
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("port must be between 1024 and 65535")
    try:
        asyncio.run(main(args.host, args.port))
    except RuntimeError as exc:
        parser.exit(2, f"{exc}\n")
