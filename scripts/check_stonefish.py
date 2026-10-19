import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from uuid import uuid4

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, default=Path("/artifacts"))
    args = parser.parse_args()
    scenario = args.scenario.resolve(strict=True)
    run_id = f"stonefish-{uuid4().hex[:12]}"
    output = args.output_root / run_id
    output.mkdir(parents=True, exist_ok=False)
    rows = []
    rclpy.init()
    node = rclpy.create_node("stonefish_probe_check")

    def receive(message):
        rows.append({"run_id": run_id, "source": "stonefish",
                     "frame_id": message.header.frame_id,
                     "stamp_sec": message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                     "gauge_pressure_pa": message.fluid_pressure})

    node.create_subscription(FluidPressure, "/probe/pressure", receive, qos_profile_sensor_data)
    try:
        with (output / "simulator.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen([
                "ros2", "run", "stonefish_ros2", "stonefish_simulator_nogpu",
                str(scenario.parent) + "/", str(scenario), "100.0",
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic() + 30
                while len(rows) < 10 and time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError(f"Stonefish exited early; inspect {output / 'simulator.log'}")
                    rclpy.spin_once(node, timeout_sec=0.2)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        (output / "pressure.jsonl").write_text(
            "".join(json.dumps(row, allow_nan=False) + "\n" for row in rows), encoding="utf-8")
    if len(rows) < 10:
        raise RuntimeError(f"Expected 10 pressure messages, received {len(rows)}; see {output}")
    if any(not math.isfinite(row["gauge_pressure_pa"]) or
           not 40000 < row["gauge_pressure_pa"] < 60000 for row in rows):
        raise RuntimeError("Pressure inconsistent with the configured 5 m depth")
    if any(right["stamp_sec"] <= left["stamp_sec"] for left, right in zip(rows, rows[1:])):
        raise RuntimeError("Sensor timestamps do not increase")
    summary = {"run_id": run_id, "sensor_messages": len(rows), "source": "stonefish",
               "topic": "/probe/pressure", "fixture": "fixed sphere at 5 m",
               "diagnostic_agent_connected": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"STONEFISH_SMOKE_PASS {output}")


if __name__ == "__main__":
    main()
