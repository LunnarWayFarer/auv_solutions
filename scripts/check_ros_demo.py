import argparse
import json
from pathlib import Path
import subprocess
from uuid import uuid4

from auv_diagnostics.core import validate_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=Path("artifacts"))
    parser.add_argument("--count", type=int, default=5)
    args = parser.parse_args()
    if not 1 <= args.count <= 100:
        parser.error("count must be between 1 and 100")
    run_id = f"ros2-{uuid4().hex[:12]}"
    output = (args.output_root / run_id).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run([
        "ros2", "launch", "auv_diagnostics", "week1.launch.py",
        f"run_id:={run_id}", "vehicle_id:=auv-01", f"count:={args.count}",
        f"output_dir:={output}",
    ], timeout=45, check=True)
    reports = [validate_report(json.loads(line))
               for line in (output / "reports.jsonl").read_text(encoding="utf-8").splitlines()]
    if len(reports) != args.count or {r["sequence"] for r in reports} != set(range(args.count)):
        raise RuntimeError("Reports missing or duplicated")
    for report in reports:
        if report["run_id"] != run_id or report["vehicle_id"] != "auv-01":
            raise RuntimeError("Report identity mismatch")
        expected_depth = (1000.0 + report["sequence"] * 2) / 100
        if report["evidence"] != {"depth_m": expected_depth, "speed_m_s": 0.5,
                                  "thruster_command": 0.4}:
            raise RuntimeError("Telemetry changed unexpectedly in transit")
    summary = {"run_id": run_id, "reports": len(reports), "transport": "ROS2",
               "agent": "fixed-stub/0.1", "launch_returncode": result.returncode,
               "diagnostic_accuracy": None}
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"ROS2_SMOKE_PASS {output}")


if __name__ == "__main__":
    main()
