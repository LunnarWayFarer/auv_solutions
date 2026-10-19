import argparse
from pathlib import Path
from uuid import uuid4

from .core import diagnose_stub, encode, normalize, sample, validate_report


def run(output, count, run_id, vehicle_id="auv-01"):
    if type(count) is not int or not 1 <= count <= 100:
        raise ValueError("count must be between 1 and 100")

    sample(run_id, vehicle_id, 0)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    with (
        (output / "raw.jsonl").open("x", encoding="utf-8") as raw_file,
        (output / "normalized.jsonl").open("x", encoding="utf-8") as normalized_file,
        (output / "reports.jsonl").open("x", encoding="utf-8") as report_file,
    ):
        for sequence in range(count):
            raw = sample(run_id, vehicle_id, sequence)
            normalized = normalize(raw)
            report = validate_report(diagnose_stub(normalized))
            for file, message in ((raw_file, raw), (normalized_file, normalized), (report_file, report)):
                file.write(encode(message) + "\n")
    summary = {"run_id": run_id, "vehicle_id": vehicle_id, "reports": count,
               "transport": "in_process", "agent": "fixed-stub/0.1",
               "diagnostic_accuracy": None}
    (output / "summary.json").write_text(encode(summary) + "\n", encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--run-id", default=f"local-{uuid4().hex[:12]}")
    parser.add_argument("--vehicle-id", default="auv-01")
    parser.add_argument("--output", type=Path, help="New directory; existing runs are never overwritten")
    args = parser.parse_args()
    output = args.output or Path("artifacts") / f"local-{uuid4().hex[:12]}"
    try:
        summary = run(output, args.count, args.run_id, args.vehicle_id)
    except (OSError, ValueError) as error:
        parser.exit(1, f"Demo failed: {error}\n")
    print(encode({**summary, "output": str(output.resolve())}))


if __name__ == "__main__":
    main()
