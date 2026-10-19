import json
from pathlib import Path
import tempfile
import unittest

from auv_diagnostics.core import (
    DataFormatError, decode, diagnose_stub, normalize, sample, validate_report,
)
from auv_diagnostics.demo import run


class DataFormatTests(unittest.TestCase):
    def test_conversion_and_identity(self):
        raw = sample("run-42", "auv-02", 3)
        raw.update(depth_cm=1234, speed_cm_s=25, thruster_command_pct=-50)
        normalized = normalize(raw)
        self.assertEqual(normalized["depth_m"], 12.34)
        self.assertEqual(normalized["speed_m_s"], 0.25)
        self.assertEqual(normalized["thruster_command"], -0.5)
        report = validate_report(diagnose_stub(normalized))
        for key in ("run_id", "vehicle_id", "sequence", "stamp_sec"):
            self.assertEqual(report[key], raw[key])
        self.assertEqual(report["status"], "stub")
        self.assertEqual(report["anomaly_type"], "unknown")

    def test_invalid_inputs(self):
        cases = [
            ("depth_cm", float("nan")), ("depth_cm", float("inf")),
            ("depth_cm", 10 ** 1000),
            ("depth_cm", -1), ("speed_cm_s", True), ("speed_cm_s", "50"),
            ("sequence", 1.5), ("sequence", True), ("sequence", -1),
            ("stamp_sec", -1), ("run_id", " "), ("vehicle_id", None),
            ("thruster_command_pct", 101), ("schema_version", "2.0"),
        ]
        for field, value in cases:
            with self.subTest(field=field, value=value):
                raw = sample("run", "auv", 0)
                raw[field] = value
                with self.assertRaises(DataFormatError):
                    normalize(raw)

    def test_missing_and_ground_truth_fields_rejected(self):
        raw = sample("run", "auv", 0)
        del raw["depth_cm"]
        with self.assertRaises(DataFormatError):
            normalize(raw)
        raw = sample("run", "auv", 0)
        raw["ground_truth"] = "thruster_fault"
        with self.assertRaises(DataFormatError):
            normalize(raw)

    def test_malformed_json_and_duplicate_keys(self):
        for payload in ('{"x": NaN}', '{"x": 1, "x": 2}', '[]', '{broken'):
            with self.subTest(payload=payload), self.assertRaises(DataFormatError):
                decode(payload)

    def test_stub_cannot_claim_real_diagnosis_or_sources(self):
        for field, value in (("status", "ok"), ("anomaly_type", "normal"),
                             ("rag_references", ["invented"]), ("recommendations", [])):
            report = diagnose_stub(normalize(sample("run", "auv", 0)))
            report[field] = value
            with self.subTest(field=field), self.assertRaises(DataFormatError):
                validate_report(report)

    def test_normalized_format_rejects_wrong_source_and_values(self):
        for field, value in (("source", "stonefish"), ("thruster_command", 40),
                             ("depth_m", None)):
            normalized = normalize(sample("run", "auv", 0))
            normalized[field] = value
            with self.subTest(field=field), self.assertRaises(DataFormatError):
                diagnose_stub(normalized)

    def test_local_pipeline_persists_every_message(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            summary = run(output, 5, "run-42", "auv-02")
            self.assertEqual(summary["reports"], 5)
            self.assertIsNone(summary["diagnostic_accuracy"])
            for filename in ("raw.jsonl", "normalized.jsonl", "reports.jsonl"):
                rows = [json.loads(line) for line in (output / filename).read_text().splitlines()]
                self.assertEqual([row["sequence"] for row in rows], list(range(5)))
                self.assertTrue(all(row["run_id"] == "run-42" for row in rows))
            before = (output / "reports.jsonl").read_bytes()
            with self.assertRaises(FileExistsError):
                run(output, 5, "run-42")
            self.assertEqual((output / "reports.jsonl").read_bytes(), before)

    def test_count_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            for count in (0, -1, 101, True):
                with self.subTest(count=count), self.assertRaises(ValueError):
                    run(Path(directory) / "unused", count, "run")


if __name__ == "__main__":
    unittest.main()
