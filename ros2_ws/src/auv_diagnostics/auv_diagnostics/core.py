import json
import math

VERSION = "0.1"
IDENTITY = {"schema_version", "run_id", "vehicle_id", "sequence", "stamp_sec"}
RAW_KEYS = IDENTITY | {"depth_cm", "speed_cm_s", "thruster_command_pct"}
NORMALIZED_KEYS = IDENTITY | {"source", "depth_m", "speed_m_s", "thruster_command"}
REPORT_KEYS = IDENTITY | {
    "status", "anomaly_type", "agent", "evidence", "recommendations", "rag_references"
}


class DataFormatError(ValueError):
    pass


def number(value, name, minimum=0.0, maximum=None):
    if type(value) not in (int, float):
        raise DataFormatError(f"{name}: expected finite number")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise DataFormatError(f"{name}: expected finite number")
    if value < minimum or (maximum is not None and value > maximum):
        raise DataFormatError(f"{name}: outside allowed range")
    return value


def identity(message, keys):
    if not isinstance(message, dict) or set(message) != keys:
        raise DataFormatError(f"Expected exactly these fields: {sorted(keys)}")
    if message["schema_version"] != VERSION:
        raise DataFormatError("Unsupported schema_version")
    for field in ("run_id", "vehicle_id"):
        if not isinstance(message[field], str) or not message[field].strip():
            raise DataFormatError(f"{field}: expected non-empty string")
    if type(message["sequence"]) is not int or message["sequence"] < 0:
        raise DataFormatError("sequence: expected non-negative integer")
    number(message["stamp_sec"], "stamp_sec")
    return {key: message[key] for key in sorted(IDENTITY)}


def validate_raw(message):
    identity(message, RAW_KEYS)
    number(message["depth_cm"], "depth_cm")
    number(message["speed_cm_s"], "speed_cm_s")
    number(message["thruster_command_pct"], "thruster_command_pct", -100, 100)
    return message


def normalize(message):
    validate_raw(message)
    return {
        **identity(message, RAW_KEYS),
        "source": "synthetic",
        "depth_m": message["depth_cm"] / 100,
        "speed_m_s": message["speed_cm_s"] / 100,
        "thruster_command": message["thruster_command_pct"] / 100,
    }


def validate_normalized(message):
    identity(message, NORMALIZED_KEYS)
    if message["source"] != "synthetic":
        raise DataFormatError("Week-one adapter only supports synthetic input")
    number(message["depth_m"], "depth_m")
    number(message["speed_m_s"], "speed_m_s")
    number(message["thruster_command"], "thruster_command", -1, 1)
    return message


def diagnose_stub(message):
    validate_normalized(message)
    return {
        **identity(message, NORMALIZED_KEYS),
        "status": "stub",
        "anomaly_type": "unknown",
        "agent": "fixed-stub/0.1",
        "evidence": {key: message[key] for key in ("depth_m", "speed_m_s", "thruster_command")},
        "recommendations": ["Тест интеграции: диагностическая модель ещё не подключена."],
        "rag_references": [],
    }


def validate_report(message):
    identity(message, REPORT_KEYS)
    if (message["status"], message["anomaly_type"], message["agent"]) != (
        "stub", "unknown", "fixed-stub/0.1"
    ):
        raise DataFormatError("Expected an explicitly marked stub report")
    evidence = message["evidence"]
    if not isinstance(evidence, dict) or set(evidence) != {
        "depth_m", "speed_m_s", "thruster_command"
    }:
        raise DataFormatError("Invalid evidence fields")
    number(evidence["depth_m"], "depth_m")
    number(evidence["speed_m_s"], "speed_m_s")
    number(evidence["thruster_command"], "thruster_command", -1, 1)
    recommendations = message["recommendations"]
    if not isinstance(recommendations, list) or not recommendations or any(
        not isinstance(item, str) or not item.strip() for item in recommendations
    ):
        raise DataFormatError("Expected non-empty recommendations")
    if message["rag_references"] != []:
        raise DataFormatError("Stub has no RAG sources")
    return message


def sample(run_id, vehicle_id, sequence):
    message = {
        "schema_version": VERSION,
        "run_id": run_id,
        "vehicle_id": vehicle_id,
        "sequence": sequence,
        "stamp_sec": float(sequence),
        "depth_cm": 1000.0 + sequence * 2,
        "speed_cm_s": 50.0,
        "thruster_command_pct": 40.0,
    }
    return validate_raw(message)


def encode(message):
    return json.dumps(message, ensure_ascii=False, allow_nan=False, sort_keys=True)


def decode(payload):
    def reject_constant(value):
        raise DataFormatError(f"Invalid JSON number: {value}")

    def unique_keys(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise DataFormatError(f"Duplicate JSON key: {key}")
            result[key] = value
        return result

    try:
        message = json.loads(payload, parse_constant=reject_constant, object_pairs_hook=unique_keys)
    except (ValueError, TypeError) as error:
        raise DataFormatError(str(error)) from error
    if not isinstance(message, dict):
        raise DataFormatError("JSON message must be an object")
    return message
