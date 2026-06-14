import argparse
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
MASTER_LOG_DIR = ROOT / "master_logs"
SERVO_LOG_DIR = ROOT / "servo_logs"
VERIFY_LOG_PATH = ROOT / "verify_transform_output.jsonl"
JOINT_LABELS = [f"joint{i}" for i in range(1, 7)]


def load_records(jsonl_path: Path) -> list[dict]:
    with jsonl_path.open("r", encoding="utf-8") as file:
        return [json.loads(line) for line in file if line.strip()]


def latest_master_log() -> Path:
    candidates = sorted(MASTER_LOG_DIR.glob("master_*.jsonl"))
    if not candidates:
        raise FileNotFoundError(f"No master log found in {MASTER_LOG_DIR}")
    return candidates[-1]


def latest_servo_log() -> Path:
    candidates = sorted(SERVO_LOG_DIR.glob("servo_*.jsonl"))
    if not candidates:
        raise FileNotFoundError(f"No servo log found in {SERVO_LOG_DIR}")
    return candidates[-1]


def latest_verify_log() -> Path:
    if not VERIFY_LOG_PATH.exists():
        raise FileNotFoundError(f"No verify transform output found at {VERIFY_LOG_PATH}")
    return VERIFY_LOG_PATH


def pairwise_abs_deltas(series: list[list[float]], joint_index: int) -> list[float]:
    return [abs(cur[joint_index] - prev[joint_index]) for prev, cur in zip(series, series[1:])]


def pairwise_signed_deltas(series: list[list[float]], joint_index: int) -> list[float]:
    return [cur[joint_index] - prev[joint_index] for prev, cur in zip(series, series[1:])]


def summarize(values: list[float]) -> dict[str, float]:
    if not values:
        return {"mean": 0.0, "max": 0.0}

    return {
        "mean": sum(values) / len(values),
        "max": max(values),
    }


def print_summary(title: str, summary: dict[str, float], unit: str) -> None:
    print(
        f"{title:<16} mean {summary['mean']:.4f}{unit}  "
        f"max {summary['max']:.4f}{unit}"
    )


def compute_velocities(series: list[list[float]], timestamps: list[float], joint_index: int) -> list[float]:
    velocities = []
    for previous, current, dt in zip(series, series[1:], [cur - prev for prev, cur in zip(timestamps, timestamps[1:])]):
        if dt > 1e-9:
            velocities.append(abs(current[joint_index] - previous[joint_index]) / dt)
    return velocities


def compute_accelerations(series: list[list[float]], timestamps: list[float], joint_index: int) -> list[float]:
    velocities = []
    dt_values = [cur - prev for prev, cur in zip(timestamps, timestamps[1:])]
    for previous, current, dt in zip(series, series[1:], dt_values):
        if dt > 1e-9:
            velocities.append((current[joint_index] - previous[joint_index]) / dt)

    accelerations = []
    for previous, current, dt in zip(velocities, velocities[1:], dt_values[1:]):
        if dt > 1e-9:
            accelerations.append(abs(current - previous) / dt)
    return accelerations


def compute_step_change(series: list[list[float]], joint_index: int) -> list[float]:
    steps = pairwise_signed_deltas(series, joint_index)
    return [abs(current - previous) for previous, current in zip(steps, steps[1:])]


def estimate_delay(
    reference_series: list[list[float]],
    response_series: list[list[float]],
    timestamps: list[float],
    joint_index: int,
    max_lag_steps: int = 30,
) -> dict[str, float]:
    if len(reference_series) < 4 or len(response_series) < 4 or len(timestamps) < 2:
        return {"lag_steps": 0.0, "lag_ms": 0.0, "score": 0.0}

    ref = [values[joint_index] for values in reference_series]
    resp = [values[joint_index] for values in response_series]
    mean_dt = sum(cur - prev for prev, cur in zip(timestamps, timestamps[1:])) / (len(timestamps) - 1)
    max_lag = min(max_lag_steps, len(ref) - 2, len(resp) - 2)

    best_lag = 0
    best_score = float("-inf")

    for lag in range(max_lag + 1):
        aligned_ref = ref[: len(ref) - lag]
        aligned_resp = resp[lag:]
        sample_count = min(len(aligned_ref), len(aligned_resp))
        if sample_count < 3:
            continue

        aligned_ref = aligned_ref[:sample_count]
        aligned_resp = aligned_resp[:sample_count]

        ref_mean = sum(aligned_ref) / sample_count
        resp_mean = sum(aligned_resp) / sample_count
        ref_centered = [value - ref_mean for value in aligned_ref]
        resp_centered = [value - resp_mean for value in aligned_resp]
        numerator = sum(a * b for a, b in zip(ref_centered, resp_centered))
        denom_ref = sum(a * a for a in ref_centered)
        denom_resp = sum(b * b for b in resp_centered)
        if denom_ref <= 1e-12 or denom_resp <= 1e-12:
            continue

        score = numerator / ((denom_ref * denom_resp) ** 0.5)
        if score > best_score:
            best_score = score
            best_lag = lag

    return {
        "lag_steps": float(best_lag),
        "lag_ms": best_lag * mean_dt * 1000.0,
        "score": 0.0 if best_score == float("-inf") else best_score,
    }


def print_delay(title: str, delay: dict[str, float]) -> None:
    print(
        f"{title:<16} lag {delay['lag_steps']:.0f} steps  "
        f"{delay['lag_ms']:.2f}ms  corr {delay['score']:.3f}"
    )


def print_log_header(title: str, jsonl_path: Path, sample_count: int, duration: float, dt_values: list[float]) -> None:
    print("=" * 88)
    print(f"{title}: {jsonl_path}")
    print(f"samples: {sample_count}")
    print(f"duration: {duration:.3f}s")
    print(f"effective rate: {sample_count / duration:.2f} Hz")
    print_summary("sample_dt", summarize(dt_values), "s")
    print("=" * 88)


def analyze_master_data(jsonl_path: Path) -> None:
    records = load_records(jsonl_path)
    if len(records) < 2:
        raise ValueError("Need at least two samples to analyze master data")

    timestamps = [record["timestamp"] for record in records]
    dt_values = [cur - prev for prev, cur in zip(timestamps, timestamps[1:])]
    raw_series = [record["raw_jaka_joints_deg"] for record in records]
    smoothed_series = [record["smoothed_jaka_joints_deg"] for record in records]
    limited_series = [record.get("limited_jaka_joints_deg", record["smoothed_jaka_joints_deg"]) for record in records]
    action_series = [[record["action"][f"joint{i}.pos"] for i in range(1, 7)] for record in records]

    duration = timestamps[-1] - timestamps[0]

    print_log_header("Master log", jsonl_path, len(records), duration, dt_values)

    for joint_index, label in enumerate(JOINT_LABELS):
        raw_steps = pairwise_abs_deltas(raw_series, joint_index)
        limited_steps = pairwise_abs_deltas(limited_series, joint_index)

        raw_range = max(value[joint_index] for value in raw_series) - min(value[joint_index] for value in raw_series)
        limited_range = max(value[joint_index] for value in limited_series) - min(value[joint_index] for value in limited_series)

        print(f"\n[{label}]")
        print(f"range raw {raw_range:.3f}deg  limited {limited_range:.3f}deg")
        print_summary("raw_step", summarize(raw_steps), "deg")
        print_summary("raw_vel", summarize(compute_velocities(raw_series, timestamps, joint_index)), "deg/s")
        print_summary("raw_acc", summarize(compute_accelerations(raw_series, timestamps, joint_index)), "deg/s2")
        print_summary("raw_step_change", summarize(compute_step_change(raw_series, joint_index)), "deg")
        print_summary("limited_step", summarize(limited_steps), "deg")
        print_summary("limited_vel", summarize(compute_velocities(limited_series, timestamps, joint_index)), "deg/s")
        print_summary("limited_acc", summarize(compute_accelerations(limited_series, timestamps, joint_index)), "deg/s2")
        print_summary("limited_step_change", summarize(compute_step_change(limited_series, joint_index)), "deg")
        print_summary("sent_vel", summarize(compute_velocities(action_series, timestamps, joint_index)), "deg/s")
        print_summary("sent_acc", summarize(compute_accelerations(action_series, timestamps, joint_index)), "deg/s2")
        print_summary("sent_step_change", summarize(compute_step_change(action_series, joint_index)), "deg")
        print_delay("raw->limited", estimate_delay(raw_series, limited_series, timestamps, joint_index))
        print_delay("raw->sent", estimate_delay(raw_series, action_series, timestamps, joint_index))


def analyze_servo_data(jsonl_path: Path) -> None:
    records = load_records(jsonl_path)
    if len(records) < 2:
        raise ValueError("Need at least two samples to analyze servo data")

    timestamps = [record["timestamp"] for record in records]
    dt_values = [cur - prev for prev, cur in zip(timestamps, timestamps[1:])]
    position_series = [record["jointPosition"] for record in records]
    step_num_values = [record.get("stepNum", 0) for record in records]
    rel_flag_values = [record.get("relFlag", 0) for record in records]
    duration = timestamps[-1] - timestamps[0]

    print_log_header("Servo log", jsonl_path, len(records), duration, dt_values)
    print(f"ip: {records[0].get('ip', 'unknown')}")
    print_summary("stepNum", summarize(step_num_values), "")
    print_summary("relFlag", summarize(rel_flag_values), "")

    for joint_index, label in enumerate(JOINT_LABELS):
        steps = pairwise_abs_deltas(position_series, joint_index)
        joint_values = [value[joint_index] for value in position_series]
        joint_range = max(joint_values) - min(joint_values)

        print(f"\n[{label}]")
        print(f"range sent {joint_range:.3f}deg")
        print_summary("step", summarize(steps), "deg")
        print_summary("velocity", summarize(compute_velocities(position_series, timestamps, joint_index)), "deg/s")
        print_summary("acceleration", summarize(compute_accelerations(position_series, timestamps, joint_index)), "deg/s2")
        print_summary("step_change", summarize(compute_step_change(position_series, joint_index)), "deg")


def analyze_verify_data(jsonl_path: Path) -> None:
    records = load_records(jsonl_path)
    if len(records) < 2:
        raise ValueError("Need at least two samples to analyze verify transform data")

    timestamps = [record["timestamp"] for record in records]
    dt_values = [cur - prev for prev, cur in zip(timestamps, timestamps[1:])]
    raw_series = [record["raw_jaka_joints_deg"] for record in records]
    limited_series = [record["limited_jaka_joints_deg"] for record in records]
    slave_smoothed_series = [record["smoothed_slave_position_deg"] for record in records]
    sent_series = [record["sent_joint_position_deg"] for record in records]
    duration = timestamps[-1] - timestamps[0]

    print_log_header("Verify log", jsonl_path, len(records), duration, dt_values)

    for joint_index, label in enumerate(JOINT_LABELS):
        raw_range = max(value[joint_index] for value in raw_series) - min(value[joint_index] for value in raw_series)
        sent_range = max(value[joint_index] for value in sent_series) - min(value[joint_index] for value in sent_series)

        print(f"\n[{label}]")
        print(f"range raw {raw_range:.3f}deg  sent {sent_range:.3f}deg")
        print_summary("master_raw_vel", summarize(compute_velocities(raw_series, timestamps, joint_index)), "deg/s")
        print_summary("master_raw_acc", summarize(compute_accelerations(raw_series, timestamps, joint_index)), "deg/s2")
        print_summary("master_limited_vel", summarize(compute_velocities(limited_series, timestamps, joint_index)), "deg/s")
        print_summary("master_limited_acc", summarize(compute_accelerations(limited_series, timestamps, joint_index)), "deg/s2")
        print_summary("slave_smooth_vel", summarize(compute_velocities(slave_smoothed_series, timestamps, joint_index)), "deg/s")
        print_summary("slave_smooth_acc", summarize(compute_accelerations(slave_smoothed_series, timestamps, joint_index)), "deg/s2")
        print_summary("sent_vel", summarize(compute_velocities(sent_series, timestamps, joint_index)), "deg/s")
        print_summary("sent_acc", summarize(compute_accelerations(sent_series, timestamps, joint_index)), "deg/s2")
        print_summary("sent_step_change", summarize(compute_step_change(sent_series, joint_index)), "deg")
        print_delay("raw->sent", estimate_delay(raw_series, sent_series, timestamps, joint_index))
        print_delay("limited->sent", estimate_delay(limited_series, sent_series, timestamps, joint_index))


def analyze_real_mode(master_log_path: Path, servo_log_path: Path) -> None:
    print("\n### Real Mode: master + servo ###\n")
    analyze_master_data(master_log_path)
    print("\n")
    analyze_servo_data(servo_log_path)


def analyze_offline_mode(master_log_path: Path, verify_log_path: Path) -> None:
    print("\n### Offline Mode: master + verify ###\n")
    analyze_master_data(master_log_path)
    print("\n")
    analyze_verify_data(verify_log_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze the real or offline master-slave pipeline logs.")
    parser.add_argument(
        "--mode",
        choices=("real", "offline"),
        default="real",
        help="Select real mode (master + servo) or offline mode (master + verify).",
    )
    parser.add_argument(
        "--master-log",
        default=None,
        help="Optional path to a master_*.jsonl log. Defaults to the newest file under master_logs/.",
    )
    parser.add_argument(
        "--servo-log",
        default=None,
        help="Optional path to a servo_*.jsonl log for real mode. Defaults to the newest file under servo_logs/.",
    )
    parser.add_argument(
        "--verify-log",
        default=None,
        help="Optional path to verify_transform_output.jsonl for offline mode. Defaults to verify_transform_output.jsonl in the workspace root.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    master_log_path = Path(args.master_log) if args.master_log else latest_master_log()

    if args.mode == "real":
        servo_log_path = Path(args.servo_log) if args.servo_log else latest_servo_log()
        analyze_real_mode(master_log_path, servo_log_path)
    elif args.mode == "offline":
        verify_log_path = Path(args.verify_log) if args.verify_log else latest_verify_log()
        analyze_offline_mode(master_log_path, verify_log_path)
    else:
        raise ValueError(f"Unsupported mode: {args.mode}")


if __name__ == "__main__":
    main()