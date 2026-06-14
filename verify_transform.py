from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
MASTER_LOG_DIR = ROOT / "master_logs"
DEFAULT_HOMING_OFFSET_PATH = ROOT / "homing_offset_so101.json"
JOINT_ORDER = [
	"shoulder_pan",
	"shoulder_lift",
	"elbow_flex",
	"wrist_flex",
	"wrist_roll",
	"extra_joint",
]
JOINT_LABELS = [f"joint{i}" for i in range(1, 7)]


class JointEmaSmoother:
	def __init__(self, alpha: float = 0.7, joint_num: int = 6):
		self.alpha = float(alpha)
		self.joint_num = int(joint_num)
		self.smoothed: list[float] | None = None

	def reset(self) -> None:
		self.smoothed = None

	def update(self, values: list[float] | None) -> list[float] | None:
		if values is None or len(values) != self.joint_num:
			return self.smoothed.copy() if self.smoothed is not None else None

		if self.smoothed is None:
			self.smoothed = list(values)
		else:
			self.smoothed = [
				self.alpha * values[index] + (1 - self.alpha) * self.smoothed[index]
				for index in range(self.joint_num)
			]
		return self.smoothed.copy()


class JointTargetRateLimiter:
	def __init__(self, vmax_deg_s: float | tuple[float, ...] | list[float] = 45.0, joint_num: int = 6):
		self.joint_num = int(joint_num)
		if isinstance(vmax_deg_s, (list, tuple)):
			if len(vmax_deg_s) != self.joint_num:
				raise ValueError("vmax_deg_s length must match joint_num")
			self.vmax = [float(value) for value in vmax_deg_s]
		else:
			self.vmax = [float(vmax_deg_s)] * self.joint_num
		self.prev_output: list[float] | None = None

	def reset(self) -> None:
		self.prev_output = None

	@staticmethod
	def _clip(value: float, lower: float, upper: float) -> float:
		return max(lower, min(upper, value))

	def update(self, target: list[float] | None, dt: float) -> list[float] | None:
		if target is None or len(target) != self.joint_num or dt <= 0:
			return self.prev_output.copy() if self.prev_output is not None else None

		if self.prev_output is None:
			self.prev_output = list(target)
			return self.prev_output.copy()

		limited = []
		for index in range(self.joint_num):
			max_step = self.vmax[index] * dt
			delta = target[index] - self.prev_output[index]
			limited.append(self.prev_output[index] + self._clip(delta, -max_step, max_step))

		self.prev_output = limited
		return limited.copy()


class JointClosedLoopRateLimiter:
	def __init__(
		self,
		kp_far: float = 1.0,
		kp_mid: float = 0.6,
		kp_near: float = 0.25,
		e_far_deg: float = 1.0,
		e_mid_deg: float = 0.2,
		e_stop_deg: float = 0.08,
		e_resume_deg: float = 0.15,
		vmax_deg_s: float | tuple[float, ...] | list[float] = 160.0,
		amax_deg_s2: float | tuple[float, ...] | list[float] = 2600.0,
		joint_num: int = 6,
	):
		self.kp_far = float(kp_far)
		self.kp_mid = float(kp_mid)
		self.kp_near = float(kp_near)
		self.e_far = float(e_far_deg)
		self.e_mid = float(e_mid_deg)
		self.e_stop = float(e_stop_deg)
		self.e_resume = float(e_resume_deg)
		self.joint_num = int(joint_num)
		self.vmax = self._expand(vmax_deg_s)
		self.amax = self._expand(amax_deg_s2)
		self.prev_step = [0.0] * self.joint_num
		self.in_stop_band = False

	def _expand(self, value: float | tuple[float, ...] | list[float]) -> list[float]:
		if isinstance(value, (list, tuple)):
			if len(value) != self.joint_num:
				raise ValueError("Per-joint limiter parameter length must match joint_num")
			return [float(item) for item in value]
		return [float(value)] * self.joint_num

	def reset(self) -> None:
		self.prev_step = [0.0] * self.joint_num
		self.in_stop_band = False

	@staticmethod
	def _clip(x: float, lo: float, hi: float) -> float:
		return max(lo, min(hi, x))

	def update(self, target: list[float] | None, actual: list[float] | None, dt: float) -> list[float] | None:
		if target is None or actual is None:
			return None
		if len(target) != self.joint_num or len(actual) != self.joint_num or dt <= 0:
			return None

		next_cmd = [0.0] * self.joint_num
		errors = [target[index] - actual[index] for index in range(self.joint_num)]
		max_abs_err = max(abs(err) for err in errors)

		if self.in_stop_band:
			if max_abs_err < self.e_resume:
				self.prev_step = [0.0] * self.joint_num
				return actual.copy()
			self.in_stop_band = False

		if max_abs_err < self.e_stop:
			self.in_stop_band = True
			self.prev_step = [0.0] * self.joint_num
			return actual.copy()

		for index in range(self.joint_num):
			err = errors[index]
			abs_err = abs(err)
			if abs_err > self.e_far:
				kp = self.kp_far
			elif abs_err > self.e_mid:
				kp = self.kp_mid
			else:
				kp = self.kp_near

			raw_step = kp * err
			max_step = self.vmax[index] * dt
			step_v = self._clip(raw_step, -max_step, max_step)

			max_step_change = self.amax[index] * dt * dt
			step = self._clip(
				step_v,
				self.prev_step[index] - max_step_change,
				self.prev_step[index] + max_step_change,
			)

			if err > 0:
				step = min(step, err)
			else:
				step = max(step, err)

			next_cmd[index] = actual[index] + step
			self.prev_step[index] = step

		return next_cmd


class JointCommandStateLimiter:
	"""直接约束发送命令序列本身，避免反馈基线 + step叠加带来的额外折线。"""

	def __init__(
		self,
		kp_far: float = 1.0,
		kp_mid: float = 0.6,
		kp_near: float = 0.25,
		e_far_deg: float = 1.0,
		e_mid_deg: float = 0.2,
		e_stop_deg: float = 0.08,
		e_resume_deg: float = 0.15,
		vmax_deg_s: float | tuple[float, ...] | list[float] = 160.0,
		amax_deg_s2: float | tuple[float, ...] | list[float] = 2600.0,
		deadband_deg: float | tuple[float, ...] | list[float] = 0.0,
		feedback_gain: float | tuple[float, ...] | list[float] = 0.0,
		feedback_max_step_deg: float | tuple[float, ...] | list[float] = 0.0,
		joint_num: int = 6,
	):
		self.kp_far = float(kp_far)
		self.kp_mid = float(kp_mid)
		self.kp_near = float(kp_near)
		self.e_far = float(e_far_deg)
		self.e_mid = float(e_mid_deg)
		self.e_stop = float(e_stop_deg)
		self.e_resume = float(e_resume_deg)
		self.joint_num = int(joint_num)
		self.vmax = self._expand(vmax_deg_s)
		self.amax = self._expand(amax_deg_s2)
		self.deadband = self._expand(deadband_deg)
		self.feedback_gain = self._expand(feedback_gain)
		self.feedback_max_step = self._expand(feedback_max_step_deg)
		self.prev_cmd: list[float] | None = None
		self.prev_step = [0.0] * self.joint_num
		self.in_stop_band = False

	def _expand(self, value: float | tuple[float, ...] | list[float]) -> list[float]:
		if isinstance(value, (list, tuple)):
			if len(value) != self.joint_num:
				raise ValueError("Per-joint limiter parameter length must match joint_num")
			return [float(item) for item in value]
		return [float(value)] * self.joint_num

	def reset(self) -> None:
		self.prev_cmd = None
		self.prev_step = [0.0] * self.joint_num
		self.in_stop_band = False

	@staticmethod
	def _clip(x: float, lo: float, hi: float) -> float:
		return max(lo, min(hi, x))

	def update(self, target: list[float] | None, dt: float, actual: list[float] | None = None) -> list[float] | None:
		if target is None or len(target) != self.joint_num or dt <= 0:
			return self.prev_cmd.copy() if self.prev_cmd is not None else None

		if self.prev_cmd is None:
			self.prev_cmd = list(target)
			self.prev_step = [0.0] * self.joint_num
			return self.prev_cmd.copy()

		base_cmd = self.prev_cmd.copy()
		if actual is not None and len(actual) == self.joint_num:
			for index in range(self.joint_num):
				feedback_error = actual[index] - base_cmd[index]
				feedback_step = self._clip(
					self.feedback_gain[index] * feedback_error,
					-self.feedback_max_step[index],
					self.feedback_max_step[index],
				)
				base_cmd[index] += feedback_step

		next_cmd = [0.0] * self.joint_num
		errors = [target[index] - base_cmd[index] for index in range(self.joint_num)]
		max_abs_err = max(abs(err) for err in errors)

		if self.in_stop_band:
			if max_abs_err < self.e_resume:
				self.prev_step = [0.0] * self.joint_num
				return self.prev_cmd.copy()
			self.in_stop_band = False

		if max_abs_err < self.e_stop:
			self.in_stop_band = True
			self.prev_step = [0.0] * self.joint_num
			return self.prev_cmd.copy()

		for index in range(self.joint_num):
			err = errors[index]
			if abs(err) < self.deadband[index]:
				next_cmd[index] = base_cmd[index]
				self.prev_step[index] = 0.0
				continue
			abs_err = abs(err)
			if abs_err > self.e_far:
				kp = self.kp_far
			elif abs_err > self.e_mid:
				kp = self.kp_mid
			else:
				kp = self.kp_near

			raw_step = kp * err
			max_step = self.vmax[index] * dt
			step_v = self._clip(raw_step, -max_step, max_step)

			max_step_change = self.amax[index] * dt * dt
			step = self._clip(
				step_v,
				self.prev_step[index] - max_step_change,
				self.prev_step[index] + max_step_change,
			)

			if err > 0:
				step = min(step, err)
			else:
				step = max(step, err)

			next_cmd[index] = base_cmd[index] + step
			self.prev_step[index] = step

		self.prev_cmd = next_cmd.copy()
		return next_cmd


@dataclass
class VerifyConfig:
	homing_offset_path: Path = DEFAULT_HOMING_OFFSET_PATH
	joint_direction: dict[str, int] | None = None
	master_read_period: float = 0.02
	control_freq: float = 30.0
	master_smooth_alpha: float = 0.55
	master_target_vmax_deg_s: tuple[float, float, float, float, float, float] = (75.0, 75.0, 75.0, 75.0, 75.0, 75.0)
	slave_smooth_alpha: float = 0.65
	limiter_kp_far: float = 0.75
	limiter_kp_mid: float = 0.5
	limiter_kp_near: float = 0.2
	limiter_e_far_deg: float = 1.5
	limiter_e_mid_deg: float = 0.35
	limiter_e_stop_deg: float = 0.12
	limiter_e_resume_deg: float = 0.20
	limiter_vmax_deg_s: float | tuple[float, ...] = (60.0, 52.0, 60.0, 70.0, 70.0, 70.0)
	limiter_amax_deg_s2: float | tuple[float, ...] = (210.0, 185.0, 210.0, 240.0, 240.0, 240.0)
	limiter_deadband_deg: float | tuple[float, ...] = (0.03, 0.03, 0.03, 0.03, 0.03, 0.03)
	feedback_gain: float | tuple[float, ...] = (0.03, 0.03, 0.03, 0.03, 0.03, 0.03)
	feedback_max_step_deg: float | tuple[float, ...] = (0.015, 0.015, 0.015, 0.015, 0.015, 0.015)

	def __post_init__(self) -> None:
		if self.joint_direction is None:
			self.joint_direction = {
				"shoulder_pan": -1,
				"shoulder_lift": -1,
				"elbow_flex": -1,
				"wrist_flex": -1,
				"wrist_roll": 1,
				"extra_joint": -1,
			}


class OfflineSlaveTransform:
	def __init__(self, config: VerifyConfig):
		self.config = config
		self.homing_offset = self._load_homing_offset(config.homing_offset_path)
		self.master_smoother = JointEmaSmoother(alpha=config.master_smooth_alpha, joint_num=6)
		self.master_target_limiter = JointTargetRateLimiter(
			vmax_deg_s=config.master_target_vmax_deg_s,
			joint_num=6,
		)
		self.slave_smoother = JointEmaSmoother(alpha=config.slave_smooth_alpha, joint_num=6)
		self.slave_limiter = JointCommandStateLimiter(
			kp_far=config.limiter_kp_far,
			kp_mid=config.limiter_kp_mid,
			kp_near=config.limiter_kp_near,
			e_far_deg=config.limiter_e_far_deg,
			e_mid_deg=config.limiter_e_mid_deg,
			e_stop_deg=config.limiter_e_stop_deg,
			e_resume_deg=config.limiter_e_resume_deg,
			vmax_deg_s=config.limiter_vmax_deg_s,
			amax_deg_s2=config.limiter_amax_deg_s2,
			deadband_deg=config.limiter_deadband_deg,
			feedback_gain=config.feedback_gain,
			feedback_max_step_deg=config.feedback_max_step_deg,
			joint_num=6,
		)
		self.simulated_slave_position: list[float] | None = None
		self.latest_master_stage: dict[str, Any] | None = None

	@staticmethod
	def _load_homing_offset(path: Path) -> dict[str, int]:
		with path.open("r", encoding="utf-8") as file:
			return json.load(file)

	def reset(self) -> None:
		self.master_smoother.reset()
		self.master_target_limiter.reset()
		self.slave_smoother.reset()
		self.slave_limiter.reset()
		self.simulated_slave_position = None
		self.latest_master_stage = None

	def raw_to_angles(self, raw: dict[str, int]) -> dict[str, float]:
		angles: dict[str, float] = {}
		for motor_name, homing_offset in self.homing_offset.items():
			diff = raw[motor_name] - homing_offset
			if diff > 2048:
				diff -= 4096
			elif diff < -2048:
				diff += 4096
			angle = diff * 360 / 4096
			angles[motor_name] = angle * self.config.joint_direction[motor_name]
		return angles

	def angles_to_master_target(self, angles: dict[str, float], dt: float) -> dict[str, Any]:
		raw_joints = [angles[name] for name in JOINT_ORDER]
		smoothed_joints = self.master_smoother.update(raw_joints)
		limiter_dt = dt if dt > 1e-9 else self.config.master_read_period
		limited_joints = self.master_target_limiter.update(smoothed_joints, limiter_dt)
		action = {f"joint{index}.pos": limited_joints[index - 1] for index in range(1, 7)}
		action["gripper.pos"] = 0.0
		return {
			"raw_joints": raw_joints,
			"smoothed_joints": smoothed_joints,
			"limited_joints": limited_joints,
			"action": action,
		}

	def update_master_from_raw(self, raw_record: dict[str, int], dt: float) -> dict[str, Any]:
		angles = self.raw_to_angles(raw_record)
		master_stage = self.angles_to_master_target(angles, dt)
		self.latest_master_stage = {
			"raw_positions": raw_record,
			"angles_deg": angles,
			**master_stage,
		}
		return self.latest_master_stage

	def master_action_to_slave_command(self, master_action: dict[str, Any]) -> dict[str, Any]:
		target = [master_action[f"joint{index}.pos"] for index in range(1, 7)]
		if self.simulated_slave_position is None:
			self.simulated_slave_position = target.copy()

		command_base_position = self.slave_limiter.prev_cmd.copy() if self.slave_limiter.prev_cmd is not None else self.simulated_slave_position.copy()
		smoothed_slave_position = self.slave_smoother.update(self.simulated_slave_position)
		sent_joint_position = self.slave_limiter.update(
			target,
			1.0 / self.config.control_freq,
			actual=smoothed_slave_position,
		)
		if sent_joint_position is None:
			sent_joint_position = self.simulated_slave_position.copy()

		# 离线模式下默认假设从臂每拍能跟到上一拍发送结果，便于直接观察限速器效果。
		self.simulated_slave_position = sent_joint_position.copy()
		return {
			"target": target,
			"command_base_position": command_base_position,
			"smoothed_slave_position": smoothed_slave_position,
			"sent_joint_position": sent_joint_position,
			"command_step": [sent_joint_position[index] - command_base_position[index] for index in range(6)],
			"step_num": max(1, round((1.0 / self.config.control_freq) / 0.008)),
		}

	def transform_raw_record(self, raw_record: dict[str, Any]) -> dict[str, Any]:
		master_stage = self.update_master_from_raw(raw_record, self.config.master_read_period)
		slave_stage = self.master_action_to_slave_command(master_stage["action"])
		return {
			"raw_positions": master_stage["raw_positions"],
			"angles_deg": master_stage["angles_deg"],
			"raw_jaka_joints_deg": master_stage["raw_joints"],
			"smoothed_jaka_joints_deg": master_stage["smoothed_joints"],
			"limited_jaka_joints_deg": master_stage["limited_joints"],
			"master_action": master_stage["action"],
			"command_base_position_deg": slave_stage["command_base_position"],
			"smoothed_slave_position_deg": slave_stage["smoothed_slave_position"],
			"sent_joint_position_deg": slave_stage["sent_joint_position"],
			"command_step_deg": slave_stage["command_step"],
			"servo_step_num": slave_stage["step_num"],
		}


def load_jsonl(path: Path) -> list[dict[str, Any]]:
	with path.open("r", encoding="utf-8") as file:
		return [json.loads(line) for line in file if line.strip()]


def latest_master_log() -> Path:
	candidates = sorted(MASTER_LOG_DIR.glob("master_*.jsonl"))
	if not candidates:
		raise FileNotFoundError(f"No master log found in {MASTER_LOG_DIR}")
	return candidates[-1]


def simulate_master_log(
	master_log_path: Path,
	output_path: Path | None = None,
	initial_slave_position: list[float] | None = None,
) -> list[dict[str, Any]]:
	records = load_jsonl(master_log_path)
	simulator = OfflineSlaveTransform(VerifyConfig())
	simulator.reset()
	if initial_slave_position is not None:
		simulator.simulated_slave_position = list(initial_slave_position)

	if not records:
		return []

	control_period = 1.0 / simulator.config.control_freq
	transformed_records = []
	first_timestamp = records[0].get("timestamp", 0.0)
	initial_stage = simulator.update_master_from_raw(records[0]["raw_positions"], simulator.config.master_read_period)
	current_master_stage = initial_stage
	current_source_timestamp = first_timestamp
	source_index = 1
	tick_time = first_timestamp
	end_time = records[-1].get("timestamp", tick_time)

	while tick_time <= end_time:
		while (
			source_index < len(records)
			and records[source_index].get("timestamp", tick_time) <= tick_time
		):
			record = records[source_index]
			record_timestamp = record.get("timestamp", current_source_timestamp)
			record_dt = record_timestamp - current_source_timestamp
			current_master_stage = simulator.update_master_from_raw(record["raw_positions"], record_dt)
			current_source_timestamp = record_timestamp
			source_index += 1

		slave_stage = simulator.master_action_to_slave_command(current_master_stage["action"])
		transformed_records.append(
			{
				"timestamp": tick_time,
				"source_timestamp": current_source_timestamp,
				"raw_positions": current_master_stage["raw_positions"],
				"angles_deg": current_master_stage["angles_deg"],
				"raw_jaka_joints_deg": current_master_stage["raw_joints"],
				"smoothed_jaka_joints_deg": current_master_stage["smoothed_joints"],
				"limited_jaka_joints_deg": current_master_stage["limited_joints"],
				"master_action": current_master_stage["action"],
				"command_base_position_deg": slave_stage["command_base_position"],
				"smoothed_slave_position_deg": slave_stage["smoothed_slave_position"],
				"sent_joint_position_deg": slave_stage["sent_joint_position"],
				"command_step_deg": slave_stage["command_step"],
				"servo_step_num": slave_stage["step_num"],
			}
		)
		tick_time += control_period

	if output_path is not None:
		with output_path.open("w", encoding="utf-8") as file:
			for record in transformed_records:
				file.write(json.dumps(record, ensure_ascii=False) + "\n")

	return transformed_records


def summarize_steps(records: list[dict[str, Any]]) -> None:
	if len(records) < 2:
		print("Not enough records to summarize")
		return

	print(f"samples: {len(records)}")
	for joint_index, label in enumerate(JOINT_LABELS):
		values = [record["sent_joint_position_deg"][joint_index] for record in records]
		deltas = [abs(current - previous) for previous, current in zip(values, values[1:])]
		print(
			f"{label:<8} range {max(values) - min(values):7.3f} deg  "
			f"step_mean {sum(deltas) / len(deltas):7.4f} deg  "
			f"step_max {max(deltas):7.4f} deg"
		)


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Offline reproduction of the current master-to-slave limiting pipeline.")
	parser.add_argument(
		"--master-log",
		default=None,
		help="Path to a master_*.jsonl log. Defaults to the newest file in master_logs/.",
	)
	parser.add_argument(
		"--output",
		default=None,
		help="Optional output JSONL path for the simulated slave send coordinates.",
	)
	parser.add_argument(
		"--initial-slave",
		default=None,
		help="Optional comma-separated initial slave joint positions in degrees.",
	)
	return parser.parse_args()


def parse_initial_slave(value: str | None) -> list[float] | None:
	if value is None:
		return None
	items = [item.strip() for item in value.split(",") if item.strip()]
	if len(items) != 6:
		raise ValueError("--initial-slave must provide 6 comma-separated joint values")
	return [float(item) for item in items]


def main() -> None:
	args = parse_args()
	master_log_path = Path(args.master_log) if args.master_log else latest_master_log()
	output_path = Path(args.output) if args.output else ROOT / "verify_transform_output.jsonl"
	records = simulate_master_log(
		master_log_path=master_log_path,
		output_path=output_path,
		initial_slave_position=parse_initial_slave(args.initial_slave),
	)
	print(f"master_log: {master_log_path}")
	print(f"output: {output_path}")
	summarize_steps(records)


if __name__ == "__main__":
	main()
