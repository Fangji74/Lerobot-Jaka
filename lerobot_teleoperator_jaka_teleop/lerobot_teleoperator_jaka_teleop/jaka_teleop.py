from typing import Any
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.robots.robot import Robot

import os
import sys
import threading
sys.path.insert(0, r'D:\Desktop\schoolwork_22\club\jaka\lerobot_jaka\lerobot_teleoperator_jaka_teleop\lerobot_teleoperator_jaka_teleop')
sys.path.insert(0, r'D:\Desktop\schoolwork_22\club\jaka\lerobot_jaka\lerobot_robot_jaka\lerobot_robot_jaka')

from lerobot_teleoperator_jaka_teleop.config_jaka_teleop import JakaTeleopConfig
from lerobot.motors.feetech import FeetechMotorsBus
import numpy as np
import logging
import time
import json

logger = logging.getLogger(__name__)

PI = 3.14159
deg2rad = PI / 180


def _copy_action_dict(action: dict[str, Any] | None) -> dict[str, Any] | None:
    if action is None:
        return None
    copied = {}
    for key, value in action.items():
        copied[key] = value.copy() if isinstance(value, list) else value
    return copied

class JointEmaSmoother:
    """关节目标坐标平滑器，主臂目标与从臂反馈的轻量去噪"""

    def __init__(self, alpha=0.7, joint_num=6):
        self.alpha = float(alpha)
        self.joint_num = int(joint_num)
        self.smoothed = None

    def reset(self):
        self.smoothed = None

    def update(self, values):
        if values is None:
            return self.smoothed
        if len(values) != self.joint_num and len(values) != self.joint_num + 1: # 允许夹爪状态一起输入
            return self.smoothed
        if self.smoothed is None:
            self.smoothed = values.copy()
        else:
            # 保持夹爪状态不变
            gripper_state = self.smoothed[-1] if len(self.smoothed) == self.joint_num + 1 else None
            # 只平滑关节坐标部分
            self.smoothed = self.smoothed[:self.joint_num]
            self.smoothed = [
                self.alpha * values[i] + (1 - self.alpha) * self.smoothed[i]
                for i in range(self.joint_num)
            ]
            # 恢复夹爪状态
            if gripper_state is not None:
                self.smoothed.append(gripper_state)
        return self.smoothed.copy()


class JointTargetRateLimiter:
    """将主臂台阶式目标整形成连续关节目标。"""

    def __init__(self, vmax_deg_s=45.0, joint_num=6):
        self.joint_num = int(joint_num)
        if isinstance(vmax_deg_s, (list, tuple)):
            if len(vmax_deg_s) != self.joint_num:
                raise ValueError("vmax_deg_s length must match joint_num")
            self.vmax = [float(value) for value in vmax_deg_s]
        else:
            self.vmax = [float(vmax_deg_s)] * self.joint_num
        self.prev_output = None

    def reset(self):
        self.prev_output = None

    @staticmethod
    def _clip(value, lower, upper):
        return max(lower, min(upper, value))

    def update(self, target, dt):
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


class JointVelocityHoldFilter:
    """在输入短暂停住时短时保持最近速度，减少编码器量化带来的断续感。"""

    def __init__(self, input_deadband_deg=0.02, hold_decay=0.65, max_lead_deg=0.25, joint_num=6):
        self.joint_num = int(joint_num)
        self.input_deadband = self._expand(input_deadband_deg)
        self.hold_decay = self._expand(hold_decay)
        self.max_lead = self._expand(max_lead_deg)
        self.prev_input = None
        self.velocity = [0.0] * self.joint_num
        self.output = None

    def _expand(self, value):
        if isinstance(value, (list, tuple)):
            if len(value) != self.joint_num:
                raise ValueError("Filter parameter length must match joint_num")
            return [float(item) for item in value]
        return [float(value)] * self.joint_num

    @staticmethod
    def _clip(value, lower, upper):
        return max(lower, min(upper, value))

    def reset(self):
        self.prev_input = None
        self.velocity = [0.0] * self.joint_num
        self.output = None

    def update(self, target, dt):
        if target is None or len(target) != self.joint_num or dt <= 0:
            return self.output.copy() if self.output is not None else None

        if self.prev_input is None:
            self.prev_input = list(target)
            self.output = list(target)
            self.velocity = [0.0] * self.joint_num
            return self.output.copy()

        output = [0.0] * self.joint_num
        for index in range(self.joint_num):
            delta = target[index] - self.prev_input[index]
            if abs(delta) > self.input_deadband[index]:
                self.velocity[index] = delta / dt
            else:
                self.velocity[index] *= self.hold_decay[index]

            lead = self._clip(self.velocity[index] * dt, -self.max_lead[index], self.max_lead[index])
            output[index] = target[index] + lead

        self.prev_input = list(target)
        self.output = output
        return output.copy()

class JakaTeleop(Teleoperator):
    """主从臂遥操作器 主臂采用类so101的同构机械臂"""
    config_class = JakaTeleopConfig
    name = "jaka_teleop"

    def __init__(self, config: JakaTeleopConfig):
        super().__init__(config)
        self.config = config
        self._connected = False
        self._calibrated = False
        
        # 主臂和从臂机器人实例
        self.master_robot: Robot = None # 主臂，需要外部注入
        
        # 状态变量
        self._master_joint_positions = None # 主臂关节位置
        self._master_pose = None # 主臂末端位姿
        self._gripper_state = 0.0 # 夹爪状态
        self._latest_action = None
        self._prev_action_sample = None
        self._latest_action_sample = None
        self._latest_joint_velocity = [0.0] * 6
        self._master_read_thread = None
        self._master_state_lock = threading.Lock()
        self._master_log_lock = threading.Lock()
        self._master_log_file = None
        self._master_log_path = None
        
        # 控制参数
        self._control_mode = "joint" # 暂时固定为joint
        self._follow_mode = True # True: 跟随, False: 停止跟随

        self._stop_thread = False
        
        # 性能参数
        self._control_freq = config.control_freq  # 控制频率
        self._dt = 1.0 / self._control_freq
        self._master_read_period = float(config.master_read_period)
        self._enable_master_logging = bool(config.enable_master_logging)
        self._max_extrapolation_window = max(self._master_read_period * 2.0, self._dt)

        # 舵机配置和校准数据
        homing_offset_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "homing_offset.json")
        )
        
        with open(homing_offset_path, "r", encoding="utf-8") as f:
            self.HOMING_OFFSET = json.load(f)
        self.JOINT_DIRECTION = {
            "shoulder_pan": -1,
            "shoulder_lift": -1,
            "elbow_flex": -1,
            "wrist_flex": -1,
            "wrist_roll": 1,
            "extra_joint": -1,
            "gripper": 1,
        }

        # #so101实验代码
        # # 舵机配置和校准数据
        # homing_offset_path = os.path.abspath(
        #     os.path.join(os.path.dirname(__file__), "..", "..", "homing_offset_so101.json")
        # )
        
        # with open(homing_offset_path, "r", encoding="utf-8") as f:
        #     self.HOMING_OFFSET = json.load(f)
        # self.JOINT_DIRECTION = {
        #     "shoulder_pan": -1,
        #     "shoulder_lift": -1,
        #     "elbow_flex": -1,
        #     "wrist_flex": -1,
        #     "wrist_roll": 1,
        #     "extra_joint": -1
        # }

        # 控制相关
        self.master_smooth_alpha = 0.35  # 轻量去噪，避免把高频主臂状态再次抹平
        self.master_smoother = JointEmaSmoother(alpha=self.master_smooth_alpha, joint_num=6)

    def set_robots(self, master_robot: Robot):
        """设置主臂机器人实例"""
        self.master_robot = master_robot

    def connect(self, calibrate: bool = True) -> None:
        """连接主臂"""
        # 如果未外部注入，则根据配置创建机器人实例
        if self.master_robot is None:
            self.master_robot = FeetechMotorsBus(
                port=self.config.PORT,
                motors=self.config.MOTORS_CONFIG,
                calibration=self.config.CALIBRATION,
            )

        if not self.master_robot:
            raise ValueError("Master robot must be set before connecting")
        
        # 连接主臂
        if not self.master_robot.is_connected:
            self.master_robot.connect()
        time.sleep(1)

        if self._enable_master_logging:
            self._open_master_log()

        self._stop_thread = False
        self.master_smoother.reset()
        with self._master_state_lock:
            self._prev_action_sample = None
            self._latest_action_sample = None
            self._latest_action = None
            self._latest_joint_velocity = [0.0] * 6
        self._update_master_state()
        self._master_read_thread = threading.Thread(target=self._master_read_loop, daemon=True)
        self._master_read_thread.start()
        
        self._connected = True
        
        logger.info("Master-slave teleop connected")

    def _open_master_log(self) -> None:
        log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "master_logs"))
        os.makedirs(log_dir, exist_ok=True)
        log_name = f"master_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
        self._master_log_path = os.path.join(log_dir, log_name)
        self._master_log_file = open(self._master_log_path, "a", encoding="utf-8", buffering=1)

    def _close_master_log(self) -> None:
        with self._master_log_lock:
            if self._master_log_file is not None:
                self._master_log_file.flush()
                self._master_log_file.close()
                self._master_log_file = None

    def _log_master_sample(self, raw_positions, angles, raw_joints, smoothed_joints, estimated_velocity, action) -> None:
        if self._master_log_file is None:
            return

        record = {
            "timestamp": time.time(),
            "raw_positions": raw_positions,
            "angles_deg": angles,
            "raw_jaka_joints_deg": raw_joints,
            "smoothed_jaka_joints_deg": smoothed_joints,
            "estimated_velocity_deg_s": estimated_velocity,
            "action": action,
        }
        with self._master_log_lock:
            self._master_log_file.write(json.dumps(record, ensure_ascii=False) + "\n")

    @staticmethod
    def _estimate_velocity(prev_sample: dict[str, Any] | None, latest_sample: dict[str, Any] | None) -> list[float]:
        if prev_sample is None or latest_sample is None:
            return [0.0] * 6

        t0 = float(prev_sample["timestamp"])
        t1 = float(latest_sample["timestamp"])
        dt = t1 - t0
        if dt <= 1e-6:
            return [0.0] * 6

        prev_action = prev_sample["action"]
        latest_action = latest_sample["action"]
        velocity = [0.0] * 6
        for index in range(6):
            key = f"joint{index + 1}.pos"
            velocity[index] = (latest_action[key] - prev_action[key]) / dt
        return velocity

    def _build_control_action(self, output_time: float) -> dict[str, Any] | None:
        latest_sample = self._latest_action_sample
        if latest_sample is None:
            return None

        action = _copy_action_dict(latest_sample["action"]) or {}
        lead_time = max(0.0, min(output_time - float(latest_sample["timestamp"]), self._max_extrapolation_window))
        for index in range(6):
            key = f"joint{index + 1}.pos"
            action[key] = float(action[key] + self._latest_joint_velocity[index] * lead_time)
        return action

    def _update_master_state(self) -> None:
        sample_timestamp = time.time()
        raw_positions = self.master_robot.sync_read("Present_Position", normalize=False)
        angles = self._raw2angle(raw_positions)
        master_obs, raw_joints, smoothed_joints = self._motor2jaka(angles, return_details=True)
        output_joints = smoothed_joints
        if output_joints is not None:
            for index, joint in enumerate(output_joints, start=1):
                master_obs[f"joint{index}.pos"] = joint

        action = {f"joint{i}.pos": master_obs[f"joint{i}.pos"] for i in range(1, 7)}
        if "gripper.pos" in master_obs:
            action["gripper.pos"] = master_obs["gripper.pos"]

        with self._master_state_lock:
            previous_sample = self._latest_action_sample
            new_sample = {
                "timestamp": sample_timestamp,
                "action": _copy_action_dict(action),
            }
            self._prev_action_sample = previous_sample
            self._latest_action_sample = new_sample
            self._latest_joint_velocity = self._estimate_velocity(previous_sample, new_sample)
            self._master_joint_positions = [action[f"joint{i}.pos"] for i in range(1, 7)]
            self._gripper_state = action.get("gripper.pos", 0.0)
            self._latest_action = _copy_action_dict(action)

        self._log_master_sample(
            raw_positions=raw_positions,
            angles=angles,
            raw_joints=raw_joints,
            smoothed_joints=smoothed_joints,
            estimated_velocity=self._latest_joint_velocity.copy(),
            action=action,
        )

    def _master_read_loop(self) -> None:
        next_tick = time.perf_counter()
        while not self._stop_thread:
            try:
                self._update_master_state()
            except Exception as exc:
                logger.warning(f"Failed to refresh master state: {exc}")

            next_tick += self._master_read_period
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

    def _raw2angle(self, raw: dict):
        # 将舵机原始数据转换为角度 注意是根据当前的homing_offset进行转换的，确保HOMING_OFFSET正确才能得到正确的角度
        angles = {}
        for motor_name, homing_offset in self.HOMING_OFFSET.items():
            diff = raw[motor_name] - homing_offset
            if diff > 2048:
                diff -= 4096
            elif diff < -2048:
                diff += 4096
            angle = diff * 360 / 4096
            angles[motor_name] = angle * self.JOINT_DIRECTION[motor_name] # 根据方向调整角度符号
        return angles

    def _motor2jaka(self, raw, return_details=False):
        jaka_observation = {}
        order = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "extra_joint"]
        
        jaka_joints = [raw[name] for name in order]
        # jaka_joints[0] = ((raw["shoulder_pan"] + 270) % 361) * -1

        # if raw["shoulder_lift"] > 0: raw["shoulder_lift"] = 0
        # if raw["shoulder_lift"] < -180: raw["shoulder_lift"] = -180
        # jaka_joints[1] = (raw["shoulder_lift"] + 180) % 181 - 90

        # if raw["elbow_flex"] > 55: raw["elbow_flex"] = 55
        # if raw["elbow_flex"] < -125: raw["elbow_flex"] = -125
        # jaka_joints[2] = (raw["elbow_flex"] + 125) % 181 - 90

        # if raw["wrist_flex"] > 0: jaka_joints[3] = 360 - raw["wrist_flex"]
        # else: jaka_joints[3] = -raw["wrist_flex"]

        # if raw["wrist_roll"] > 10: raw["wrist_roll"] = 10
        # if raw["wrist_roll"] < -170: raw["wrist_roll"] = -170
        # jaka_joints[4] = (raw["wrist_roll"] + 170) % 181 * -1 + 90

        # jaka_joints[5] = 180 - raw["extra_joint"] + 104

        # jaka_joints = [angle * deg2rad for angle in jaka_joints] # 使用TCP/IP方式控制不需要转换为弧度

        # 主臂坐标平滑
        smoothed_angles = self.master_smoother.update(jaka_joints)

        for i, joint in enumerate(smoothed_angles, start=1):
            jaka_observation[f"joint{i}.pos"] = joint

        gripper_raw = raw.get("gripper", 0)
        jaka_observation["gripper.pos"] = 1.0 if gripper_raw > 40 else 0.0
        if return_details:
            return jaka_observation, list(jaka_joints), list(smoothed_angles)
        return jaka_observation

    def get_action(self) -> dict[str, Any]:
        """获取当前动作（用于记录）"""
        if not self._connected:
            raise RuntimeError("Teleoperator not connected")

        output_time = time.time()

        with self._master_state_lock:
            control_action = self._build_control_action(output_time)

        if control_action is not None:
            with self._master_state_lock:
                self._latest_action = _copy_action_dict(control_action)
            return control_action

        self._update_master_state()
        output_time = time.time()
        with self._master_state_lock:
            control_action = self._build_control_action(output_time)
            latest_action = _copy_action_dict(self._latest_action) if self._latest_action is not None else {}

        if control_action is not None:
            with self._master_state_lock:
                self._latest_action = _copy_action_dict(control_action)
            return control_action
        return latest_action
    
    def calibrate(self) -> None:
        """校准"""
        self._calibrated = True
        logger.info("Master calibrated")

    def set_follow_mode(self, follow: bool):
        """设置是否跟随"""
        self._follow_mode = follow

    def set_control_mode(self, mode: str):
        """设置控制模式：joint 或 cartesian"""
        if mode in ["joint", "cartesian"]:
            self._control_mode = mode

    def disconnect(self) -> None:
        """断开连接"""
        self._stop_thread = True

        if self._master_read_thread is not None and self._master_read_thread.is_alive():
            self._master_read_thread.join(timeout=max(0.1, self._master_read_period * 2))
        self._master_read_thread = None
        self.master_smoother.reset()
        with self._master_state_lock:
            self._prev_action_sample = None
            self._latest_action_sample = None
            self._latest_joint_velocity = [0.0] * 6
        self._close_master_log()
        self._connected = False
        
        self.master_robot.disconnect()

    @property
    def action_features(self) -> dict[str, type]:
        """动作特征定义"""
        if self._control_mode == "joint":
            features = {f"joint{i}.pos": float for i in range(1, 7)}
        else:
            features = {"pose": np.ndarray}
        
        features["gripper.pos"] = float
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return self._calibrated

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass
