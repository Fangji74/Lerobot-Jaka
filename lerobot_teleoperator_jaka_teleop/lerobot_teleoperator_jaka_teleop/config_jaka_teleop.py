# from dataclasses import dataclass

# from lerobot.teleoperators.config import TeleoperatorConfig


# # @TeleoperatorConfig.register_subclass("lerobot_teleoperator_jaka_teleop")
# @dataclass
# class JakaTeleopConfig(TeleoperatorConfig):
#     master_ip: str = "10.5.5.100"  # 主臂IP地址
#     control_freq: float = 5.0  # 控制频率，单位Hz

from dataclasses import dataclass
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.teleoperators.config import TeleoperatorConfig


# @TeleoperatorConfig.register_subclass("lerobot_teleoperator_jaka_teleop")
@dataclass
class JakaTeleopConfig(TeleoperatorConfig):
    PORT = "COM8"                    # 连接从臂的串口号
    BAUDRATE = 1000000               # STS3215舵机默认波特率
    control_freq: float = 30.0  # 控制频率，单位Hz
    servo_lpf: float = 15.0  # JAKA伺服一阶低通滤波截止频率，单位Hz，适度提高跟手性
    master_read_period: float = 0.02  # 主臂后台读取周期，单位秒
    enable_master_logging: bool = True  # 是否记录主臂后台采样坐标
    master_target_vmax_deg_s: tuple[float, float, float, float, float, float] = (75.0, 75.0, 75.0, 75.0, 75.0, 75.0)  # 适度放宽主臂目标限速，减少快动作时的链路滞后

    CALIBRATION = {
    "shoulder_pan": MotorCalibration(id=1, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
    "shoulder_lift": MotorCalibration(id=2, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
    "elbow_flex": MotorCalibration(id=3, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
    "wrist_flex": MotorCalibration(id=4, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
    "wrist_roll": MotorCalibration(id=5, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
    "extra_joint": MotorCalibration(id=6, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
    "gripper": MotorCalibration(id=7, drive_mode=0, homing_offset=0, range_min=0, range_max=4095),
    }

    MOTORS_CONFIG = {
        "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
        "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
        "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
        "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
        "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
        "extra_joint": Motor(6, "sts3215", MotorNormMode.DEGREES),
        "gripper": Motor(7, "sts3215", MotorNormMode.RANGE_0_100)
    }

# # so101实验代码
# # @TeleoperatorConfig.register_subclass("lerobot_teleoperator_jaka_teleop")
# @dataclass
# class JakaTeleopConfig(TeleoperatorConfig):
#     PORT = "COM8"                    # 连接从臂的串口号
#     BAUDRATE = 1000000               # STS3215舵机默认波特率
#     control_freq: float = 30.0  # 控制频率，单位Hz
#     servo_lpf: float = 15.0  # JAKA伺服一阶低通滤波截止频率，单位Hz，适度提高跟手性
#     master_read_period: float = 0.02  # 主臂后台读取周期，单位秒
#     enable_master_logging: bool = True  # 是否记录主臂后台采样坐标
#     master_target_vmax_deg_s: tuple[float, float, float, float, float, float] = (75.0, 75.0, 75.0, 75.0, 75.0, 75.0)  # 适度放宽主臂目标限速，减少快动作时的链路滞后
#     CALIBRATION = {
#     "shoulder_pan": MotorCalibration(id=1, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
#     "shoulder_lift": MotorCalibration(id=2, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
#     "elbow_flex": MotorCalibration(id=3, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
#     "wrist_flex": MotorCalibration(id=4, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
#     "wrist_roll": MotorCalibration(id=5, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
#     "extra_joint": MotorCalibration(id=6, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095)
#     }

#     MOTORS_CONFIG = {
#         "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
#         "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
#         "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
#         "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
#         "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
#         "extra_joint": Motor(6, "sts3215", MotorNormMode.DEGREES)
#     }