'''
本程序用于校准舵机和jaka，得到正确的舵机转换数据
'''

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
import time
import json


PI = 3.14159
deg2rad = PI / 180

# 舵机配置部分
PORT = "COM8" # 臂实际端口号
BAUDRATE = 1000000 # STS3215舵机默认波特率

# # 定义七个舵机（标号已经按顺序固定）
# MOTORS_CONFIG = {
#     "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
#     "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
#     "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
#     "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
#     "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
#     "extra_joint": Motor(6, "sts3215", MotorNormMode.DEGREES),
#     "gripper": Motor(7, "sts3215", MotorNormMode.RANGE_0_100)
# }

# # 初始校准数据
# # range_min=0, range_max=4095 表示不做缩放
# CALIBRATION = {
#     "shoulder_pan": MotorCalibration(id=1, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
#     "shoulder_lift": MotorCalibration(id=2, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
#     "elbow_flex": MotorCalibration(id=3, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
#     "wrist_flex": MotorCalibration(id=4, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
#     "wrist_roll": MotorCalibration(id=5, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
#     "extra_joint": MotorCalibration(id=6, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
#     "gripper": MotorCalibration(id=7, drive_mode=0, homing_offset=0, range_min=0, range_max=4095),
# }

# 使用六关节的so101实验代码
# 定义六个舵机（标号已经按顺序固定）
MOTORS_CONFIG = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
    "extra_joint": Motor(6, "sts3215", MotorNormMode.DEGREES)
}

# 初始校准数据
# range_min=0, range_max=4095 表示不做缩放
CALIBRATION = {
    "shoulder_pan": MotorCalibration(id=1, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
    "shoulder_lift": MotorCalibration(id=2, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
    "elbow_flex": MotorCalibration(id=3, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
    "wrist_flex": MotorCalibration(id=4, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
    "wrist_roll": MotorCalibration(id=5, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),
    "extra_joint": MotorCalibration(id=6, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095)
}

# 需要连续追踪的关节（jaka可从-360到360）
CONTINUOUS_JOINTS = {"shoulder_pan", "wrist_roll", "extra_joint"}  # 对应关节1、4、6
# 有限范围关节（ jaka从-90到90，用 ±180° 归一化即可）
LIMITED_JOINTS = {"shoulder_lift", "elbow_flex", "wrist_flex"}  # 对应关节2、3、5

# # 新的校准数据 只校准homing_offset和joint_direction（实际需要根据调整结果修改）
# HOMING_OFFSET = {
#     "shoulder_pan": 2048,
#     "shoulder_lift": 2048,
#     "elbow_flex": 2048,
#     "wrist_flex": 2048,
#     "wrist_roll": 2048,
#     "extra_joint": 2048,
#     "gripper": 0
# }

# # 定义每个关节的方向（1代表与jaka一致，-1代表与jaka相反）
# # 此处未添加自定义方向调整，直接写死了
# JOINT_DIRECTION = {
#     "shoulder_pan": -1,
#     "shoulder_lift": -1,
#     "elbow_flex": -1,
#     "wrist_flex": -1,
#     "wrist_roll": 1,
#     "extra_joint": -1,
#     "gripper": 1,
# }

#so101实验代码
# 新的校准数据 只校准homing_offset和joint_direction（实际需要根据调整结果修改）
HOMING_OFFSET = {
    "shoulder_pan": 2048,
    "shoulder_lift": 2048,
    "elbow_flex": 2048,
    "wrist_flex": 2048,
    "wrist_roll": 2048,
    "extra_joint": 2048
}

# 定义每个关节的方向（1代表与jaka一致，-1代表与jaka相反）
# 此处未添加自定义方向调整，直接写死了
JOINT_DIRECTION = {
    "shoulder_pan": -1,
    "shoulder_lift": -1,
    "elbow_flex": -1,
    "wrist_flex": -1,
    "wrist_roll": 1,
    "extra_joint": -1
}

bus = FeetechMotorsBus(
    port=PORT,
    motors=MOTORS_CONFIG,
    calibration=CALIBRATION,
)

def set_homing_offset(use_cached: bool = False):
    # 使用之前文件中保存的校准数据
    if use_cached:
        # with open("homing_offset.json", "r") as f:
        with open("homing_offset.json", "r") as f:
            cached_offset = json.load(f)
            for key in HOMING_OFFSET.keys():
                if key in cached_offset:
                    HOMING_OFFSET[key] = cached_offset[key]
        # print("Loaded homing offsets from homing_offset.json:")
        print("Loaded homing offsets from homing_offset.json:")
    else:
        print("Starting motor calibration...")
        try:
            for motor_name in MOTORS_CONFIG.keys():
                print(f"\nAdjust the position of {motor_name}, then press ENTER to set homing offset.")
                input()
                raw = bus.sync_read("Present_Position", normalize=False)
                print(f"Set {motor_name} homing offset to {raw[motor_name]}")

                HOMING_OFFSET[motor_name] = raw[motor_name]
                time.sleep(0.5)
            # with open("homing_offset.json", "w") as f:
            with open("homing_offset_so101.json", "w") as f:
                json.dump(HOMING_OFFSET, f, indent=4)
        except KeyboardInterrupt:
            print("\nAdjustment stopped.")

# def raw2angle(raw: dict):
#     # 将舵机原始数据转换为角度 注意是根据当前的homing_offset进行转换的，确保HOMING_OFFSET正确才能得到正确的角度
#     angles = {} 
#     for motor_name, homing_offset in HOMING_OFFSET.items():
#         angle = (raw[motor_name] - homing_offset) * 360 / 4096
#         angles[motor_name] = angle
#     return angles

def raw2angle(raw: dict):
    # 将舵机原始数据转换为角度 注意是根据当前的homing_offset进行转换的，确保HOMING_OFFSET正确才能得到正确的角度
    angles = {}
    for motor_name, homing_offset in HOMING_OFFSET.items():
        diff = raw[motor_name] - homing_offset
        if diff > 2048:
            diff -= 4096
        elif diff < -2048:
            diff += 4096
        angle = diff * 360 / 4096
        angles[motor_name] = angle * JOINT_DIRECTION[motor_name] # 根据方向调整角度符号
    return angles

def angle2raw(angles: dict):
    # 将角度转换为舵机原始数据 注意是根据当前的homing_offset进行转换的，确保HOMING_OFFSET正确才能得到正确的原始值
    raw_values = {}
    for motor_name, angle in angles.items():
        homing_offset = HOMING_OFFSET[motor_name] * JOINT_DIRECTION[motor_name] # 根据方向调整homing_offset
        raw = (angle * 4096 / 360) + homing_offset
        raw = round(raw) % 4096
        raw_values[motor_name] = raw
    return raw_values

def motor2jaka(raw):
    # 爪子未提取
    order = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "extra_joint"]
    
    jaka_joints = [raw[name] * deg2rad for name in order]
    
    return jaka_joints


if __name__ == "__main__":
    bus.connect()
    set_homing_offset(use_cached=False)

    try:
        while True:
            # 注意normalize=False才会返回原始值，如果用默认的normalize=True则会根据校准数据转化成角度
            raw_positions = bus.sync_read("Present_Position", normalize=False)
            positions = raw2angle(raw_positions)
            output = "\r"
            for motor_name, pos in positions.items():
                # pos 是原始值 (0-4095)
                output += f"{motor_name}:{pos:6.2f}  "
            print(output, end="")
            
            time.sleep(0.5)  # 约2Hz

    except KeyboardInterrupt:
        print("\n\n用户停止读取。")
    finally:
        bus.disconnect()
        print("已断开连接。")