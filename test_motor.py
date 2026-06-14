import time
import jkrc

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorCalibration, MotorNormMode

PORT = "COM8"                    # 主臂端口号
BAUDRATE = 1000000               # STS3215舵机默认波特率

PI = 3.14159
deg2rad = PI / 180

# 格式: "名称": Motor(ID, 型号, 归一化模式)
# 注意：名称可以自定义，只要唯一即可
MOTORS_CONFIG = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
    "extra_joint": Motor(6, "sts3215", MotorNormMode.DEGREES),
    "gripper": Motor(7, "sts3215", MotorNormMode.RANGE_0_100)
}


# 创建假的校准数据
# range_min=0, range_max=4095 表示不做缩放
CALIBRATION = {
    "shoulder_pan": MotorCalibration(id=1, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),# 初始-273 反
    "shoulder_lift": MotorCalibration(id=2, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),# 同构-180到0 jaka-90到90
    "elbow_flex": MotorCalibration(id=3, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),# 同构55到-125jaka 90到-90
    "wrist_flex": MotorCalibration(id=4, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),# 同构-180到180 jaka 180到-180
    "wrist_roll": MotorCalibration(id=5, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),# 同构10到-170 jaka 90到-90
    "extra_joint": MotorCalibration(id=6, drive_mode=0, homing_offset=2048, range_min=0, range_max=4095),# 同构0作为初始点，jaka增减相反
    "gripper": MotorCalibration(id=7, drive_mode=0, homing_offset=0, range_min=0, range_max=4095),# 大于40认为爪子开，小于认为关
}

bus = FeetechMotorsBus(
    port=PORT,
    motors=MOTORS_CONFIG,
    calibration=CALIBRATION,
)

def motor2jaka(raw):
    jaka_joints = [0]*6
    jaka_joints[0] = ((raw["shoulder_pan"] + 270) % 361) * -1

    if raw["shoulder_lift"] > 0: raw["shoulder_lift"] = 0
    if raw["shoulder_lift"] < -180: raw["shoulder_lift"] = -180
    jaka_joints[1] = (raw["shoulder_lift"] + 180) % 181 - 90

    if raw["elbow_flex"] > 55: raw["elbow_flex"] = 55
    if raw["elbow_flex"] < -125: raw["elbow_flex"] = -125
    jaka_joints[2] = (raw["elbow_flex"] + 125) % 181 - 90

    if raw["wrist_flex"] > 0: jaka_joints[3] = 360 - raw["wrist_flex"]
    else: jaka_joints[3] = -raw["wrist_flex"]

    if raw["wrist_roll"] > 10: raw["wrist_roll"] = 10
    if raw["wrist_roll"] < -170: raw["wrist_roll"] = -170
    jaka_joints[4] = (raw["wrist_roll"] + 170) % 181 * -1 + 90

    jaka_joints[5] = 180 - raw["extra_joint"] + 104

    jaka_joints = [angle * deg2rad for angle in jaka_joints]
    return jaka_joints

if __name__ == "__main__":
    print(f"正在连接 {PORT}...")
    bus.connect()

    print(f"成功连接！正在读取 {len(MOTORS_CONFIG)} 个舵机的数据...")
    print("-" * 60)
    print("关节位置 (原始值 0-4095，2048≈中位)")
    print("-" * 60)

    # robot = jkrc.RC("192.168.1.102")
    # robot = jkrc.RC("10.5.5.100")
    # robot.login()
    # robot.power_on()
    # robot.enable_robot()

    # (_, position) = robot.get_joint_position() 

    try:
        while True:
            # 注意normalize=False才会返回原始值，如果用默认的normalize=True则会根据校准数据转化成角度
            positions = bus.sync_read("Present_Position")
            output = "\r"
            for motor_name, pos in positions.items():
                # pos 是原始值 (0-4095)
                output += f"{motor_name}:{pos:6.2f}  "
            print(output, end="")

            # 若要测试转换函数，可以取消注释以下代码：
            # jaka_joints = motor2jaka(positions)

            # robot.joint_move(jaka_joints,0,False,50)

            # print(jaka_joints)
            
            time.sleep(0.5)  # 约2Hz

    except KeyboardInterrupt:
        print("\n\n用户停止读取。")
    finally:
        bus.disconnect()
        print("已断开连接。")