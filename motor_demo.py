# 舵机试验
import time

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorCalibration, MotorNormMode

PORT = "COM8"                    
BAUDRATE = 1000000
MOTORS_CONFIG = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
    # "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    # "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES)
}
PI = 3.14159
deg2rad = PI / 180

CALIBRATION = {
    "shoulder_pan": MotorCalibration(id=1, drive_mode=0, homing_offset=1974, range_min=0, range_max=4095),# 初始-273 反
    # "shoulder_lift": MotorCalibration(id=2, drive_mode=0, homing_offset=979, range_min=0, range_max=4095),
    # "elbow_flex": MotorCalibration(id=3, drive_mode=0, homing_offset=3795, range_min=0, range_max=4095)
}

bus = FeetechMotorsBus(
    port=PORT,
    motors=MOTORS_CONFIG,
    calibration=CALIBRATION,
)

def main():
    bus.connect()
    try:
        for motor in bus._get_motors_list(None):
            bus.write("Torque_Limit", motor, 1000, normalize=False, num_retry=0)
        while True:
            # 注意normalize=False才会返回原始值，如果用默认的normalize=True则会根据校准数据转化成角度
            # positions = bus.sync_read("Present_Position",normalize=False)
            angles = bus.sync_read("Present_Position")
            output = "\r"
            # for motor_name, pos in positions.items():
            #     # pos 是原始值 (0-4095)
            #     output += f"{motor_name}:{pos:6.2f}  "
            # output += " | "
            for motor_name, angle in angles.items():
                # angle 是角度值
                output += f"{motor_name}:{angle:6.2f}  "
            print(output, end="")
            bus.enable_torque([1])

            time.sleep(0.5)  # 约2Hz

    except KeyboardInterrupt:
        print("\n\n用户停止读取。")
    finally:
        bus.disconnect()
        print("已断开连接。")

if __name__ == "__main__":
    main()