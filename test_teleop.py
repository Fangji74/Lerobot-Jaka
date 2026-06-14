import time
import sys
sys.path.insert(0, r'D:\Desktop\schoolwork_22\club\jaka\lerobot_jaka\lerobot_teleoperator_jaka_teleop\lerobot_teleoperator_jaka_teleop')
sys.path.insert(0, r'D:\Desktop\schoolwork_22\club\jaka\lerobot_jaka\lerobot_robot_jaka\lerobot_robot_jaka')

from lerobot_robot_jaka.lerobot_robot_jaka import Jaka, JakaConfig  
# from jaka_robot import Jaka, JakaConfig
from lerobot_teleoperator_jaka_teleop.lerobot_teleoperator_jaka_teleop import JakaTeleop, JakaTeleopConfig


def wait_until(target_time: float, spin_margin: float = 0.002) -> None:
    while True:
        remaining = target_time - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > spin_margin:
            time.sleep(remaining - spin_margin)
        else:
            while time.perf_counter() < target_time:
                pass
            return

def main():
    # 配置 IP 地址（根据实际情况修改）
    # SLAVE_IP = "192.168.1.102"   # 从臂IP
    SLAVE_IP = "10.5.5.100"   # 从臂IP
    teleop_config = JakaTeleopConfig()
    teleop_config.master_read_period = 0.01
    teleop_config.control_freq = 62.5
    servo_step_num = 2
    
    # 1. 创建实例
    print("Creating robot instances...")
    slave = Jaka(JakaConfig(ip=SLAVE_IP))
    teleop = JakaTeleop(teleop_config)
    
    print("Connecting to slave arm...")
    slave.connect()
    teleop.connect(calibrate=False)  # 不自动校准
    
    # 3. 可选：校准（将从臂移动到主臂当前位置）
    input("\nPress ENTER to calibrate (slave will move to master position)...")
    print("Calibrating...")
    master_obs = teleop.get_action()
    action = {}
    for i in range(1, 7):
        action[f"joint{i}.pos_normal"] = master_obs[f"joint{i}.pos"]
    if "gripper.pos" in master_obs:
        action["gripper.pos"] = master_obs["gripper.pos"]
    print(master_obs)
    slave.send_action(action)
    time.sleep(6)  # 等待运动完成
    print("Calibration done!")

    # print("test servo mode")
    # master_obs = teleop.get_action()
    # action = {}
    # for i in range(1, 4):
    #     action[f"joint{i}.pos"] = master_obs[f"joint{i}.pos"]
    # for i in range(4, 7):
    #     action[f"joint{i}.pos"] = master_obs[f"joint{i}.pos"] + 0.5
    # if "gripper.pos" in master_obs:
    #     action["gripper.pos"] = master_obs["gripper.pos"]
    # slave.send_action(action)
    # time.sleep(2)

    
    # 4. 开始遥操作
    print("\n" + "="*50)
    print("Starting teleoperation!")
    print("Move the master arm, the slave will follow.")
    print("Press Ctrl+C to stop.")
    print("="*50 + "\n")
    
    try:
        control_freq = teleop_config.control_freq
        dt = 1.0 / control_freq
        next_tick = time.perf_counter()
        loop_lag_warn_threshold = 0.004

        slave.init_servo_mode(
            lpf=teleop_config.servo_lpf,
            control_freq=control_freq,
            servo_step_num=servo_step_num,
        )
        
        while True:
            start_time = time.perf_counter()
            
            # 读取主臂动作
            action = teleop.get_action()
            # print(f"Master action: {action}")
            # 发送给从臂
            slave.send_action(action)
            
            # 控制频率
            elapsed = time.perf_counter() - start_time
            next_tick += dt
            now = time.perf_counter()
            sleep_time = next_tick - now
            if sleep_time > 0:
                wait_until(next_tick)
            else:
                lag = -sleep_time
                next_tick = now
                if lag > loop_lag_warn_threshold:
                    print(
                        f"Warning: control loop missed its {dt:.3f}s period by {lag:.3f}s. "
                        f"This is scheduler timing drift, not a TCP command timeout. "
                        f"Loop work time: {elapsed:.3f}s"
                    )
                
    except KeyboardInterrupt:
        print("\n\nStopping teleoperation...")
    
    # 5. 断开连接
    print("Disconnecting...")
    teleop.disconnect()
    slave.disconnect()
    print("Done!")

if __name__ == "__main__":
    main()