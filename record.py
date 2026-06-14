import sys
import time
sys.path.insert(0, r'D:\Desktop\schoolwork_22\club\jaka\lerobot_jaka\lerobot_teleoperator_jaka_teleop\lerobot_teleoperator_jaka_teleop')
sys.path.insert(0, r'D:\Desktop\schoolwork_22\club\jaka\lerobot_jaka\lerobot_robot_jaka\lerobot_robot_jaka')

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
# from jaka_robot import Jaka, JakaConfig
from lerobot_robot_jaka.lerobot_robot_jaka import JakaConfig, Jaka # 若未注入模块，取消注释并使用正确的路径
from lerobot_teleoperator_jaka_teleop.lerobot_teleoperator_jaka_teleop import JakaTeleopConfig, JakaTeleop
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun
from lerobot.scripts.lerobot_record import record_loop
from lerobot.processor import make_default_processors

# 采集配置部分
NUM_EPISODES = 1
EPISODE_TIME_SEC = 20
RESET_TIME_SEC = 5
TASK_DESCRIPTION = "Push the red cube"

# robot_config = JakaConfig(ip = "192.168.1.102",
#                           cameras={
#                         "front":OpenCVCameraConfig(index_or_path=0, warmup_s=1, width=640, height=480, fps=FPS)
#                         }
#         )

robot_config = JakaConfig(ip = "192.168.1.101"
        )

teleop_config = JakaTeleopConfig()
FPS = int(teleop_config.control_freq)

def record():

    # Initialize the robot and teleoperator
    robot = Jaka(robot_config)
    teleop = JakaTeleop(teleop_config)

    # Configure the dataset features
    action_features = hw_to_dataset_features(robot.action_features, "action")
    obs_features = hw_to_dataset_features(robot.observation_features, "observation")
    dataset_features = {**action_features, **obs_features}

    # Create the dataset
    dataset = LeRobotDataset.create(
        repo_id="foggy214/jaka_push_18",
        fps=FPS,
        features=dataset_features,
        robot_type=robot.name,
        use_videos=True,
        image_writer_threads=4,
    )

    # Initialize the keyboard listener and rerun visualization
    _, events = init_keyboard_listener()
    init_rerun(session_name="recording")

    # Connect the robot and teleoperator
    robot.connect()
    teleop.connect()

    # Create the required processors
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    episode_idx = 0
    while episode_idx < NUM_EPISODES and not events["stop_recording"]:
        input("\nPress ENTER to calibrate (slave will move to master position)...")
        print("Calibrating...")
        master_obs = teleop.get_action()
        action = {}
        for i in range(1, 7):
            action[f"joint{i}.pos_normal"] = master_obs[f"joint{i}.pos"]
        if "gripper.pos" in master_obs:
            action["gripper.pos"] = master_obs["gripper.pos"]
        robot.send_action(action)
        time.sleep(5)  # 等待运动完成
        print("Calibration done!")

        robot.init_servo_mode(lpf=teleop_config.servo_lpf, control_freq=FPS) # 进入伺服模式，lpf越低越平滑

        input("\nPress ENTER to start recording...")

        log_say(f"Recording episode {episode_idx + 1} of {NUM_EPISODES}")
        print(f"Recording episode {episode_idx + 1} of {NUM_EPISODES}")

        record_loop(
            robot=robot,
            events=events,
            fps=FPS,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            teleop=teleop,
            dataset=dataset,
            control_time_s=EPISODE_TIME_SEC,
            single_task=TASK_DESCRIPTION,
            display_data=False,
        )

        # Reset the environment if not stopping or re-recording
        if not events["stop_recording"] and (episode_idx < NUM_EPISODES - 1 or events["rerecord_episode"]):
            log_say("Reset the environment")
            print("Reset the environment")
            record_loop(
                robot=robot,
                events=events,
                fps=FPS,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                teleop=teleop,
                control_time_s=RESET_TIME_SEC,
                single_task=TASK_DESCRIPTION,
                display_data=False,
            )

        if events["rerecord_episode"]:
            log_say("Re-recording episode")
            print("Re-recording episode")
            events["rerecord_episode"] = False
            events["exit_early"] = False
            dataset.clear_episode_buffer()
            continue

        dataset.save_episode()
        time.sleep(0.5)  # 给后台写入线程时间
        episode_idx += 1

    # Clean up
    log_say("Stop recording")
    print("Stop recording")
    robot.disconnect()
    teleop.disconnect()
    time.sleep(1)
    dataset.push_to_hub()

if __name__ == "__main__":
    # mp.set_start_method('spawn', force=True)
    # p = mp.Process(target=record)
    # p.start()
    # p.join() 
    record()