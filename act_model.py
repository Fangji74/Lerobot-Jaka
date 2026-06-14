"""
ACT模型推理脚本 - 基于Jaka机器人
用法: python eval_act_model.py
"""

import sys
import time
import torch
import numpy as np
from pathlib import Path

# 添加你的自定义模块路径
sys.path.insert(0, r'D:\Desktop\schoolwork_22\club\jaka\lerobot_jaka\lerobot_robot_jaka\lerobot_robot_jaka')
sys.path.insert(0, r'D:\Desktop\schoolwork_22\club\jaka\lerobot_jaka\lerobot_teleoperator_jaka_teleop\lerobot_teleoperator_jaka_teleop')

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.pretrained import PreTrainedPolicy
from jaka_robot import Jaka, JakaConfig
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun
from lerobot.utils.control_utils import init_keyboard_listener

# ============ 配置参数 ============
# 模型路径（训练好的ACT模型）
MODEL_PATH = r"D:\Desktop\schoolwork_22\club\jaka\lerobot_jaka\models\pretrained_model"

# 从数据集获取的动作统计信息
ACTION_MEAN = np.array([-4.92763408, 0.66904743, 1.07804871, 3.12761317, -1.30745801, 1.75976526])
ACTION_STD = np.array([0.37860409, 0.15269376, 0.20857689, 0.02110939, 0.11770948, 0.10527599])

# 机器人配置（与训练时保持一致）
ROBOT_IP = "192.168.1.102"
FPS = 2  # 控制频率
EPISODE_TIME_SEC = 20  # 每个episode时长
WARMUP_TIME_S = 3  # 预热时间
RESET_TIME_S = 5  # 重置时间

# 任务描述
TASK_DESCRIPTION = "Push the red cube"

# 相机配置（与训练时保持一致）
CAMERAS = {
    "front": OpenCVCameraConfig(
        index_or_path=0, 
        warmup_s=1, 
        width=640, 
        height=480, 
        fps=FPS
    )
}


def create_robot():
    """创建并连接机器人（推理模式，不需要遥操作设备）"""
    robot_config = JakaConfig(
        ip=ROBOT_IP,
        cameras=CAMERAS
    )
    robot = Jaka(robot_config)
    return robot


def load_policy(model_path: str, device: str = "cpu"):
    """
    加载训练好的ACT模型
    
    Args:
        model_path: 模型文件夹路径（包含config.json和model.safetensors）
        device: 运行设备，"cpu" 或 "cuda"
    """
    print(f"正在从 {model_path} 加载模型...")
    
    # 使用LeRobot官方API加载模型
    policy = ACTPolicy.from_pretrained(model_path)
    
    # 移动到指定设备
    device = torch.device(device)
    policy = policy.to(device)
    policy.eval()  # 切换到评估模式
    
    print(f"模型加载完成，运行设备: {device}")
    print(f"策略类型: {type(policy).__name__}")
    
    return policy, device


def get_observation_from_robot(robot):
    """
    从机器人获取观测数据，并转换为模型输入格式
    
    Returns:
        dict: 包含图像和关节状态的观测字典
    """
    # 获取机器人观测
    obs = robot.get_observation()
    
    # 提取关节位置（假设有7个关节）
    joint_positions = []
    for i in range(1, 8):  # joint1.pos 到 joint7.pos
        key = f"joint{i}.pos"
        if key in obs:
            joint_positions.append(obs[key])
    
    # 如果只有6个关节，调整
    if len(joint_positions) == 6:
        # 某些机械臂只有6个自由度
        pass
    
    # 提取图像（从相机获取）
    images = {}
    for cam_name in robot.cameras:
        if cam_name in obs:
            images[cam_name] = obs[cam_name]
    
    return {
        "observation.state": np.array(joint_positions, dtype=np.float32),
        "observation.images": images,
        "observation.images.front": images.get("front"),  # 方便访问
    }


def preprocess_observation(obs, device, normalize=True):
    """
    预处理观测数据，转换为模型输入格式
    
    Args:
        obs: get_observation_from_robot 返回的原始观测
        device: torch设备
        normalize: 是否归一化图像到[0,1]
    
    Returns:
        dict: 模型可接受的batch格式
    """
    batch = {}
    
    # 处理关节状态
    state = obs["observation.state"]
    batch["observation.state"] = torch.tensor(state).float().unsqueeze(0).to(device)
    
    # 处理图像
    for cam_name, img in obs["observation.images"].items():
        # img 通常是 numpy array (H, W, C) 格式，值范围 0-255
        if img is not None:
            # 转换为 (C, H, W) 格式并归一化
            img_tensor = torch.tensor(img).float()
            img_tensor = img_tensor.permute(2, 0, 1)  # (H,W,C) -> (C,H,W)
            if normalize:
                img_tensor = img_tensor / 255.0
            batch[f"observation.images.{cam_name}"] = img_tensor.unsqueeze(0).to(device)
    
    return batch

def denormalize_action(norm_action, mean, std):
    """
    反归一化动作
    
    ACT模型使用的是 MEAN_STD 归一化：
    normalized = (real - mean) / std
    所以：real = normalized * std + mean
    """
    return norm_action * std + mean

def send_action_to_robot(robot, action, debug=True):
    """发送动作到机器人"""
    if isinstance(action, torch.Tensor):
        action = action.squeeze(0).cpu().numpy()
    
    # 反归一化
    real_action = denormalize_action(action, ACTION_MEAN, ACTION_STD)
    
    if debug:
        print(f"归一化动作: [{', '.join([f'{x:+.3f}' for x in action])}]")
        print(f"实际动作:   [{', '.join([f'{x:+.3f}' for x in real_action])}]")
    
    # 检查动作是否合理
    if np.abs(real_action).max() > 6.28:  # 超过 2π
        print(f"⚠️ 警告: 动作值过大 ({np.abs(real_action).max():.2f})，可能有问题")
    
    # 构建动作字典
    action_dict = {}
    for i in range(6):
        action_dict[f"joint{i+1}.pos"] = float(real_action[i])
    
    robot.send_action(action_dict)


def run_inference_episode(robot, policy, device, episode_idx: int):
    """
    运行一个episode的自主推理
    
    Args:
        robot: 机器人实例
        policy: 加载的模型
        device: 运行设备
        episode_idx: episode编号
    """
    log_say(f"开始执行 Episode {episode_idx + 1}")
    print(f"\n{'='*50}")
    print(f"Episode {episode_idx + 1}: {TASK_DESCRIPTION}")
    print(f"{'='*50}")
    
    policy.reset()
    
    obs = get_observation_from_robot(robot)
    
    start_time = time.time()
    step = 0
    inference_times = []
    
    while (time.time() - start_time) < EPISODE_TIME_SEC:
        step_start = time.time()
        
        obs = get_observation_from_robot(robot)
        
        batch = preprocess_observation(obs, device)
        
        with torch.no_grad():
            action = policy.select_action(batch)
        
        # select_action 返回的是tensor，需要转换为numpy
        if isinstance(action, torch.Tensor):
            action_np = action.squeeze(0).cpu().numpy()
        else:
            action_np = action
        
        send_action_to_robot(robot, action_np)
        
        inference_time = time.time() - step_start
        inference_times.append(inference_time)
        
        sleep_time = max(0, 1.0 / FPS - inference_time)
        time.sleep(sleep_time)
        
        step += 1
        
        # 打印进度
        if step % 50 == 0:
            elapsed = time.time() - start_time
            print(f"  步数: {step}, 已用时: {elapsed:.1f}s / {EPISODE_TIME_SEC}s")
    
    # 统计信息
    avg_inference = np.mean(inference_times) * 1000
    print(f"\nEpisode {episode_idx + 1} 完成!")
    print(f"  总步数: {step}")
    print(f"  平均推理时间: {avg_inference:.2f}ms")
    print(f"  实际频率: {step / EPISODE_TIME_SEC:.1f} Hz")


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"检测到设备: {device}")
    
    try:
        policy, device = load_policy(MODEL_PATH, device)
    except Exception as e:
        print(f"模型加载失败: {e}")
        print("请检查模型路径是否正确，以及是否包含 config.json 和模型权重文件")
        return
    
    print("\n正在连接机器人...")
    robot = create_robot()
    
    try:
        robot.connect()
        print("机器人连接成功!")
    except Exception as e:
        print(f"机器人连接失败: {e}")
        return
    
    _, events = init_keyboard_listener()
    init_rerun(session_name="act_inference")
    
    print(f"\n预热 {WARMUP_TIME_S} 秒...")
    time.sleep(WARMUP_TIME_S)
    
    episode_idx = 0
    while episode_idx < NUM_EPISODES and not events["stop_recording"]:
        input(f"\n按 Enter 开始 Episode {episode_idx + 1}...")
        
        run_inference_episode(robot, policy, device, episode_idx)
        
        episode_idx += 1
        
        if episode_idx < NUM_EPISODES and not events["stop_recording"]:
            log_say(f"重置环境 {RESET_TIME_S} 秒")
            print(f"等待 {RESET_TIME_S} 秒后开始下一个episode...")
            time.sleep(RESET_TIME_S)
    
    log_say("推理结束，断开连接")
    print("\n正在断开机器人连接...")
    robot.disconnect()
    print("完成!")


# 配置参数
NUM_EPISODES = 1  # 运行的episode数量
FPS = 2  # 控制频率（与训练时保持一致）


if __name__ == "__main__":
    main()