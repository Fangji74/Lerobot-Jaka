from dataclasses import dataclass, field
from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig

# @RobotConfig.register_subclass("lerobot_robot_jaka")
@dataclass
class JakaConfig(RobotConfig):
    ip: str = "192.168.1.102"

    # 以下三个参数未适配
    use_effort: bool = False
    use_velocity: bool = False
    use_acceleration: bool = False

    # home坐标
    home_translation: list[float] = field(default_factory=lambda: [0,0,0])
    home_orientation_euler: list[float] = field(default_factory=lambda: [0,0,0])

    # 相机
    cameras: dict[str, CameraConfig] = field(default_factory=dict)