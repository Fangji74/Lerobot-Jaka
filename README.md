# LeRobot JAKA Adapter

本项目用于将 JAKA 机械臂和同构主臂接入 LeRobot，支持遥操作、数据采集、数据集合并和本地策略推理。

当前版本通过 TCP 协议控制 JAKA 机械臂，不再依赖旧版 JAKA Python SDK。

## 内容

- `lerobot_robot_jaka/`：JAKA 机械臂的 LeRobot robot 适配实现
- `lerobot_teleoperator_jaka_teleop/`：同构主臂遥操作端的 LeRobot teleoperator 适配实现
- `record.py`：主从臂数据采集程序
- `test_teleop.py`：遥操作链路测试
- `test_motor.py`：主臂舵机读取与关节映射测试
- `dataset_merge.py`：多个 LeRobot 数据集合并
- `act_model.py`：本地 ACT 模型推理

## 文档

详细环境配置、设备连接、运行流程和注意事项见：

- [README/README.md](README/README.md)

## 注意

- `test_dataset.py` 包含本地私有测试配置，已通过 `.gitignore` 排除。
- 数据集、日志、模型权重和 Hugging Face token 不应提交到 GitHub。
- 完整数据集和模型建议使用 Hugging Face Dataset/Model、GitHub Release 或其他外部存储管理。
