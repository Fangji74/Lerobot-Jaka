from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.dataset_tools import merge_datasets

datasets = []

for i in range(1,18):
    dataset = LeRobotDataset(f"foggy214/jaka_push_{i}")
    print(f"数据集 jaka_push_{i} 加载完成，包含 {dataset.num_episodes} 个 episodes，{dataset.num_frames} 帧")
    datasets.append(dataset)

merged = merge_datasets(
    datasets=datasets,
    output_repo_id="foggy214/jaka_test_merged",  # 合并后的数据集名称
    output_dir="./jaka_push_dataset"  # 可选：指定本地存储路径
)

print(f"合并完成！共 {merged.num_episodes} 个 episodes，{merged.num_frames} 帧")