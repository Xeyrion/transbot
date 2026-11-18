# 离线语义地图

正式入口：`bash semantic/run_semantic.sh <bag>`。使用同次录制的全部分卷，结果写入 `<录制文件夹>/semantic/<bag 主干>/run_…/`，保留模型、来源、时间配对与几何信息。

栅格底图可选，物品三维定位的数据不能省略：YOLO 用 RGB 输出类别及像素区域；有效深度和彩色相机内参把观测变成相机坐标；相机外参与采集时刻的机器人位姿把观测转换到统一的 map/odom 坐标。即使只定位一帧里的物品，也需要这条定位链；多帧融合在此基础上进行。

当前底图及其关联轨迹来自该会话的 `maps/<录制名>/association.json`。建图和语义生成可以分别运行。没有当前底图时使用记录的里程计位姿，最终图在 odom 坐标中显示轨迹和物品点，位置可能随里程计累计漂移。缺少内参、深度配对或有效位姿的采样帧不会进入三维融合；缺少安装外参时默认报错，检测区域深度不足时保留二维框及拒绝原因。当前入口生成三维语义结果，不会将缺少定位数据的包当作纯二维检测成功。

```bash
STEP=1 MAX_FRAMES=120 bash semantic/run_semantic.sh bags/<会话>/<首卷>.bag
# 自训练权重：SEMANTIC_WEIGHTS=/path/to/best.pt bash semantic/run_semantic.sh <bag>
```

默认检测环境使用控制机独立 Conda Python，ROS bag 读取使用系统 Python2。相机安装边缺失时应提供 `BAGTOOLS_CALIBRATION`；显式 `ALLOW_ESTIMATED_EXTRINSICS=1` 只用于探索，不视为实测标定。

算法细节见 [semantic_algorithm.md](../docs/semantic_algorithm.md)，标定步骤见 [calibration.md](../docs/calibration.md)。物品点代表可见表面样本；动态类别默认不加入静态地图。裂缝数据集暂未开始，不把已有通用类别模型视为裂缝模型。

本轮 COCO 试跑及模型对照：`bags/session_20261117_071239/semantic/cam_20261117_071320/comparisons/session_20261117_071239_demo/README.md`。
使用 138 组相同 RGB-D、已重建地图和记录外参，对比现有 YOLO26n 与 YOLO26s；输出包含二维拒绝记录和地图候选，不将候选数视为真实物品数。
带 `MAP_ASSOCIATION` 可显式使用独立重建结果；`SEMANTIC_WEIGHTS` 选择预训练权重，无需下载 COCO 训练数据集。
