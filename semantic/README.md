# 离线语义地图

正式入口：`bash semantic/run_semantic.sh <bag>`。使用同次录制的全部分卷，结果写入 `results/semantic/run_…/`，保留模型、来源、时间配对与几何信息。

当前地图来自该会话的 `maps/<录制名>/association.json`。先在数据阅读中建图，再运行语义流程；没有当前地图时使用 odom 坐标，不加载散落旧图或旧 AMCL 的 map 坐标。

```bash
STEP=1 MAX_FRAMES=120 bash semantic/run_semantic.sh bags/<会话>/<首卷>.bag
# 自训练权重：SEMANTIC_WEIGHTS=/path/to/best.pt bash semantic/run_semantic.sh <bag>
```

默认检测环境使用控制机独立 Conda Python，ROS bag 读取使用系统 Python2。相机安装边缺失时应提供 `BAGTOOLS_CALIBRATION`；显式 `ALLOW_ESTIMATED_EXTRINSICS=1` 只用于探索，不视为实测标定。

算法细节见 [semantic_algorithm.md](../docs/semantic_algorithm.md)，标定步骤见 [calibration.md](../docs/calibration.md)。物品点代表可见表面样本；动态类别默认不加入静态地图。裂缝数据集暂未开始，不把已有通用类别模型视为裂缝模型。
