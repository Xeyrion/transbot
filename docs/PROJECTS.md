# 项目与数据导航

| 项目 | 入口 | 当前阶段 |
|---|---|---|
| 语义建图工作台 | `semantic_mapping/run.sh`，英文 `run_en.sh` | 离线建图与语义回放演示，见项目内 `docs/stage_01.md` |
| PC 遥控台 | `console/run_gui.sh` | 遥控、传感器、机械臂及建图导航 |
| 机械臂 | `arm/gazebo/run_simple.sh` | 2R 仿真、实测几何与真机同步 |
| 墙面任务台 | `wall_task/task_gui/start_task.sh` | 导航、对准和竖线规划；完整画线尚需实机验证 |

根目录只维护项目入口、总览及仓库约定。具体算法和操作说明放回对应子项目。源码历史统一由 Transbot Git 仓库管理。

## 本机资料

`_local/` 为已有诊断材料、辅助工具、临时文件与历史仓库备份，保持忽略。隐藏的 `.workbuddy-ai/`、`.slidep/` 属于本机工具状态，不参与发布。

Windows 旧 `bagtools/` 中的数据集、结果和归档保持原位置；控制机原存储 `~/robot_research/bagtools/` 中的 bag、模型、图片、结果和归档同样保持原位置。它们不随源码迁移，也不进入 GitHub。控制机 `~/transbot/bagfiles/` 等既有数据目录也不移动。

新源码 `semantic_mapping/paths.local.json` 配置 `data_dir` 与 `models_dir`，不提交。可从 `paths.example.json` 复制后编辑。每个录制的当前地图和语义结果继续保存在该录制目录内。

## 版本与发布范围

源码、文档、配置示例及测试可以提交。原始 `.bag`、活动录制尾文件、模型权重、批量图像/数组、生成结果与本机配置不提交。阶段演示完成不等于定位精度已经验证；研究边界以各项目的阶段说明为准。

本次整理保留已有其他子项目的本机修改；没有发布远程仓库，也没有在机器人上执行录制或移动动作。
