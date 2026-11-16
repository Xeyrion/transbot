# Transbot Bagtools

控制机上的录制、回传、数据阅读、二维建图和离线语义地图工具。日常从一个工作台进入，运行时只使用当前地图。

```bash
cd ~/robot_research/bagtools
./run.sh           # 中文工作台
./run_en.sh        # English workbench
```

在“数据阅读”中选择会话文件夹，点击“建图 / 重新建图”。同次录制的分卷自动合并；已有当前地图时确认替换。成功后直接使用新图，旧图归档，失败或取消不替换当前结果。

## 目录

```text
bagtools/
  *.py, *.sh           工作台、阅读器、建图入口
  recording/           录制与回传
  semantic/            RGB-D、检测、投影、融合、标定
  tests/               自动测试
  locale/              界面翻译
  docs/                当前文档
  bags/<会话>/         原始 bag 与会话信息
    maps/<录制名>/     当前地图、轨迹、association.json
  results/             当前研究输出（例如语义运行）
  archive/             旧地图、构建日志、实验、历史报告与备份
```

源码和当前文档由 Git 管理；bag、模型权重、输出和归档不提交。项目是独立本地仓库，尚未发布远程。使用 `git status`、`git diff` 查看改动，确认后提交；不再为每次代码修改复制一份源码备份。

## 文档

- [工作流](docs/workflow.md)
- [建图按钮](docs/mapping_button.md)
- [架构与目录](docs/architecture.md)
- [相机外参](docs/calibration.md)
- [语义算法](docs/semantic_algorithm.md)
- [项目状态](docs/project_status.md)

## 命令行

```bash
./rebuild_map.sh bags/<会话>/<首卷>.bag            # 首次建图
./rebuild_map.sh bags/<会话>/<首卷>.bag --replace  # 更新当前地图，旧图归档
./build_maps.sh --dry-run                        # 查看批量任务
./build_maps.sh                                 # 给尚无当前地图的会话建图
```

命令行与界面使用同一套 MappingJob；`rebuild_map.py` 是内部 ROS 解算器。二维建图使用雷达、里程计和底盘 TF，相机标定用于后续物品投影。地图完整性取决于实际采集覆盖，不因成功出图就自动获得精度保证。

验证：在控制机 source ROS 后运行 `BAGTOOLS_GUI_TEST=1 xvfb-run -a python3 -m unittest discover -s tests -v`。ROS 端到端检查为 `python3 tests/mapping_integration_checks.py "$PWD"`。
