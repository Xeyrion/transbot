# 项目架构

跨项目的主机分工、控制权和文档维护以[整体架构](../../docs/ARCHITECTURE.md)为准。本页只说明语义工作台内部实现。

工作台 `workbench.py` 提供两个入口：录制控制与数据阅读。实际传感器原图在机器人本地录制，控制机通过 SSH 发命令、查看状态和回传会话。

数据阅读 `bag_quality_gui.py` 负责会话列表、分析、预览、查看和建图任务。常驻阅读器 `bag_viewer_server.py` 使用本机 ROS 的 Python（Melodic 为 Python 2、Noetic 为 Python 3）读取 ROS bag，两个 Python 版本共用 `viewer_core.py` 的分卷、坐标与地图关联规则。

`mapping_job.py` 统一界面与命令行建图：检查连续区间 → 过滤旧地图/AMCL 变换 → 私有 ROS master 下 SLAM → 验证 → 更新当前地图。`mapping_probe.py` 只检查导航输入；`rebuild_map.py` 是内部解算器。

## 唯一当前地图

每组录制只有一份当前地图：`bags/<会话>/maps/<录制名>/`，包含 `map.pgm`、`map.yaml`、`trajectory.bag`、`association.json`。阅读器和语义流程只从这里自动加载。不存在当前地图就使用里程计视图，不猜测其他地图文件。

替换成功时，旧地图整体移到 `archive/maps/`。构建输入、日志和状态集中放在 `archive/builds/`。`paths.local.json` 的 `archive_dir` 或 `BAGTOOLS_ARCHIVE_DIR` 可将建图档案放到外接磁盘。档案目录不参与自动地图选择。关联记录保留原始分卷指纹与有效时间范围；部分建图只在对应时段使用地图坐标。

## 其他目录

- `bags/`：原始采集与回传信息，不在整理中改写原始包。
- 数据根目录中 `<会话>/semantic/<录制名>/`：各次语义运行、地图快照、图库和比较。
- 共享模型目录：由本机配置指定，权重不随源码提交。
- 本地 `results/`：跨会话诊断和验证材料。
- `archive/experiments/`：过去的试验输出。
- `archive/reports/`：日期报告、截图和验证证据。
- `archive/backups/`：整理前已有的源码备份。
- `archive/legacy_tools/`：已退出正式入口的旧辅助脚本。

源码历史使用 Git。归档目录被 Git 忽略，也不会被会话目录扫描器当成数据会话。

正式源码位于 `transbot/semantic_mapping/`。`project_paths.py` 为工作台、阅读器、录制 GUI、回传与模型入口统一解析路径；`paths.local.json` 引用保留在原处的数据和模型，环境变量可覆盖。保留 `BAGTOOLS_*`、ROS 节点及录制协议标识，以沿用现有配置和机器人脚本。
