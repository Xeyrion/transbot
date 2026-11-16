# Recording tools

日常使用控制机工作台的录制入口。remote_record.sh 在启动前校验完整机器人脚本包；机器人尚未部署或版本不一致时会拒绝开始，先在机器人连接后运行 deploy_to_robot.sh，再进行现场短录制验证。

record_full.sh 管理录前检查与单实例，auto_record_cam.sh 管理 recorder、逐话题观察器和守护进程，session_finalize.py 分别报告文件关闭、退出结局、话题/类型与实时监测质量。record_probe.py 只把本地观察器计数传回 GUI，不额外通过 Wi-Fi 拉图。

`clock_check.py` 在启动/录制前做只读检查：拒绝重置到 2024 年以前的系统时间、实机上的 `/use_sim_time=true`，以及观察窗口内的系统时钟跳变。Chrony 未同步会明确报告警告，短期稳定不等于已获得外部对时。启动流程不会自行修改正在运行的传感器或 EKF 的时间。

实时监测单独检查 `odom→base_footprint` 的 TF 时间戳、时间戳冻结和超过 0.5 秒的采集/到达时间变化；异常时停止收尾并标记不完整，保留数据。原始消息不会被改写。会话中的 `.clock.json` 保存录前检查证据；`complete` 仍不代表已经完成逐帧时序审计或几何标定。

若机器人开机时间重置，先停止 ROS 数据源，完成对时，再重新启动传感器和 EKF。RTC 是否能在断电后保时需要单独验证；离线机器人不能依赖无法访问的公网 NTP 池。部署前版本检查和 SHA-256 校验必须通过，旧脚本集中保存在机器人 `bagtools_archive/deployments/`。

正式离线算法和标定工具位于 ../semantic/。旧标定、投影和截图辅助脚本已移至项目 archive/legacy_tools/，不作为运行入口。

使用流程见 ../docs/workflow.md，标定说明见 ../docs/calibration.md。

相机安装参数统一读取本目录 `calibration.json`，与 `record_sources.launch` 一起部署并参与版本校验。
录前检查实际 TF 数值及唯一发布源，每次完整录制自动保存配置副本；旧的厂商录制 launch 不再作为配置生效的依据。
