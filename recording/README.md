# Recording tools

日常使用控制机工作台的录制入口。remote_record.sh 在启动前校验完整机器人脚本包；机器人尚未部署或版本不一致时会拒绝开始，先在机器人连接后运行 deploy_to_robot.sh，再进行现场短录制验证。

record_full.sh 管理录前检查与单实例，auto_record_cam.sh 管理 recorder、逐话题观察器和守护进程，session_finalize.py 分别报告文件关闭、退出结局、话题/类型与实时监测质量。record_probe.py 只把本地观察器计数传回 GUI，不额外通过 Wi-Fi 拉图。

正式离线算法和标定工具位于 ../semantic/。旧标定、投影和截图辅助脚本已移至项目 archive/legacy_tools/，不作为运行入口。

使用流程见 ../docs/workflow.md，标定说明见 ../docs/calibration.md。
