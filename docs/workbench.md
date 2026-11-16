# 统一启动入口

日常在控制机桌面双击 **Transbot 数据工作台**，或在应用菜单中搜索 Transbot。

首页两个入口：

- **录制控制**：打开采集、停止、状态与回传窗口。
- **数据阅读**：打开会话列表，进入文件夹后分析、回放。

它们默认共用控制机数据根目录 `~/robot_research/bagtools/bags`。工作台首页显示该路径，并可打开对应文件夹。机器人采集仍在机器人本地目录进行，回传后才出现在控制机的阅读器中。

工作台本身不会连接机器人或开始录制；点击“录制控制”才打开已有控制台。关闭工作台不会结束其他工具窗口或机器人录制，停止录制请使用录制控制台的“停止”。同一工作台不会重复启动仍在运行的工具窗口。

## 命令行兼容

```bash
~/robot_research/bagtools/run.sh                  # 工作台首页
~/robot_research/bagtools/run.sh --record         # 直接打开录制控制
~/robot_research/bagtools/run.sh --review         # 直接打开阅读器
~/robot_research/bagtools/run.sh /path/to/bags     # 兼容原来的目录参数
```

`recording/run.sh`保留兼容，日常不再需要进入该目录寻找入口。

可选环境变量 `BAGTOOLS_DATA_DIR`（其次 `LOCAL_DIR`）覆盖共用的控制机数据目录；默认无需设置。

桌面快捷方式由 `install_workbench.sh` 安装，执行路径与当前安装目录绑定。移动代码目录后重新运行安装器即可。工具启动日志位于 `~/.cache/transbot_bagtools/launcher/`。
