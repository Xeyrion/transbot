# -*- coding: utf-8 -*-
"""把一段真实的录制脚本输出喂给英文录制台，截图看有没有漏翻 / 截断。

用 Xvfb 跑，不碰机器人：
    xvfb-run -a python3 i18n_shot.py /tmp/shot.png
"""
import os
import sys

os.environ['BAGTOOLS_LANG'] = 'en'
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import tkinter as tk                      # noqa: E402
import record_gui                         # noqa: E402

BLOCK = u'''== 启动机器人录制：300s -> /home/jetson/bags ==
已在后台启动，PID=2451
用 './remote_record.sh status' 看进度，'log' 看实时日志。
== 实时日志（Ctrl-C 只看不退录制）==
停止可能残留的旧栈 ...
启动数据源：transbot_record.launch
  OK  /camera/rgb/image_raw
  OK  /camera/depth/image_raw
  OK  /scan
  note 这些话题现在没有发布者，先订阅，出现后自动开始记录：
  /cmd_vel /amcl_pose
  （没有 SLAM/定位节点在跑，不等 /map 与 /amcl_pose）
检测到数据源已在运行，跳过启动。
保存目录 : /home/jetson/bags/session_20261111_0143_x（剩余 11 GB）
模式     : full
时长     : 一直录，Ctrl-C 停
压缩     : lz4，每 2048 MB 分卷
保留卷数 : 最新 3 卷（自动删旧）
话题     : /scan /odom /tf /cmd_vel
本次会话目录：/home/jetson/bags/session_20261111_0143_x
=== 会话收尾：完成 3 卷，未关闭 0 卷 ===
地图已保存：/home/jetson/bags/session_20261111_0143_x/map_20261111_0148.pgm + .yaml
提示：数据源仍在后台运行。要停掉：
  pkill -f 'transbot_nav transbot_record.launch'
== 优雅停止 ==
取消会话 [2451]；收尾录制进程 [2469]
会话已停止。
时间：2026-11-11 01:48:12.345678
状态：未在录制
目录：/home/jetson/bags；可用 10.87 GiB
会话数：1；本次分卷合计 4096.0 MiB
日志尾部：
ERROR: 这些话题 6 秒内没有收到任何消息：/camera/depth/image_raw
  话题存在但没有数据，录下来会是空文件。常见原因：
  * 相机：Astra 驱动在订阅者断开后会停流且不自恢复 ——
    pkill -f astrapro.launch 后重新 roslaunch astra_camera astrapro.launch
  * 雷达：/dev/rplidar 没建出来（USB 松了），插好后 ls -l /dev/rplidar
  确实要在缺数据的情况下录，加 -A / --allow-missing。
ERROR: 已经有一个录制会话在跑（PID 4321），先停掉再开：
  停止：./remote_record.sh stop
（清理过期的锁：PID 4321 已经不在了）
ERROR: /home/jetson/bags 只剩 3 GB，至少需要 5 GB
  换目录：/home/jetson/auto_record_cam.sh -o /media/jetson/bags
  或清理：rm -f /home/jetson/bags/*.bag（确认不要了再删）
提示：当前是 x86_64（不像 Jetson）。full 模式含相机原图，
      在控制机上走 WiFi 会掉帧，建议在机器人本地跑，或改用 -p nav。
WARNING: map_saver 失败，地图没存下来（bag 里的 /map 话题仍然在）
（当前没有 /map 话题，跳过存图；只录数据不建图时属正常）
--no-launch：假定数据源已就绪
数据源已启动，未录制。
  查看日志：tail -f /tmp/transbot_record.log
== 检查完成，未录制（-c / --check）==
ERROR: 一个话题都没有
ERROR: 录制尚未完成收尾；保留进程和 .active 文件，不强杀。
ERROR: 仍有进程在收尾，未强杀；请检查 .active 和日志。
ERROR: 会话不完整，保留全部原文件。
== 机器人：jetson@192.168.1.11   目标：/home/jetson ==
--- 现状 ---
--- 备份 ---
--- 上传 ---
--- 校验 ---
  语法 OK
完成。录之前建议先跑一次干检查（只看话题在不在，不录）：
相机就绪（RGB 有图）
相机已停止
未知命令：frobnicate
未知参数：--bogus
PROFILE 只能是 full 或 nav
-d 需要一个秒数
ERROR: 找不到 rosbag。先 source ROS：
ERROR: 连不上 ROS master。先起机器人/本机 ROS：
ERROR: 60 秒内等不到 /scan，看日志：
ERROR: 60 秒内没等到图像，看日志：tail -30 /tmp/astra.log
  WARN 等不到 /scan，--allow-missing 继续（这个包缺这一路）
  WARN 这些话题 6 秒内没有收到任何消息：/scan
ERROR: 找不到 /home/jetson/auto_record_cam.sh
已经有 rosbag 在录（可能是手动起的），先 stop：'''


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/shot.png'
    dump = sys.argv[2] if len(sys.argv) > 2 else None
    root = tk.Tk()
    root.title('record_gui i18n check')
    root.geometry('1180x760+0+0')
    app = record_gui.App(root, record_gui.Remote('jetson@192.168.1.11', 'yahboom'))
    for line in BLOCK.splitlines():
        app._log(line)

    def shoot():
        root.update_idletasks()
        root.update()
        if dump:
            # 日志控件里的原文才是用户真正看到的东西，比截图更能说明问题
            with open(dump, 'w') as f:
                f.write(app.log.get('1.0', 'end'))
        os.system('import -window root %s' % out)
        root.after(300, root.destroy)
    root.after(2500, shoot)
    root.mainloop()
    print(out)


if __name__ == '__main__':
    main()
