# semantic — 物体级语义地图（离线原型）

从一段 bag 里把 RGB 检测结果投到地图坐标系，多帧聚合成物体。整条链在控制机上跑，
不需要机器人，也不需要 torch（用 OpenCV 的 dnn 模块读 Darknet 模型）。

```bash
cd ~/robot_research/bagtools
bash semantic/run_semantic.sh bags/session_x/cam_xxx_0.bag
# 结果在 /tmp/semantic_cam_xxx_0/：objects.json + semantic_map.png
```

可调参数（环境变量）：`STEP`（抽样间隔，默认 1.0 秒）、`CONF`（检测阈值 0.25）、
`RADIUS`（同一物体的归并半径 0.6 m）、`MIN_HITS`（少于几次观测就丢掉，默认 3）、
`BACKEND`（`yolo26` 或 `cv2`，默认 yolo26）。

## 检测器

两个后端，同一套投影代码：

| 后端 | 模型 | 依赖 | 实测（120 帧 / 实验室） |
|---|---|---|---|
| `yolo26` | YOLO26n（Ultralytics，端到端无 NMS） | python≥3.8 + torch | 227 个观测 / 9 秒 |
| `cv2` | YOLOv3-tiny（OpenCV dnn） | 只要 cv2 | 5 个观测 / 2 秒 |

控制机的系统 python 是 3.6，装不了 ultralytics，所以 YOLO26 跑在独立的 miniconda 里：

```bash
# Ubuntu 18.04 是 glibc 2.27，新版 Miniconda 要 2.28，得用 py39_4.12.0 这个版本
curl -sSL -o /tmp/mc.sh https://repo.anaconda.com/miniconda/Miniconda3-py39_4.12.0-Linux-x86_64.sh
bash /tmp/mc.sh -b -p "$HOME/miniconda3"
"$HOME/miniconda3/bin/pip" install ultralytics
```

`run_semantic.sh` 会自动找 `~/miniconda3/envs/yolo/bin/python` 或 `~/miniconda3/bin/python`，
都没有就退回 `cv2` 后端。首次运行会自动下载 `yolo26n.pt`（5.3 MB）。

## 底图

`run_semantic.sh` 会自动找这个会话的地图（`viewer_core.association()` 认
`.map.json` 关联或包旁边的 `map_*.yaml`），垫在物体底下当背景，这样能看出物体
在哪面墙边。没有地图的会话就只画轨迹和物体。

单独跑 `cluster_objects.py` 时用 `--map <yaml>` 指定。只支持 yaw=0 的地图
（gmapping 出来的都是），带旋转的先跳过。

## 类别白名单

COCO 在陌生场景会误检——实验室里检出了 `airplane`、`hot dog`。用白名单滤掉：

```bash
python3 semantic/cluster_objects.py <frames_dir> --classes 'chair,dining table,laptop,tv'
```

实测同一批数据：227 个观测 → 白名单后 93 个，聚类出 4 把椅子 + 1 个显示器。

## 三步在做什么

| 步骤 | 脚本 | 干什么 |
|---|---|---|
| 1 | `extract_frames.py`（py2） | 按时间抽样，落盘彩色帧 / 深度 / `map←camera` 的 4×4 |
| 2 | `detect_project.py`（py3） | YOLO 检测 → 框中心取深度中位数 → 反投影到 map |
| 3 | `cluster_objects.py`（py3） | 同类别按距离聚类，一个簇 = 一个物体，出图 |

第 1 步必须是 py2：Melodic 的 `rosbag` 只有 py2 版本。

## 坐标链：每批包缺的环节不一样

这是最容易踩的坑。处理时按 `pose.FILL_INS` 把缺的静态环节补上——都是固定安装
尺寸，补进去和真值等价：

| 环节 | 值 | 谁提供 |
|---|---|---|
| `base_footprint → base_link` | (0, 0, 0.08) | URDF 的固定关节，有的包没录到 |
| `base_link → camera_link` | (0.08, 0, 0.10403), yaw −0.05 | `transbot_record.launch` 里的 static_transform_publisher |

**用导航栈录的包缺第二条**：`record_full.sh` 发现相机和雷达话题都在就跳过
`roslaunch`，而那条静态变换只有 `transbot_record.launch` 会发。已在
`record_full.sh` 里补了 `ensure_camera_tf()`，用录制台录的包不会再缺。

## 录一个新包要注意什么

1. **用录制台录**，别在导航栈跑着的时候录。这样 TF 完整，而且
   `depth_align=true` 会把深度对齐到彩色帧。
2. **走慢一点，每个物体从两三个角度看**。同一物体被看到 5 次以上，聚类才稳。
3. **时长 1–3 分钟**，距离跨度几米就够。
4. **物体离相机 1.5–3 m**。太近看不全，太远深度噪、小物体也检不出。
5. **别在强光或逆光下**。Astra 是结构光，太阳直射会失效。
6. 目标是 COCO 里的类别：`chair` `dining table` `laptop` `tvmonitor` `bottle`
   `person` `keyboard` `mouse` 等。

## 现在的能力边界（别把它当成品）

- **只有 COCO 80 类**。实验室的裂缝、混凝土块、铁柱都不在里面——要这些得自己标
  数据训练。
- **检出率低**。YOLOv3-tiny 输入 416×416，远处小物体容易漏；实测 120 帧只出 10
  个观测。换 YOLOv5n/v8n（需要 torch）或调低 `CONF` 能改善。
- **没标定过精度**。目前用最近邻到激光点的距离做粗检（中位 3.6 cm），只能证明
  "没有大错"，不能证明厘米级准确。
- **未对齐的包会偏**。11-12 那批 `depth_align` 没生效，深度在
  `camera_depth_optical_frame`，现在只是缩放对齐，没做重投影——用录制台新录的包
  没这个问题。

## 自己做类别（COCO 里没有的东西）

裂缝、混凝土块、铁柱这些 COCO 没有，要自己标。完整流程见
[LABELING.md](LABELING.md)：抽图 → labelImg 标注 → 整理成 YOLO 数据集 → 训练。

```bash
python2 semantic/extract_images.py <bag> /tmp/dataset_raw --fps 1 --max 300
# 在 Windows 上用 labelImg 标（格式选 YOLO，存成同名 .txt）
python3 semantic/make_yolo_dataset.py /path/to/labeled /tmp/mydata \
    --names 'crack,concrete_block,steel_column'
```

训练用装好 ultralytics 的那个 python（`~/miniconda3/bin/python`），训完把
`best.pt` 路径传给 `SEMANTIC_WEIGHTS` 就能接回这条流水线。

## 文件

```
semantic/
├── pose.py               静态环节的已知值 + 极简 3D TF（py2/py3 通用）
├── extract_frames.py     抽帧（py2）
├── detect_project.py     YOLO + 投影（py3）
├── cluster_objects.py    聚类 + 出图（py3）
├── calibrate_camera.py   相机安装位姿标定（两个竖直面 + 尺子量的高度）
├── extract_images.py     抽图做标注数据集（py2）
├── make_yolo_dataset.py  整理成 train/val + data.yaml（py3）
├── LABELING.md           自己标类别的完整流程
└── run_semantic.sh       一条命令串起来
```

模型放在 `~/robot_research/bagtools/models/`（`yolov3-tiny.cfg/.weights` +
`coco.names`，约 34 MB），可用 `SEMANTIC_MODELS` 改路径。
