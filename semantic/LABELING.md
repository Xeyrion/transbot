# 从 bag 到自己的检测模型

COCO 的 80 类里没有裂缝、混凝土块、铁柱，这些得自己标。整个流程四步，前三步都能在
控制机上跑，标注那步在你自己电脑上做最顺手。

```
bag → 抽图 → 标注 → 训练 → 接回语义地图
```

## 1. 抽图

```bash
cd ~/robot_research/bagtools
. /opt/ros/melodic/setup.bash
python2 semantic/extract_images.py bags/session_x/cam_xxx_0.bag /tmp/dataset_raw --fps 1 --max 300
```

- `--fps 1`：每秒抽一张。30 Hz 的包里一秒 30 张几乎一样的图，全抽出来标不完。
- `--diff 1.5`：和上一张的缩略图差小于这个值就跳过，去掉机器人停下时的重复帧。
- 输出 `<会话名>_<时间戳>.jpg`，文件名带时间戳，能对回原包。

**标多少够**：每个类别 200–500 个实例起步。裂缝这种细长目标建议 500 以上，
而且要有不同光照、不同角度、不同距离的样本。

## 2. 标注

推荐 **labelImg**（Windows 上直接装）：

```bash
pip install labelImg
labelImg          # 打开后选图片目录，格式切到 YOLO
```

快捷键：`W` 画框、`Ctrl+S` 保存、`D` 下一张、`A` 上一张。

要点：

- **格式选 YOLO**，保存出来是同名 `.txt`（labelImg 左侧工具栏能切）
- 类别名要和 `make_yolo_dataset.py --names` 的顺序一致（0=crack, 1=concrete_block, 2=steel_column）
- **裂缝的框不要画太松**：细长目标画大框，框里大半是背景，模型学不到东西
- 看不清的、目标被挡掉一半的，**直接跳过不标**，比标错强
- 实在觉得画框太粗，可以用 **X-AnyLabeling**（支持 SAM 辅助，点一下就出轮廓），
  但输出要转成 YOLO 框

## 3. 整理成训练集

```bash
python3 semantic/make_yolo_dataset.py /path/to/labeled /tmp/mydata \
    --names 'crack,concrete_block,steel_column'
```

会按 8:2 分 train/val，生成 `data.yaml`。

**注意划分方式**：默认是随机按文件分。同一个会话的相邻帧会同时进训练和验证，
这样报出来的指标会偏乐观。要严谨就按**会话**分（不同房间、不同天的数据分开），
自己在脚本里改一下 `splits` 的划分即可。

## 4. 训练

控制机上的 ultralytics 装在独立 conda 环境里（系统 python 是 3.6，装不了）：

```bash
~/miniconda3/bin/python -c "from ultralytics import YOLO; \
  YOLO('yolo26n.pt').train(data='/tmp/mydata/data.yaml', epochs=100, imgsz=640, batch=8)"
```

- CPU 训练会很慢，100 轮可能要几小时。有 GPU 的机器上训完把 `best.pt` 拷回来即可。
- 数据集小的话先把 `epochs` 降到 50、`imgsz` 用 640 试试。
- 训练完的权重放在 `runs/detect/train/weights/best.pt`。

## 5. 接回语义地图

```bash
SEMANTIC_WEIGHTS=/path/to/best.pt BACKEND=yolo26 \
  bash semantic/run_semantic.sh bags/session_x
```

`SEMANTIC_WEIGHTS` 会覆盖默认的 `yolo26n.pt`。聚类那步用 `--classes` 指定新类别：

```bash
python3 semantic/cluster_objects.py <frames_dir> --classes 'crack,concrete_block,steel_column'
```

## 现在能直接开始的部分

`_local/dataset_raw/` 里已经有 200 张从 `session_20210826_222419` 抽出来的图，
可以直接拿来试标注流程。但这批是通用实验室场景，**不含你要的三类目标**——
真要训练，得先录一段专门拍裂缝/混凝土块/铁柱的数据。
