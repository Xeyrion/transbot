# 裂缝图片与分割标注

裂缝细长、分叉，建议先做二分类语义分割：每个像素属于 `crack` 或背景。
标注可见的实际裂缝区域，不用矩形框代替，也不要只画没有宽度的中心线。
这与未来把裂缝投到地图上的定位步骤分开；标图本身不依赖地图或相机外参。

## 从 bag 抽取原图

在控制机运行，输出目录必须尚不存在。同一会话的分卷自动合并：

```bash
cd ~/robot_research/bagtools
. /opt/ros/melodic/setup.bash
python2 semantic/extract_images.py bags/structure_lab_11_11 \
  datasets/cracks/structure_lab_11_11/images \
  --fps 1 --max 1000 --diff 0 --format png --time-source record
```

- PNG 保留解码后的原始 RGB 像素，不缩放、不增强。不要用缩略图标注。
- `--diff 0` 关闭整体画面差分去重，避免遗漏局部细裂缝。
- 此会话存在传感器 Header 时间回退，因此按 bag 记录时间抽样。
  这只用于独立图片导出，不修复 RGB-D 配对、TF 或建图时间关系。
  记录时间也回退时，开启新的抽样段，保留两侧图片和原始时间戳，不伪造连续时间。
- `images/images.json` 记录原包路径、帧号、两种时间戳、分辨率和导出状态。
  Header 模式仍拒绝时间回退；失败会记录原因，部分结果不能当作完整导出。
- 每秒一张是初选，快速移动或短暂出现的目标可能漏掉。可提高 `--fps` 导出到新目录。

## 标注工具与目录

推荐本地 [Labelme](https://github.com/wkentaro/labelme)，支持多边形轮廓及分割数据导出。
复杂分叉也可用 [CVAT](https://docs.cvat.ai/docs/manual/advanced/annotation-with-brush-tool/) 的掩膜工具。
先人工试标少量清晰图片，确定规则，再批量处理。不要直接接受自动辅助生成的轮廓。

```text
datasets/cracks/<会话>/
  images/            原始分辨率 PNG、images.json
  annotations/       Labelme JSON；未标注时为空
  labels.txt         crack
  README.md          本批次说明
```

装好 Labelme 后可指定图片和输出目录，避免 JSON 散落在原图中：

```bash
labelme datasets/cracks/structure_lab_11_11/images \
  --output datasets/cracks/structure_lab_11_11/annotations \
  --labels datasets/cracks/structure_lab_11_11/labels.txt --nodata
```

Labelme 中放大图片，沿裂缝两侧画闭合多边形，类别统一为 `crack`。
二分类语义分割不必给每条分支不同的实例编号。多边形可在训练前转换为与原图同尺寸的
单通道掩膜：0=背景，1=裂缝；保留原始 JSON，不用截图充当掩膜。

## 标注规则

- 只标能够辨认的裂缝，保持可见宽度；接缝、构件外缘、阴影、划痕不要混入。
- 放大后仍分不清的图片，暂列为待复核或排除，不能当作无裂缝负样本。
- 真正的负样本需要逐张检查确认。没有 JSON 只代表未标注，不代表没有裂缝。
- 不要把同一裂缝的相邻帧随机分进训练集和验证集。以后按不同采集、场景或实体试件分组，
  并保证同一实体裂缝不会跨集合；现在这一个会话先用于试标，不宣称独立验证效果。

分割标注不等于现有检测流水线可以直接训练。`make_yolo_dataset.py`、现有框检测投影流程
服务于检测框；不要拿它转换这些分割标签。后续再按目标模型整理像素掩膜或实例轮廓，
并单独验证裂缝如何利用深度和位姿投影到地图。
