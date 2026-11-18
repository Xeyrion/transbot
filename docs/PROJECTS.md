# 项目与数据导航

项目清单和公开入口维护在 [projects.json](projects.json)，由检查工具生成[首页项目表](../README.md#项目入口)。跨项目的数据流、运行分工和文档维护规则统一见[整体架构](ARCHITECTURE.md)。

根目录只维护项目入口、总览及仓库约定。具体算法和操作说明放回对应子项目。源码历史统一由 Transbot Git 仓库管理。

## 本机资料

`_local/` 为已有诊断材料、辅助工具、临时文件与历史仓库备份，保持忽略。隐藏的 `.workbuddy-ai/`、`.slidep/` 属于本机工具状态，不参与发布。

Windows 旧 `bagtools/` 中的数据集、结果和归档保持原位置；控制机原存储 `~/robot_research/bagtools/` 中的 bag、模型、图片、结果和归档同样保持原位置。它们不随源码迁移，也不进入 GitHub。控制机 `~/transbot/bagfiles/` 等既有数据目录也不移动。

新源码 `semantic_mapping/paths.local.json` 配置 `data_dir` 与 `models_dir`，不提交。可从 `paths.example.json` 复制后编辑。每个录制的当前地图和语义结果继续保存在该录制目录内。

## 版本与发布范围

源码、文档、配置示例及测试可以提交。原始 `.bag`、活动录制尾文件、模型权重、批量图像/数组、生成结果与本机配置不提交。阶段演示完成不等于定位精度已经验证；研究边界以各项目的阶段说明为准。

发布前运行 `python3 tools/check_repository.py` 检查 Git 收录范围，自动检查与离线测试配置在 `.github/workflows/repository.yml`。实机验证与离线源码检查分别记录，不能互相替代。
