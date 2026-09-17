# RAICOM2 睿抗机器人大赛智能配送项目 结构索引

- 更新时间：2026-09-17

## 总体结构
```
RAICOM2/
├── .agents/skills/            # 随仓库同步的 AI 技能（工作记录、提交规范、黑话词典、本索引）
├── .github/workflows/         # CI：update-project-tree.yml 自动更新 README 结构树
├── archive/                   # 设备源码快照与历史归档（只读参考）
│   └── preliminary-code/      # 初赛代码等历史归档
├── blender-maps/              # 离线场地：Blender 可再生成源 + ROS 地图
│   ├── offline_navigation_arena.py   # 生成 .blend 场地的脚本
│   └── ricam_arena/           # blender/、gazebo/、navigation/（pgm+yaml 地图）
├── competition-rules/         # 赛事规则 PDF 与摘要
├── datasets/                  # YOLO/OCR 数据集说明与小型样例（大数据默认不入库）
│   ├── ocr/
│   └── yolo/abcd/
├── docs/                      # 跨对话记录与技术文档
│   ├── ai-records/            # AI 工作记录（CHANGE_LOG、FAILED_APPROACHES、MISTAKE_INDEX、mistakes/）
│   ├── lingo.md               # 项目黑话词典词条正文
│   └── technical/             # 技术文档
├── robot-information/         # 机器狗平台资料与硬件参考
│   ├── camera/ compute/ control/ course-resources/
│   └── private/               # 本机敏感资料（不入库）
├── robot-src/                 # ★ 机器端全部源码（唯一机器代码写入区）
│   ├── catkin_ws/src/         # 机器狗 ROS 包（Catkin 工作空间，容器内编译运行）
│   └── host-services/<device>/ # 机器端非 ROS 常驻服务
├── scripts/                   # 仓库辅助脚本（update_readme_tree.py）
├── technical-reports/         # 技术报告、阶段总结、赛后复盘
├── tmp/                       # 临时文件与实验脚本（不入库）
└── wsl-simulation/            # ★ WSL 离线仿真（独立工作区，不进 catkin_ws）
    └── src/                   # 仿真 ROS 包
```

## 功能索引

| 功能/模块 | 位置 | 一句话说明 | 详情 |
| --- | --- | --- | --- |
| 赛事规则 | `competition-rules/` | 睿抗国赛物流配送足式规则 PDF 与摘要 | - |
| 机器狗源码 | `robot-src/catkin_ws/src/` | 机器端 ROS 功能包（本次比赛任务代码存放地） | - |
| 机器端非 ROS 服务 | `robot-src/host-services/<device>/` | 机器端常驻服务源码 | - |
| WSL 离线仿真 | `wsl-simulation/src/` | 离线导航/感知仿真 ROS 包 | - |
| 离线场地地图 | `blender-maps/ricam_arena/navigation/` | 栅格地图（pgm/yaml/json/png） | - |
| Blender 生成脚本 | `blender-maps/offline_navigation_arena.py` | 可再生成与栅格地图对应的 .blend 场地 | - |
| YOLO/OCR 数据集 | `datasets/` | 数据集说明与小型样例（大数据不入库） | - |
| AI 工作记录 | `docs/ai-records/` | 变更日志、犯错索引、失败方案记录 | - |
| 项目黑话词典 | `docs/lingo.md` | 口语化指令的精确展开定义 | - |
| 硬件资料 | `robot-information/` | 上位机/摄像头/下位机资料、实机观测、下载清单 | - |
| 历史归档 | `archive/` | 设备源码快照与历史归档（只读） | - |
| 临时实验 | `tmp/` | 实验脚本、测试数据（不入库） | - |

## 关键入口

- 项目说明：`README.md`（含目录职责边界：机器代码只允许写入 `robot-src/catkin_ws/src/` 或 `robot-src/host-services/<device>/`，仿真代码写入 `wsl-simulation/src/`）
- CI 结构树自动更新：`.github/workflows/update-project-tree.yml`（配合 `scripts/update_readme_tree.py`）
