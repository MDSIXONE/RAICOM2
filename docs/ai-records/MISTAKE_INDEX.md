# 犯错记录索引

错误条目的完整内容在 `mistakes/YYYY-MM-DD.md`，本文件只做索引。读取/写入时机由 `project-memory-records` 技能定义。

## 主题索引

- 前后端配置字段同步：2026-09-27.md
- PID 缩放移植：2026-09-27.md
- PowerShell/SSH 跨 Shell 引号：2026-09-27.md、2026-10-13.md
- DOM ID 唯一性与控制台快捷键：2026-10-08.md
- Playwright range 数值格式：2026-10-13.md
- 底盘物理死区与最终指令限制：2026-10-13.md
- 雷达墙面拟合与偏航测量耦合：2026-10-13.md

## 按日期

### 2026-10-13

- 雷达墙面连续性检查把遮挡缺口当成整面无效

- PowerShell 使用反斜杠转义双引号导致 SSH/Python 命令截断
- Playwright range fill 使用带尾零数值触发 Malformed value
- 墙控非零 VX/yaw 低于物理起动门槛导致减速区停滞
- 雷达扇区距离受偏航与测点切换影响，误判平移死区

### 2026-10-08

- 巡线状态和参数使用重复 DOM ID lineLost

### 2026-09-27

- 巡线参数 UI 只暴露部分配置字段
- 巡线滑块只点击保存且与预览分离
- 巡线 PID 缩放遗漏导致持续右转
- PowerShell 向 SSH 内嵌 Python 正则时发生本地语法解析
