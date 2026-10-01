# legacy-missions · 历年练兵工程归档

> **来源**：自 `rdk-x5-flight-system`（`03_RDK-X5机载飞控系统`，2026-10-01 并入本仓后退役）迁移的历史赛题练兵工程。
> **定位**：仅作历年方案溯源与代码参考，**不随现役系统维护**——不要把这些目录的代码当作现行结构使用。

| 目录 | 对应赛题 | 说明 |
| --- | --- | --- |
| `circle_pole/` | 绕杆/立柱类任务 | 含雷达立柱追踪练兵实现 |
| `fire_patrol/` | 2023-G 空地协同消防 | 火源定位与处置任务工程 |
| `warehouse_inventory/` | 2024-D 立体货架盘点 | 货架盘点任务工程 |
| `plant_protection_2021/` | 2021-G 植保飞行器 | 植保作业任务工程 |
| `competition_2026/` | 2026-D 早期版本 | 已被 `drone-system/companion-computer/competition_2026_d/` 取代 |
| `original/` | 最初基线工程 | 项目起点的 main.py / 调试脚本 |

**注意**：这些目录各自内嵌了当时的 `Lcode/` 等依赖副本，**路径与导入关系按原仓结构冻结**；现役开发请一律回到 `drone-system/companion-computer/`。机载 2026-D 现役代码与这些历史版本的演进对照，可通过原仓 Git 历史溯源。
