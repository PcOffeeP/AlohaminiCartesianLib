# 迁移待办（已归档）

已归档，迁移在 v0.4.0 完成；剩余工作全部转入 [S4 现场实施：真机 smoke](../s4-field-smoke.md)。本文件现位于 `docs/archive/migration-backlog.md`。

## 本文件记录了什么

| 项 | 收口结论 |
| --- | --- |
| `benchmarks/alohamini_ik` | 在 v0.3.0 迁入，离线可跑 |
| `benchmarks/alohamini_io` | 在 v0.4.0 迁入（`capture_robot_fingerprint.py`、`measure_latency.py` 与两份配套测试），语义零改动，`lerobot` 保持函数体内惰性导入 |
| 测试 | `pytest tests` 报 `999 passed, 2 skipped`；跳过的是两个 parity 文件，缺 `lerobot` 做对照，不等于全部通过 |
| 厂商侧协议测试 | 按 `docs/ADR.md` D7 不迁，49 个测试函数留在旧仓库 |

## 不再在本文件跟踪的事

| 事 | 现在去哪 |
| --- | --- |
| 现场 smoke（S4）：真机 metadata 取值、Mode A/B 测量、代码 Gate F 拒绝路径 | [S4 现场实施：真机 smoke](../s4-field-smoke.md) |
| 配置样例：D2 的「模板文件 → 现场文件」两份 YAML | [S4 现场实施：真机 smoke](../s4-field-smoke.md) 第 7 节 |

ADR 决策依据：`docs/ADR.md` 的 D3（迁移范围与分阶段）与 D9（`benchmarks/alohamini_io` 迁移时机）是这次迁移的决策记录。
