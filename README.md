# AlohaMini Cartesian

AlohaMini 笛卡尔控制的产品侧库：对机器人运行时的适配协议、能力探测与单位换算裁决、配置覆盖，以及控制层与运动学。核心只依赖 numpy。

## 目标与平台

把 AlohaMini 2 Pro 从「遥操作主臂牵着走」改成「PC 上一个 Python 库直接按笛卡尔目标下发」，入口是 `connect` / `move_to` / `move_by`，底层仍走现有发送链、单位与硬件保护。

平台是厂商的 `lerobot_alohamini`：上游仓库 <https://github.com/liyiteng/lerobot_alohamini.git>（Pi 上部署的就是这份），本地 fork `/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini`。本库不含厂商代码，测试离线、不连设备，唯一接触面是 `adapter/protocol.py`。

形态变化、边界与已实现能力见 [docs/background.md](docs/background.md)。

## 关键决策

本库的关键取舍（依赖形态、配置格式、入库范围、numpy 与 `placo`、`lerobot` 版本区间、限位校验、能力拒绝形态等）逐条记在 [docs/ADR.md](docs/ADR.md)（D1–D20），每条给出背景、决定与代价。

## 当前状态

版本 `v0.7.0`：`pyproject.toml` 的 `version` 与 `src/alohamini_cartesian/__init__.py` 的 `__version__` 一致。

本轮 `uv run --no-sync python -m pytest tests -o addopts=""` 报 `1496 passed, 2 skipped`。`2 skipped` 是两个 parity 文件缺 `lerobot` 整体跳过；`2` 不等于全部通过。

验收矩阵现状（与 [docs/acceptance-baseline.md](docs/acceptance-baseline.md) §2.3 一致）：

| 维度 | 有证据 | 部分 | 不适用 | 未取 |
| --- | --- | --- | --- | --- |
| 软件侧 | 10 条 | 3 条（V01、V06、V11） | 0 条 | 0 条 |
| 实机侧 | 2 条（V01、V12） | 1 条（V10） | 1 条（V09） | 9 条 |

- **V01 已通过**：判据按 [docs/ADR.md](docs/ADR.md) D20——达成率 `≥50%`，分母 `abs(--delta-deg) × --steps`（`--anchor present` 且逐步实际命令增量与 `--delta-deg` 不等时改用 `Σ(cmd_deg − baseline_deg)`，并在记录里写明用的是哪个分母）；步末残余按关节类别分级（肩类 `≤2°`，其余轴 `≤2 tick`）。2026-11-08 `--anchor command` 两轮双侧过门槛，达成率 `77.3626%` / `79.1209%` / `82.6374%`（轮一两侧同为 `77.3626%`；轮二左 `79.1209%`、右 `82.6374%`）。
- **V12 已核对**：离线、盘上真机数据，`tools/verify_unit_consistency.py` 18/18 电机两侧 `range_min`/`range_max` 完全相等，差异 0 个、退出码 `0`；限定「只证明两侧单位口径一致，不含 URDF 限位交集」。
- **V10 是部分**：Gate F 第一段的真实拒绝记录已取（v1 宿主，退出码 `6`、零命令）；v2 宿主下第二段未触发（18/18 电机字段齐全，结构上不会拒）。

肩部现状：A 阶段自 2026-11-08 起不再受阻，V01 已按 D20 收口为「通过」。限定一：右臂只有 `arm_right_shoulder_pan` 一轴的跟随记录（两轮 `77.3626%`（STAMP `20261108T125734Z`）/ `82.6374%`（STAMP `20261108T141936Z`）），右臂其余关节未测。限定二：V02「动一侧时另一侧不串值」的实机证据仍未取。

细节见 [docs/acceptance-baseline.md](docs/acceptance-baseline.md)（V01–V13）与 [docs/field-results.md](docs/field-results.md)。

## 安装

`pip install -e .` 即可，核心只依赖 numpy。
optional extras 有三个：`yaml`（读 YAML 配置）、`kinematics`（数值 IK 用例与离线 IK 基准）、`runtime`（接触真实运行时）；用 `runtime` 之前，先按 vendor fork（`lerobot_alohamini`）的方式把 `lerobot` 装进同一环境。
细节见 [docs/library-guide.md](docs/library-guide.md) 与 [docs/ADR.md](docs/ADR.md)（D1 / D8 / D16）。

## 快速开始

完整可跑的最小脚本是 `examples/minimal_dry_run.py`（`uv run --no-sync python examples/minimal_dry_run.py`）：内存里的假对象跑一次 dry-run 全链路，不连设备、不联网，结尾按实测计数断言 `send_action` 调用 0 次。

```python
from alohamini_cartesian.control.api import connect

session = connect(host, dry_run=True, client_factory=make_client)
try:
    report = session.left.move_by(dx=0.004)  # 绝对目标改用 move_to(x, y, z)，单位米
finally:
    session.disconnect()
# 完整 API 见 docs/library-guide.md
```

`dry_run` 默认 `True`；真机运行时把 `client_factory` 换成返回厂商 `AlohaMiniClient` 的工厂；未注入 `client_factory` 时 `connect` 抛 `RuntimeUnavailable`。要让运动命令真正跑起来还要注入 kinematic adapter 与 IK 工厂（缺 adapter 时命令抛 `MotionNotEnabled`），见 [docs/library-guide.md](docs/library-guide.md) §3。

## 文档导航

| 想知道什么 | 去哪看 |
| --- | --- |
| 为什么做这件事、形态怎么变、边界在哪、各模块已提供什么 | [docs/background.md](docs/background.md) |
| 某个设计为什么这么定、代价是什么（D1–D20） | [docs/ADR.md](docs/ADR.md) |
| 配置怎么覆盖、`deg_per_unit` 怎么算、公开 API 有哪些、代码约束与机器守卫 | [docs/library-guide.md](docs/library-guide.md) |
| 验收要满足哪些条目（V01–V13） | [docs/acceptance-baseline.md](docs/acceptance-baseline.md) |
| 现场到底取到了什么数字 | [docs/field-results.md](docs/field-results.md) |
| 到现场逐条怎么执行（命令、判据、退出码） | [docs/field-ops/00-opening.md](docs/field-ops/00-opening.md) |
| 每个版本改了什么、当时测试多少条（v0.2.0 → 最新） | [docs/changelog.md](docs/changelog.md) |
| 接下来要做什么 | [TODO.md](TODO.md) |
| Agent 在本仓库的行为约定、项目现状与已知限制 | [AGENTS.md](AGENTS.md) |

## 开发与测试

开发与测试约定（自检命令与两个必须注意的坑）见 [AGENTS.md](AGENTS.md) §5。
