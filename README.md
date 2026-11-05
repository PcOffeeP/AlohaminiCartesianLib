# AlohaMini Cartesian

AlohaMini 笛卡尔控制的产品侧库：对机器人运行时的适配协议、能力探测与单位换算裁决、配置覆盖，以及控制层与运动学。核心只依赖 numpy。

## 目标与平台

把 AlohaMini 2 Pro 从「遥操作主臂牵着走」改成「PC 上一个 Python 库直接按笛卡尔目标下发」，入口是 `connect` / `move_to` / `move_by`，底层仍走现有发送链、单位与硬件保护。

平台是厂商的 `lerobot_alohamini`：上游仓库 <https://github.com/liyiteng/lerobot_alohamini.git>（Pi 上部署的就是这份），本地 fork `/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini`。本库不含厂商代码，测试离线、不连设备，唯一接触面是 `adapter/protocol.py`。

形态变化、边界与已实现能力见 [docs/background.md](docs/background.md)。

## 关键决策

本库的关键取舍（依赖形态、配置格式、入库范围、numpy 与 `placo`、`lerobot` 版本区间、限位校验、能力拒绝形态等）逐条记在 [docs/ADR.md](docs/ADR.md)（D1–D19），每条给出背景、决定与代价。

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
| 某个设计为什么这么定、代价是什么（D1–D19） | [docs/ADR.md](docs/ADR.md) |
| 配置怎么覆盖、`deg_per_unit` 怎么算、公开 API 有哪些、代码约束与机器守卫 | [docs/library-guide.md](docs/library-guide.md) |
| 验收要满足哪些条目（V01–V13） | [docs/acceptance-baseline.md](docs/acceptance-baseline.md) |
| 现场到底取到了什么数字 | [docs/field-results.md](docs/field-results.md) |
| 到现场逐条怎么执行（命令、判据、退出码） | [docs/s4-field-smoke.md](docs/s4-field-smoke.md) |
| 每个版本改了什么、当时测试多少条（v0.2.0 → 最新） | [docs/changelog.md](docs/changelog.md) |
| 接下来要做什么 | [TODO.md](TODO.md) |
| Agent 在本仓库的行为约定、项目现状与已知限制 | [AGENTS.md](AGENTS.md) |

## 开发与测试

开发与测试约定（自检命令与两个必须注意的坑）见 [AGENTS.md](AGENTS.md) §5。
