# AlohaMini Cartesian

AlohaMini 笛卡尔控制的产品侧库：对机器人运行时的适配协议、能力探测与单位换算裁决、配置覆盖，以及控制层与运动学。核心只依赖 numpy。

## 目标与平台

把 AlohaMini 2 Pro 从「遥操作主臂牵着走」改成「PC 上一个 Python 库直接按笛卡尔目标下发」，入口是 `connect` / `move_to` / `move_by`，底层仍走现有发送链、单位与硬件保护。

平台是厂商的 `lerobot_alohamini`：上游仓库 <https://github.com/liyiteng/lerobot_alohamini.git>（Pi 上部署的就是这份），本地 fork `/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini`。本库不含厂商代码，测试离线、不连设备，唯一接触面是 `adapter/protocol.py`。

形态变化、边界与已实现能力见 [docs/background.md](docs/background.md)。

## 关键决策

本库的关键取舍（依赖形态、配置格式、入库范围、numpy 与 `placo`、`lerobot` 版本区间、限位校验、能力拒绝形态等）逐条记在 [docs/ADR.md](docs/ADR.md)（D1–D17），每条给出背景、决定与代价。

## 安装

核心只依赖 numpy：

```bash
pip install -e .
```

读 YAML 配置加 `yaml` extra：

```bash
pip install -e ".[yaml]"
```

跑数值 IK 用例与离线 IK 基准加 `kinematics` extra：

```bash
pip install -e ".[kinematics]"
```

该 extra 会把 numpy 带到 2.3.x，原因见 [docs/ADR.md](docs/ADR.md) D8。

接触真实运行时再加 `runtime` extra：

```bash
pip install -e ".[runtime]"
```

`lerobot` 当前不在 PyPI 上发布：用 `runtime` extra 之前，先按 `lerobot` 上游 README 把 `lerobot` 装进同一环境（例如从上游仓库以 editable 方式安装）。`pyproject.toml` 里的 `runtime = ["lerobot>=0.6.1,<0.7"]` 只限定支持的版本范围，不指向可解析的分发地址，区间依据见 [docs/ADR.md](docs/ADR.md) D16。

开发依赖在 PEP 735 的 `[dependency-groups] dev` 里（`pytest>=8.0`、`ruff>=0.6`、`PyYAML>=6.0`），不进包的元数据；`uv sync` 会自动装上，不用 uv 时按这三个名字手动装。

## 快速开始

完整可跑的最小脚本是 `examples/minimal_dry_run.py`：内存里的假 transport、假 IK、假时钟跑一次 dry-run 全链路，不连设备、不联网，结尾断言 `send_action` 调用 0 次。

```bash
uv run --no-sync python examples/minimal_dry_run.py
```

读配置：

```python
from pathlib import Path

from alohamini_cartesian.config.overlay import load_settings

settings = load_settings(template=Path("configs/template.yaml"), site=Path("configs/site.yaml"))
```

开会话并发一次相对位移（会话部分取自 `examples/minimal_dry_run.py`）：

以下片段是 `examples/minimal_dry_run.py` 的节选，`_HOST`、`transport`、`clock`、`sleeper`、`_FakeArmIK`、`_make_adapter()` 都在该文件里定义。

```python
from alohamini_cartesian.control.api import connect

session = connect(
    _HOST,
    dry_run=True,
    client_factory=lambda host, **_kwargs: transport,
    clock=clock,
    sleeper=sleeper,
    ik_factory=_FakeArmIK,
    adapter=_make_adapter(),
)
try:
    report = session.left.move_by(dx=0.004)  # 绝对目标改用 move_to(x, y, z)，单位米
finally:
    session.disconnect()
```

真机上把 `client_factory` 换成返回厂商 `AlohaMiniClient` 的工厂。未注入时 `connect` 抛 `RuntimeUnavailable`。配置覆盖规则、能力探测、完整公开 API 见 [docs/library-guide.md](docs/library-guide.md)。

## 文档导航

| 想知道什么 | 去哪看 |
| --- | --- |
| 为什么做这件事、形态怎么变、边界在哪、各模块已提供什么 | [docs/background.md](docs/background.md) |
| 某个设计为什么这么定、代价是什么（D1–D17） | [docs/ADR.md](docs/ADR.md) |
| 配置怎么覆盖、`deg_per_unit` 怎么算、公开 API 有哪些、代码约束与机器守卫 | [docs/library-guide.md](docs/library-guide.md) |
| 验收要满足哪些条目（V01–V13） | [docs/acceptance-baseline.md](docs/acceptance-baseline.md) |
| 现场到底取到了什么数字 | [docs/field-results.md](docs/field-results.md) |
| 到现场逐条怎么执行（命令、判据、退出码） | [docs/s4-field-smoke.md](docs/s4-field-smoke.md) |
| 每个版本改了什么、当时测试多少条（v0.2.0 → v0.7.0） | [docs/changelog.md](docs/changelog.md) |
| 接下来要做什么 | [TODO.md](TODO.md) |
| Agent 在本仓库的行为约定、项目现状与已知限制 | [AGENTS.md](AGENTS.md) |

## 开发与测试

```bash
uv run --no-sync ruff check .             # 代码风格
uv run --no-sync ruff format --check .    # 格式
uv run --no-sync python -m pytest tests   # 自动测试
```

两个坑：

- 必须带 `--no-sync`。本仓库没有 `uv.lock`，不带它的 `uv run` 会自行解析依赖并新写一个 `uv.lock`。
- pytest 不要再追加 `-q`。`addopts = "-q"` 已生效，再加会变成 `-qq` 并隐藏 `N passed, M skipped` 汇总行；要看汇总行用 `-o addopts=""`。

本包已以 editable 方式装进 `.venv`，`uv run --no-sync` 直接复用，不必先 `pip install -e .`。当前 `pytest tests` 报 `1275 passed, 2 skipped`；跳过项是两个 parity 文件缺 `lerobot`，构成见 [AGENTS.md](AGENTS.md) 第 6 节的已知限制。测试不依赖真实设备，不联网。
