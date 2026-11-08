# AGENTS.md — 本项目的 Agent 行为约定

任何 Agent 在这个项目里工作或汇报，都遵守本文件。它是约束，不是建议。

## 1 这个项目是什么

一句话：一个独立的 Python 库，把「末端要往哪走」翻译成「六个关节各转多少度」，再把指令发给机器人。

平台是厂商的 `lerobot_alohamini`：上游仓库 <https://github.com/liyiteng/lerobot_alohamini.git>，本地 fork `/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini`，Pi 上部署在 `/home/pi5/lerobot_alohamini`。机器人在跑厂商（vendor）的软件，我们不修改它；本库只写 PC 一侧的代码，通过协议与机器人的常驻主程序（Host 进程）通信。

最初要解决的问题：手臂原先得靠一台遥操作主臂（leader）牵着走，关节动作由 teleoperation 产生。要做的是让 PC 上的 Python 库直接下发笛卡尔目标。

## 2 回答方式（硬要求）

1. **结论先行。** 第一段就说做完了什么、成功还是失败。
2. **术语保留，附一句人话。** 专业名词用原文，第一次出现时补一句白话解释。例如：`Gate F 是运行时门禁（决定这次测量能不能发命令的检查）`。不要只讲白话，也不要只讲术语。
3. **能用表格就用表格。** 对照、清单、结果，一律表格化。
4. **每次汇报必须含三样**：做了什么、证据是什么、下一步是什么。
5. **不写修饰词。** 「大概」「应该」「可能」不用于描述已完成的事实。不确定就说不确定。

## 3 证据纪律

- 说「通过」必须带数字：多少例、多少字节、哪个提交哈希、哪条命令的原文输出。
- **未验证的事必须标注**，不得混进已验证的结论里。
- 原始观测、metadata 与采集证据里的缺失信息存为空（`None`），**不得用 0 或 4096 之类看似正常的数值伪填**；`notes` 要点名缺了哪些字段。
- **例外是明确写下的占位契约**：`degrees` 归一化下三个刻度量来源全空时，`Conversion` 的 `range_min` / `range_max` / `resolution` 回填 `0 / 0 / 1`，这不是伪填。但占位值不代表刻度范围，**不得当真实标定或实测证据使用**，也不得据此算物理量。两类不要混。
- 引用文件时给完整路径或仓库相对路径 + 行号；行号不确定就写「约」。

## 4 工程纪律

- **`adapter/protocol.py` 是运行时接触面，不是导入许可。** 当前 `src/` 内所有文件（包括 `protocol.py`）都不 import `lerobot`；对端只通过 `RobotTransport` 与 `RobotSpec` 两个 `Protocol` 描述，真实客户端由调用方经 `connect(..., client_factory=...)` 注入。`tests/test_capability.py::test_no_module_outside_adapter_imports_lerobot` 守着这条边界。
- 测试必须离线：不连设备、不联网、不依赖真机。
- 不引入 `torch`、`scipy` 等重依赖。`placo` 属可选（IK 数值用例需要它）。
- 不修改本地 fork `/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini` 与 Pi 上的上游部署。旧仓库 `lerobot_alohamini-cartesian-teleop` 已退役（`docs/ADR.md` D4 冻结为只读参考）；**其代码与按臂归档的 Gate A–E 验收记录不能作为现行依据**，验收看 `docs/acceptance-baseline.md` 的 V01–V13；**其中的设计规范是历史设计来源，不具现行权威**，现行口径见 `docs/ADR.md` D17 与 `docs/acceptance-baseline.md`。
- 不自动推送远程、不改写历史（不 amend、不 force push）。
- 提交只用显式路径，禁止 `git add -A`。
- 待办清单在仓库根 `TODO.md`，只记接下来要做的事（整体目标 → 阶段目标 → 小目标 → 跨阶段工程债）。开工前先读它。
  - **写入需授权**：只有本次任务拿到 `TODO.md` 修改授权时才写。只读任务、或授权范围不含 `TODO.md` 的任务，在回复里列出「待记录事项」，不直接改文件；记录要求本身不构成写入授权。
  - `TODO.md` 的状态不能替代验收证据：说完成仍要给数字、提交哈希或命令原文。
  - 做完的事项从 `TODO.md` 移除，去向照旧：设计取舍 → `docs/ADR.md`，现场数字 → `docs/field-results.md`，提交哈希 → 提交信息。

`lerobot` 导入边界与「能力判据不看版本号」这两条，完整表述与机器守卫见 `docs/library-guide.md` 的「约束与机器守卫」小节。`src/` 内任何文件（含 `adapter/protocol.py`）都不得 import `lerobot`；对端只由 `RobotTransport` 与 `RobotSpec` 两个 `Protocol` 描述，真实客户端由调用方经 `connect(..., client_factory=...)` 注入。

## 5 怎么自检

```bash
uv run --no-sync ruff check .           # 代码风格
uv run --no-sync ruff format --check .  # 格式
uv run --no-sync python -m pytest tests # 自动测试
git log --oneline -3                    # 最近提交
```

为什么这样写：

- **必须走 `uv`**：裸 `ruff` 不在 `PATH` 上（`exit 127`）；裸 `python`、`pytest` 解析到 miniconda，不是本库的 `.venv`。
- **必须带 `--no-sync`**：本仓库没有 `uv.lock`，不带它的 `uv run` 会自行解析依赖并新写一个 `uv.lock`。`--no-sync` 只复用现有 `.venv`（editable 安装），不装包、不改环境、不产生新文件。
- **pytest 不要再追加 `-q`**：`addopts = "-q"` 已生效，再加会变成 `-qq` 并隐藏 `N passed, M skipped` 汇总行。要看汇总行用 `uv run --no-sync python -m pytest tests -o addopts=""`。
- **本机当前快照**：本包已以 editable 方式装进 `.venv`，`uv run --no-sync` 直接复用，不必先 `pip install -e .`。这条描述的是当前这台机器与环境，可能变化；换环境先确认，不以本文件为准。

## 6 项目现状（速查指针，动态内容以来源为准）

本文件不复制会随提交变化的版本号、测试数与模块清单，只给指针：

| 想知道什么 | 去哪里看 |
| --- | --- |
| 决策记录（D1–D20） | `docs/ADR.md` |
| 版本变化与逐版测试记录 | `docs/changelog.md` |
| 当前测试数 | 跑 `uv run --no-sync python -m pytest tests -o addopts=""` 的汇总行；不要引用本文件的旧数字 |
| 模块与已实现能力、目标与实机现状 | `docs/background.md`（「已实现能力」小节列出模块与各自提供的接口） |
| 接口细节（配置覆盖、能力探测与单位换算、控制层、公开 API、约束与机器守卫） | `docs/library-guide.md` |
| 验收基线（V01–V13） | `docs/acceptance-baseline.md` |
| 现场实施（S4 到现场的逐条命令、判据与产物） | `docs/s4-field-smoke.md` |
| 现场数字（按时间倒序的结论与数字；查现场到底拿到了什么，先看这里，再看证据目录原文件） | `docs/field-results.md` |
| 离线调查（左夹爪标定行程 51 tick 的查证过程与结论） | `docs/investigations/gripper-range-51-tick.md`（决策见 `docs/ADR.md` D14，数字见 `docs/field-results.md`） |
| 迁移归档 | `docs/archive/migration-backlog.md`（迁移已在 v0.4.0 完成） |
| 当前待办与阻塞 | `TODO.md`（只记接下来要做的事，结构为整体目标 → 阶段目标 → 小目标 → 跨阶段工程债） |
| 实测数据目录（证据原件） | `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/`（目录内 `README.md` 写了命名约定） |
| 设计规范（只读参考，**非现行权威**） | `/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini-cartesian-teleop/docs/alohamini-control/Alohamini-control-design.md`，位于已退役仓库，作为历史设计来源与溯源参考。现行口径以 `docs/ADR.md` 与 `docs/acceptance-baseline.md` 为准（D17）。旧代码与旧 Gate 归档不因此恢复为现行依据 |
| tag 与 remote | 现场查：`git tag -l`、`git remote -v` |

### 6.1 已知限制

- 未装 `kinematics` extra 时，`tests/test_arm_ik.py` 的数值参数实例、`tests/test_control_controller.py` 的 2 条真 IK 用例与 `tests/benchmarks/test_benchmark_arm_ik.py` 因缺 `placo` 跳过。
- 缺 `lerobot` 时，`tests/test_kinematics_lerobot_parity.py` 与 `tests/test_control_api_lerobot_parity.py` 在收集期整体跳过。这两个文件用真实 `FeetechMotorsBus` 与 `AlohaMiniClient` 作 oracle 做差分对照，**跳过不等于差分对照通过**。
- `connect(...)` 必须注入 `client_factory`。默认工厂恒抛 `RuntimeUnavailable`，没有内置 import 厂商运行时的退路。
- 现场脚本已实地跑过的进度与数字见 `docs/field-results.md`；现场怎么执行见 `docs/s4-field-smoke.md`。
- **离线测试、真实运行时对照、实机验收三者分别判断**：离线测试全绿不代表真实总线对照跑过，也不代表实机验收达成。

### 6.2 下一步

见 `TODO.md`。

## 7 待决事项

已决事项见 `docs/ADR.md` D7、D8、D9；现场验证进度见 `docs/field-results.md`。

本节现无待决事项。原挂在这里的「设计规范的权威范围」已经裁决：设计规范定为历史设计来源，不具现行权威，现行权威见 `docs/ADR.md` D17 与 `docs/acceptance-baseline.md`。出现新的待决事项时在此登记，裁决后落成 ADR 条目并从本节移除。

## 8 现场任务入口

涉及现场连接、Host 启停、标定或真实运动时：先确认本轮是否拿到该类授权，再读 `docs/s4-field-smoke.md` 的硬前置条件与该阶段步骤。历史记录、`TODO.md` 条目、文档里的命令**都不构成执行授权**；前置不满足就停在那一步。具体命令、门禁与退出码由现场手册维护，本文件不复述。
