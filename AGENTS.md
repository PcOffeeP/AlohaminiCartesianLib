# AGENTS.md — 本项目的 Agent 行为约定

任何 Agent 在这个项目里工作或汇报，都遵守本文件。它是约束，不是建议。

## 1 这个项目是什么

一句话：一个独立的 Python 库，把「末端要往哪走」翻译成「六个关节各转多少度」，再把指令发给机器人。

机器人在跑厂商（vendor）的软件，我们不修改它；本库只跑在 PC 一侧，通过协议与机器人的常驻主程序（Host 进程）通信。

## 2 回答方式（硬要求）

1. **结论先行。** 第一段就说做完了什么、成功还是失败。
2. **术语保留，附一句人话。** 专业名词用原文，第一次出现时补一句白话解释。例如：`Gate F 是运行时门禁（决定这次测量能不能发命令的检查）`。不要只讲白话，也不要只讲术语。
3. **能用表格就用表格。** 对照、清单、结果，一律表格化。
4. **每次汇报必须含三样**：做了什么、证据是什么、下一步是什么。
5. **不写修饰词。** 「大概」「应该」「可能」不用于描述已完成的事实。不确定就说不确定。

## 3 证据纪律

- 说「通过」必须带数字：多少例、多少字节、哪个提交哈希、哪条命令的原文输出。
- **未验证的事必须标注**，不得混进已验证的结论里。
- 缺失的字段存为空（`None`），**不得填 0 或 4096 之类看起来正常的值**。
- 引用文件时给完整路径或仓库相对路径 + 行号；行号不确定就写「约」。

## 4 工程纪律

- **只有 `adapter/protocol.py` 可以引用 `lerobot`**。`src/` 内其余文件禁止；有 AST 测试守着。
- 测试必须离线：不连设备、不联网、不依赖真机。
- 不引入 `torch`、`scipy` 等重依赖。`placo` 属可选（IK 数值用例需要它）。
- 不修改厂商仓库 `/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini-cartesian-teleop`。
- 不自动推送远程、不改写历史（不 amend、不 force push）。
- 提交只用显式路径，禁止 `git add -A`。

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
- 本包已以 editable 方式装进 `.venv`，`uv run --no-sync` 直接复用，不必先 `pip install -e .`。

## 6 项目现状（v0.4.0）

- 已建成：`adapter`（与厂商软件打交道的唯一入口）、`capability`（探测对端具备哪些能力）、`config`（配置分层覆盖）、`control`（控制层）、`kinematics`（运动学与关节换算）、`benchmarks`（离线 IK 基准 `benchmarks/alohamini_ik/benchmark_arm_ik.py`，现场采集与测量脚本 `benchmarks/alohamini_io/capture_robot_fingerprint.py` 与 `benchmarks/alohamini_io/measure_latency.py`）
- 测试：999 条实跑通过，2 条跳过
- 跳过的原因：两个 parity 文件需要 `lerobot` 做对照（`tests/test_control_api_lerobot_parity.py`、`tests/test_kinematics_lerobot_parity.py`）。**这不等于全部通过**，引用时必须说清
- 未完成：现场 smoke（S4）与配置样例。`benchmarks/alohamini_io` 两脚本与其配套测试已在 v0.4.0 迁入并有离线测试，但从未对着真机 Host 跑过，见 `docs/s4-field-smoke.md`
- 仓库现状：`v0.2.0`、`v0.3.0`、`v0.4.0` 三个 tag 均为 annotated（`git cat-file -t` 报 `tag`）。`v0.2.0` 指向 `3fd41e4`（`feat: 迁移控制层与运动学到独立库（v0.2.0）`）；`v0.3.0` 指向 `4281f2b`（`release: v0.3.0 版本号与文档同步`）；`v0.4.0` 指向本次文档提交（代码提交 `d23be22`）；未配置 remote，全部提交只在本地

## 7 待决事项

1. `placo` 依赖形态——已决：声明 `kinematics` extra（`placo>=0.10.1,<0.11`）并装进开发环境，接受 numpy 落到 2.3.x，见 `docs/ADR.md` D8
2. `benchmarks/` 迁移时机——已迁完（`benchmarks/alohamini_ik` 在 v0.3.0，`benchmarks/alohamini_io` 在 v0.4.0），剩现场验证：真机 metadata 取值、Mode A/B 测量、Gate F 拒绝路径、配置样例采集。见 `docs/ADR.md` D3 与 D9、`docs/s4-field-smoke.md`
3. 旧仓库那 49 例线上协议测试是否搬入（测的是厂商侧模块，本库不包含）——已决：不迁，见 D7
4. 限位表倒置（`lo > hi`）没有校验也没有用例守护，`solve()` 入口不检查

## 8 相关位置

- 决策记录：`docs/ADR.md`（D1–D9）
- 现场实施：`docs/s4-field-smoke.md`（S4 到现场的逐条命令、判据与产物）
- 迁移归档：`docs/archive/migration-backlog.md`（迁移已在 v0.4.0 完成）
- 实测数据：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/`
- 旧仓库（只读参考）：`/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini-cartesian-teleop`
