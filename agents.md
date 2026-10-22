# agents.md — 本项目的 Agent 行为约定

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
python -m pytest tests -q      # 自动测试
ruff check .                   # 代码风格
ruff format --check .          # 格式
git log --oneline -3           # 最近提交
```

## 6 项目现状（v0.2.0，HEAD 7313ad0，代码提交 3fd41e4）

- 已建成：`adapter`（与厂商软件打交道的唯一入口）、`capability`（探测对端具备哪些能力）、`config`（配置分层覆盖）、`control`（控制层）、`kinematics`（运动学与关节换算）
- 测试：737 条实跑通过，51 条跳过
- 跳过的原因：47+2 条需要可选的 `placo`（IK 数值用例），2 个 parity 文件需要 `lerobot` 做对照。**这不等于全部通过**，引用时必须说清
- 未搬入：`benchmarks/` 现场采集与测量脚本（计划 S4 与现场冒烟一起做）
- 仓库现状：未配置 remote，无 tag，3 条提交均在本地

## 7 待决事项

1. `placo` 是否声明为 `kinematics` extra 并装进开发环境——决定 49 条数值用例能否真跑
2. `benchmarks/` 迁移时机（S4）
3. 旧仓库那 49 例线上协议测试是否搬入（测的是厂商侧模块，本库不包含）

## 8 相关位置

- 决策记录：`docs/decisions.md`（D1–D6）
- 实测数据：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/`
- 旧仓库（只读参考）：`/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini-cartesian-teleop`
