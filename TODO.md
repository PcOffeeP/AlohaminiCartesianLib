# 待办

本文件是本仓库唯一的待办清单。开工前先读，完成一项改状态并写提交哈希，发现新缺口立即追加。不要在对话里记待办。

更新约定：状态只有 `待办` / `进行中` / `待定` / `已完成` 四种；每条写清「要做什么、判据是什么」，不写「大概」「可能」。

## 1 代码

| 状态 | 事项 | 判据 | 备注 |
| --- | --- | --- | --- |
| 已完成 | `src/alohamini_cartesian/control/arm_ik.py` 的 `solve()` 增加限位校验：有限性、`lo > hi`、`lo == hi`，违规抛 `ValueError` | 由 `T-IK-18.x` 系列用例守护 | 与 `joint_adapter.py:262` 的 `Inverted URDF limit` 做法对齐；对外契约变更，需追加 ADR D10；已完成，提交 57d4e6d |
| 已完成 | 多余关节键的处理定为「忽略」，并加用例钉住 | 由 `T-IK-18.x` 系列用例守护 | 现状已是忽略，缺的是契约与用例；已完成，提交 57d4e6d |
| 已完成 | `memoryview` 绕过限位形态判据、大整数在 `np.deg2rad` 抛裸 `TypeError` 两个洞已修 | 由 `T-IK-18.14` / `T-IK-18.15` 守护 | 本轮修复 |
| 已完成 | `JOINT_LIMIT` 失败分类误标：残差超标的失败被标成 `JOINT_LIMIT`（`arm_ik.py:468-476`） | 分类矩阵用例（在限位内/外 × 残差达标/超标）逐格通过 | 先定「两者同时成立」时 `JOINT_LIMIT` 优先，再改实现；已完成，提交 16e6ef9 |
| 已完成 | `tests/benchmarks/test_benchmark_arm_ik.py` 在机器满载时偶发 1–5 条红（`14_x` / `15_x` 系列），与业务改动无因果 | 满载下连跑 20 次零红，或改成不依赖墙钟的判据 | 已改成不依赖墙钟的判据：预算判据抽为模块级纯函数 within_budget(solve_p99_ms, step_p99_ms)，测试用构造耗时验证判据本身；脚本层面只断言结构，不再对绝对毫秒数断言、也不再要求小规模运行 exit 0；改后该文件连跑 20 次全绿。完成于提交 e85e828fdc31131cd9dfe858cf9d7454179e53db |
| 已完成 | ADR D10 正文里的代码行号随 `arm_ik.py` 改动漂移 | 逐条核对引用的行号与当前文件一致 | 纯文档；本轮已逐条核正，提交 16e6ef9 |
| 已完成 | `tests/test_arm_ik.py` 的 `T-IK-4.2` 有一处「六关节在区间内」的断言读的是夹紧后的 `q_deg`，对任何实现恒真；与 `T-IK-4.1` 修前的同形态缺陷 | 断言改读 `_solve_once` 的原始输出并比 `LIMIT_EPS_DEG`，且能构造出真外溢时该断言会失败 | 已改为读 _solve_once 的未夹紧原始输出并比 LIMIT_EPS_DEG（与 T-IK-4.1 共用 helper）；实测旧断言在真外溢时仍通过、新断言在真外溢时失败、正常场景通过。完成于提交 e85e828fdc31131cd9dfe858cf9d7454179e53db |

## 2 现场

| 状态 | 事项 | 判据 | 备注 |
| --- | --- | --- | --- |
| 待办 | P0：确认宿主 `robot_metadata.schema_version` 是否为整数 2 | `capture` exit 0 且 JSON 里该值为 2 | **2026-10-25 实测：未通过。** Pi `10.34.170.98` 的 vendor 仓库提交 `ec65318f`，`src/lerobot/robots/alohamini/alohamini_host.py` 第 129 行为 `"schema_version": 1,`，即源码级的 1，不是「代码 2、运行时 1」的组合。解除后才能做指纹采集与受控试动。现场执行仍以 `docs/s4-field-smoke.md` 第 2 节为准（步骤 `0.3-pi-schema` 已改为只查单文件），最终以 `capture` 实测为准 |
| 待办 | 启动 Pi 上的 Host 后才能做指纹采集与阶段 2 的 Gate F 拒绝记录 | `0.5-host-procs` 有进程、`0.6-ports` 见 5555/5556 的 `LISTEN`，随后 `1.1-capture` 能连上 | 2026-10-25 实测：Pi 上 Host 未运行（`0.5` 无进程、`0.6` 无 5555/5556 监听；`0.4b-tools` 已证 `ss`/`netstat` 存在，不是工具缺失）。需现场启动 Host，**需要人工在场与授权**，本机做不到 |
| 待办 | 启动 Host 后重跑 `4.2-host-cmdline` 与 `4.3-host-config`，复核三个配置字段（3 / 1000 / 50 目前来自源码默认，不是运行中进程的实际生效值） | 两次输出能判出实际生效值，写进 `docs/s4-field-smoke.md` 第 8 节的实测记录 | 依 `docs/s4-field-smoke.md` 第 8 节 `4.2`/`4.3` 的判据；`4.2` 需 `ps -o cmd` 里出现 `python` 才算取到 |
| 待办 | 完成 S4 现场窗口的零运动部分与受控试动 | 按 `docs/s4-field-smoke.md` 第 11 节逐条勾选 | 逐条命令、判据、产物见该文档；现场执行以 `docs/s4-field-smoke.md` 为准，Pi 侧信息一律用 SSH 取（`ssh "$PI" '…'`），不靠口头转述 |

## 3 待定

| 状态 | 事项 | 判据 | 备注 |
| --- | --- | --- | --- |
| 已完成 | 极大但有限的限位（`abs` 远超实际量级，如 `(2**63, 2**64)`）是否要在入口设上限，还是维持现状以 placo 的 `QPError: Infeasible QP` 收场 | 定下后要么在入口加量级上限并补用例，要么在 ADR D10 明确记录维持现状 | 评估后维持现状，见 ADR D10 |
| 已完成 | 证据目录长期位置：固定在 `~/pi-workspace/alohamini-cartesian-teleop/evidence/`，还是本库内另立 `evidence/` | 定下后在 `AGENTS.md` 第 8 节与现场文档同步 | 维持外部目录，命名约定见该目录 README 与 `docs/s4-field-smoke.md` 第 10 节 |

## 4 已完成

| 状态 | 事项 | 证据 |
| --- | --- | --- |
| 已完成 | 迁移收口：`benchmarks/alohamini_ik`（v0.3.0）、`benchmarks/alohamini_io` 两脚本与配套测试（v0.4.0）；测试 999 通过 / 2 跳过 | 提交 `d23be22`、`f624dd5`，tag `v0.4.0` |
| 已完成 | S4 现场实施文档 | 提交 `41ace61` |
