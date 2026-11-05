# 背景与目标

## 1 目标

一句话：把 AlohaMini 2 Pro 从「遥操作主臂牵着走」改成「PC 上一段 Python 就能让手臂按笛卡尔目标动」。

首版目标取自设计规范文件头 DOCUMENT-CONTRACT 的 Must Preserve 段：以「脱离 leader 后的受控运动验证」为目标，不承诺完整自主规划或取放任务。

三步顺序：先脱离 leader，关节动作不再由遥操作主臂产生；再把入口换成 PC 侧库的 `connect` / `move_to` / `move_by`；底层仍走现有发送链、单位与硬件保护。

## 2 形态变化

| 维度 | 改造前 | 改造后 |
| --- | --- | --- |
| 关节动作从哪来 | 遥操作主臂（leader）牵引，由 teleoperation 产生 | PC 侧库按笛卡尔目标算出 |
| 发送链、单位与硬件保护 | 现有实现 | 现有实现，不改 |
| 需不需要主臂在场 | 需要 | 不需要 |

## 3 平台与依赖

| 项 | 内容 |
| --- | --- |
| 上游仓库 | <https://github.com/liyiteng/lerobot_alohamini.git>，Pi 上部署的就是这份 |
| 本地 fork | `/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini` |
| Pi 上的部署路径 | `/home/pi5/lerobot_alohamini` |
| Host 进程（常驻主程序）、电机总线、相机、录制 | 由厂商仓库提供 |
| 本库跑在哪 | 只在 PC 一侧，不跑在 Pi 的 Host 进程里 |
| 本库与厂商代码的唯一接触面 | `src/alohamini_cartesian/adapter/protocol.py`；本库不修改厂商运行时 |

## 4 当前结果（截至 2026-11-05）

2026-11-05 存在第二次现场会话：远程 SSH、Pi 运行时 HEAD `6aee1f81`、无人在机器人现场、未断电；该次会话的结论与数字见 `docs/field-results.md` 的 2026-11-05 节，本文不复制数字表。

### 4.1 代码侧

| 项 | 值 |
| --- | --- |
| 版本 | `0.7.0`（`src/alohamini_cartesian/__init__.py`） |
| 测试 | 当前测试数见 `docs/changelog.md` |
| 跳过的两条 | `tests/test_control_api_lerobot_parity.py`、`tests/test_kinematics_lerobot_parity.py`，需要对 vendor fork（`liyiteng/lerobot_alohamini`）做差分对照；**不等于全部通过** |
| 核心依赖与 `lerobot` 引用位置 | 核心只依赖 numpy；`src/` 内**任何**文件（含 `adapter/protocol.py`）都不 import `lerobot`，由 `tests/test_capability.py` 的 AST 断言守着 |
| 决策记录 | `docs/ADR.md` 现有 D1–D19 |

### 4.2 已实现能力

| 模块 | 已提供什么 |
| --- | --- |
| `adapter/protocol.py` | `RobotTransport`（`get_observation`、`send_action`、`prime_observation_request_window`）与 `RobotSpec` 两个 `typing.Protocol`，本库对 `lerobot` 的唯一接触面 |
| `capability/detect.py` | `detect_from_observation` 解析 `_robot_metadata`（schema 1 与 2，其余 fail-closed）；`resolve_deg_per_unit` 按 metadata → 标定文件 → 型号表三级给 `deg_per_unit`，缺刻度抛 `ConversionUnavailable` |
| `config/overlay.py` | `load_settings` 做默认值 → 模板 → 现场文件 → 环境变量四级覆盖，`cameras` 按键合并，未知键与类型不符抛 `ConfigError`，`check_devices=True` 时校验 `/dev/` 节点 |
| `control/api.py` | `connect`、`AlohaMiniControl`、`ArmHandle.move_to`/`move_by`、`LiftHandle`、`BaseHandle`、`GripperHandle`；`dry_run` 默认 `True`，非 dry-run 必须 `enable_motion=True`，未注入 `client_factory` 抛 `RuntimeUnavailable` |
| `control/commands.py` | `ControlState` 与 `Stage` 枚举、7 类 frozen 命令（选臂、夹爪、底盘、升降、停、恢复、释放）构造期校验、`ControlStateMachine` 管 STANDBY/READY/ACTIVE/SETTLING/BLOCKED/FAULT 与阶段互斥 |
| `control/controller.py` | `CartesianController.tick`：状态门、adapter 换算、过流 hold 有界退让、跟随超时、IK 求解、残差门、步长门（2.0°/周期）、领先夹紧、转回电机刻度下发；`dry_run` 在 `_send` 短路 |
| `control/arm_ik.py` | `ArmIK.fk` 与 `solve`，返回 `IKResult`（`q_deg`、`ok`、`reason`、`position_error_m`、`orientation_error_deg`、`iterations`、`solve_time_s`）；失败原因 `NON_FINITE` / `JOINT_LIMIT` / `RESIDUAL`；限位在 `solve` 入口校验，`placo` 函数内惰性 import |
| `kinematics/joint_adapter.py` | `JointKinematicAdapter` 在电机刻度与 URDF 关节角之间双向换算（sign/zero 构造期注入）；`effective_limits_deg` 取标定区间与 URDF 限位的交集 |
| `kinematics/model_specs.py` | `ROBOT_SPECS` 型号表、`ARM_PROFILE_JOINTS` 关节名表、`validate_robot_model`、`arm_state_keys` |
| `kinematics/assets/am_arm200/kinematics.urdf` | 单臂六旋转关节的最小运动学模型，不含 mesh 与碰撞体 |
| `benchmarks/alohamini_ik/benchmark_arm_ik.py` | 离线 FK/IK 基准：p50/p95/p99 求解耗时与整步耗时、位置残差、相邻步关节变化；判据 solve p99 ≤ 7.0 ms、整步 p99 ≤ 15.0 ms |
| `benchmarks/alohamini_io/capture_robot_fingerprint.py` | 只读采集对端 metadata 指纹，零 `send_action`；`schema_version` 不是 2 时 exit 6 且不写盘 |
| `benchmarks/alohamini_io/measure_latency.py` | 两个模式：`rtt`（不动）与 `tracking`（Mode B，会真动机器人）；退出码 0/1/3/5/6/7/8/9，无 `latest_request_timing` 时 exit 9 |

### 4.3 设计条目 → 实现位置 / 状态

按设计规范的文档地图逐项对到本库。

| 设计条目 | 实现位置 / 状态 |
| --- | --- |
| Python 控制库 SDK（`connect` · `move_to` · `move_by` · `lift` · `base`） | 已实现：`src/alohamini_cartesian/control/api.py` |
| KeyboardTeleop（可选上层调试示例） | 部分：离线示例 `examples/minimal_dry_run.py` 已入库（dry-run 全链路，零发送）；键盘调试层未实现 |
| 通用抽象命令（选臂 · 速度向量 · 夹爪 · 停 · 恢复） | 已实现：`src/alohamini_cartesian/control/commands.py` |
| 控制状态与阶段互锁（`stop()` 锁定 · `resume()` 恢复 · base/lift 与 arm 互斥） | 已实现：`src/alohamini_cartesian/control/commands.py`（`ControlStateMachine`） |
| Base / Lift action | 已实现：`src/alohamini_cartesian/control/api.py`（`BaseHandle`、`LiftHandle`）与 `src/alohamini_cartesian/control/controller.py`（`_tick_base_command`、`_tick_base_gap`） |
| CartesianController 内核 | 已实现：`src/alohamini_cartesian/control/controller.py` |
| 独立 ArmIK 组件 | 已实现：`src/alohamini_cartesian/control/arm_ik.py` |
| JointKinematicAdapter | 已实现：`src/alohamini_cartesian/kinematics/joint_adapter.py` |
| 动作合并 | 部分：已做的是阶段互斥，同一时刻只有一条路径在发（`control/controller.py` 的状态门）；未做的是把两臂、底盘、升降、夹爪组装成一帧的合并层，现状各子系统各发局部动作字典（`control/api.py` 的 `send_action` 调用点、`control/controller.py:294` 的 `_base_action`） |
| 统一发送门禁（dry-run 不得 `send_action`） | 已实现：`control/controller.py` 的 `_send` 在 `dry_run` 下直接返回，`control/api.py` 各 handle 与 `_send_initial_hold` 均有 `dry_run` 守卫，`tests/test_control_controller.py` 的 `T-C3` 系列覆盖全生命周期零发送 |
| AlohaMiniClient · ZMQ · Host | 部分：本库只定义 `RobotTransport` 协议并要求调用方注入 `client_factory`（`control/api.py` 的 `connect`）；Client、ZMQ 与 Host 实体属厂商代码，本库不实现 |
| 反馈与 metadata（位置 · 时效 · 保护状态） | 已实现：`capability/detect.py` 解析 metadata；`control/api.py` 的 `_resolve_feedback` 用 `sample_sequence` 判时效并把 `settled`/`valid` fail-closed；`control/controller.py` 按 `_control_status.joint_holds` 做过流退让。`_host_timing` 只探测存在，控制层不用它做门禁 |

### 4.4 实机现状

本节与本文其余部分的现场日期与钟点都是 UTC，本地时间为 UTC+8（UTC 07:00 对应本地 15:00，UTC 09:33 对应本地 17:33）。

| 阶段 | 状态 | 依据 |
| --- | --- | --- |
| 阶段 0 到场核对 | 已做 | 证据目录 `stdout/0.*`，含 Pi 提交、`schema_version` 行、Host 进程与端口 |
| P0：宿主广播 `robot_metadata.schema_version` 为整数 2 | 已关闭 | Pi 上的运行时已从 v1 升级到含 metadata v2 的版本。完全跑通的路径 `/home/pi5/lerobot_alohamini`，HEAD `6aee1f81d503be98972d605ed32189221a674607`（该时点的 HEAD，UTC 09:33:54 的采指纹已在这版上完成；`main` 与 `codex/alohamini-cartesian-teleop` 的合并提交链；升级前 UTC 07:00 前后的 HEAD 是 `ec65318f`，见旧部署目录名）。旧部署备份在 `/home/pi5/lerobot_alohamini.legacy-ec65318f-20261025`（`ec65318f` 加 6 个未提交改动，原样保留）。关闭判据：`capture` exit 0，且 JSON 里 `robot_metadata.schema_version=2`。升级后厂商测试 `60 passed`，`import lerobot` 落点为 `/home/pi5/lerobot_alohamini/src/lerobot/__init__.py` |
| 阶段 1 零运动测量与指纹 | 已取得 | `capture` exit 0，`schema_version` 为 `2`，18 个电机的 `homing_offset` 与 `resolution` 全部有值，样本 5，与 codex 原分支逐字段对比 `field diffs: 0`。`measure --mode rtt` exit 0，`achieved_rate_hz=49.8`（请求 1000 Hz，受 Host 50 Hz 主循环封顶），`request_age_ms` p50 20.109，`net_rtt_estimate_ms` p50 6.176，`host_timing_missing=0/100`。这组 RTT 是 Pi 本机回环，不是 PC 到 Pi 的链路延迟 |
| 阶段 2 制造代码 Gate F 拒绝 | 第一段已取得，第二段未验证 | 第一段：v1 宿主时期 exit 6、零命令，报文 `Gate F refused: the host metadata schema_version is 1, expected 2.`。第二段（换算字段不全时的拒绝）在 v2 宿主上未触发、未验证：现在 18/18 字段齐全，第二段不会拒 |
| 阶段 3 受控试动（Mode B） | 已首次执行（2026-10-25） | 腕部跟随 87.9%（`arm_left_wrist_roll`，净行程 +1.758°/指令 2.0°，两步 `settled=True`，误差 0.121°）；肩部未跟随：首次试动 `arm_left_shoulder_pan`（1°×3 步）5.9%，随后同一轴 8.8%（1°×3 步）与 8.8%（1°×2 步），`arm_left_shoulder_lift` 13.2%（1°×2 步），全部 `settled=False`，未停稳。五份 CSV 的 `note` 列无 `aborted`，均未中途中止。扭矩状态与肩部不跟随的原因排查中。数据见 `docs/s4-field-smoke.md` 第 7 节 |
| 阶段 4 配置样例 | 已完成 | 两份 YAML 已入库（`configs/template.yaml`、`configs/site.yaml`），`site.yaml` 的 `observation_request_window` / `watchdog_timeout_ms` / `max_loop_freq_hz` 取现场实测 `3` / `1000` / `50`（出处 Pi `config_alohamini.py:110/122/125`，见 `docs/s4-field-smoke.md` 第 4.3 节）；两份都能被 `load_settings` 读通。`stdout/4.*` 命令与日志齐备 |

证据目录：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261025/`。逐条命令与判据见 `docs/s4-field-smoke.md`。

## 5 边界

取自设计规范的 Non-goals 段，末行为本库自己的边界。

| 边界 | 内容 |
| --- | --- |
| 不做完整自主规划与动态避障 | 首版只做受控运动验证 |
| 不支持直接 Roll/Pitch/Yaw 姿态输入 | 姿态是软约束（weight 0.01），只记录偏差，优先保证位置精度 |
| 无动力学与力控 | 不建模惯量、摩擦、接触力 |
| 不自动抓取 | 夹爪只有开、合、指定行程三个动作 |
| 不用仿真模型或合成数据替代实机验证 | 缺证据就标注缺证据，不拿仿真补齐 |
| 本库自己的边界 | 不改 vendor 运行时；厂商内部协议（ZMQ 报文、Host 行为）不承担回归责任 |

## 6 权威文档索引

| 文档 | 管什么 | 冲突时谁优先 |
| --- | --- | --- |
| 设计规范：`/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini-cartesian-teleop/docs/alohamini-control/Alohamini-control-design.md`（自标 SSOT，位于已退役仓库，未迁入本库） | 历史设计来源：接口契约、状态机、决策 D001–D018；Gate A–E 已归档，不再是现行验收口径 | 不参与优先级：历史设计来源与溯源参考 |
| `docs/ADR.md` | 本库落地时的取舍与代价（D1–D19） | 与本库代码冲突时以它为准；与设计规范冲突时以本库为准，差异记入 `docs/ADR.md`（D17） |
| `docs/acceptance-baseline.md` | 验收口径（V01–V13）：逐条要求、软件证据、实机证据与判据 | 验收以它为准，是唯一验收口径；退役仓库那份按臂归档的 Gate A–E 已由它替代（D17） |
| `docs/s4-field-smoke.md` | 现场逐条命令、判据、产物、退出码 | 现场怎么执行以它为准 |
| `docs/field-results.md` | 现场实际取到的结论与数字，按时间倒序，每条给证据文件 | 现场拿到了什么以它为准；数字与本文第 4.4 节或现场手册不一致时，以它引的证据文件为准 |
| `docs/library-guide.md` | 配置覆盖规则、能力探测与单位换算、控制层、公开 API、代码约束与机器守卫 | 接口与行为以它为准 |
| `docs/changelog.md` | 各版本的改动、当时的测试条数与提交哈希（v0.2.0 → v0.7.0） | 历史事实以它为准 |
| `docs/investigations/gripper-range-51-tick.md` | 2026-11 离线调查：左夹爪标定行程 51 tick 的查证过程与结论（决策见 `docs/ADR.md` D14，数字见 `docs/field-results.md`） | 查证过程以它为准；结论与决策仍以 ADR / field-results 为准 |
| `TODO.md` | 接下来要做的事、判据与影响面（整体目标 → 阶段目标 → 小目标 → 跨阶段工程债） | 事项状态与优先级以它为准 |

### 命名对照：两组 Gate 不是一回事

| 名字 | 出处 | 指什么 |
| --- | --- | --- |
| Gate A–E | 设计规范第 9 节 | 模型门槛：标定与换算的模型正确性。**已归档**：Gate 系统与设计规范 D018 定下的归档 JSON 格式在本库无承载物，不再用作准入记录，验收由 V01–V13 替代（`docs/ADR.md` D17） |
| Gate A–F | `benchmarks/alohamini_io/measure_latency.py` 的代码运动门禁 | 运行时门禁：这次测量能不能发命令 |

两组不要混用。
