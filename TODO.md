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
| 待办 | 能力探测只验属性存在，未验内容。若客户端有 `latest_request_timing` 但值为空 dict（该字段在 `__init__` 里初始化为 `{}`），探测会放行，rtt 可能写出空值行。真实路径上 `connect()` 已填充，当前不可达。 | 首帧计时为空时也判 `9`，并有测试锁定 | 见 `docs/s4-field-smoke.md` 第 13 节 |
| 待办 | `tests/test_examples_and_configs.py` 的 `SITE_VALUES` 冻结值仍是 `1` / `500` / `50`，与 `configs/site.yaml` 本轮改成的现场部署值 `3` / `1000` / `50` 不一致 | `test_site_yaml_values_are_frozen` 与 `test_site_yaml_loads_with_expected_settings` 转绿，且 `pytest tests` 回到 `1074 passed, 2 skipped` | 本轮改 `configs/site.yaml` 时发现；两处口径得先定一个（冻结值跟现场部署值，或 site.yaml 回退），本轮未动 `tests/` |

## 2 现场

| 状态 | 事项 | 判据 | 备注 |
| --- | --- | --- | --- |
| 已完成 | P0：确认宿主 `robot_metadata.schema_version` 是否为整数 2 | `capture` exit 0 且 JSON 里该值为 2 | 2026-10-25 已关闭。Pi 上的运行时从 v1 升级到含 metadata v2 的版本：完全跑通的路径 `/home/pi5/lerobot_alohamini`，HEAD `6aee1f81d503be98972d605ed32189221a674607`；旧部署原样备份在 `/home/pi5/lerobot_alohamini.legacy-ec65318f-20261025`（`ec65318f` 加 6 个未提交改动）。关闭依据：`capture` exit 0、`robot_metadata.schema_version` 为 `2`、18 个电机的 `homing_offset` 与 `resolution` 全部有值；改名后厂商测试 `60 passed`。v1 时期的两级证据（源码级 1、`capture` exit 6）保留在 `docs/s4-field-smoke.md` 第 2 节 |
| 已完成 | 阶段 1 的指纹与 RTT（原表述为「仍缺」） | `1.1-capture` exit 0 且写出 JSON；`1.2` / `1.3` 能采样 | 2026-10-25 已取：`capture` exit 0、`schema_version` 为 `2`、18/18 字段齐全（`capture/fingerprint-final-20261025.json`）；`measure --mode rtt` exit 0，`achieved_rate_hz=49.8`（请求 1000 Hz）、`host_timing_missing=0/100`。这组 RTT 是 Pi 本机回环，不是 PC↔Pi 链路延迟。两脚本的退出码 `9` 早先曾因 `PYTHONPATH` 未指向被测代码而误触，见 `docs/s4-field-smoke.md` 第 5 节 |
| 已完成 | 启动 Host 后重跑 `4.2-host-cmdline` 与 `4.3-host-config`，复核三个配置字段（3 / 1000 / 50 目前来自源码默认，不是运行中进程的实际生效值） | 两次输出能判出实际生效值，写进 `docs/s4-field-smoke.md` 第 8 节的实测记录 | 依 `docs/s4-field-smoke.md` 第 8 节 `4.2`/`4.3` 的判据；`4.2` 需 `ps -o cmd` 里出现 `python` 才算取到；2026-10-25 Host 启动后已核对，启动命令未含覆盖 observation_request_window / watchdog_timeout_ms / max_loop_freq_hz 的参数，三值确为源码默认 |
| 待办 | 完成 S4 现场窗口的受控试动（阶段 3，Mode B） | 按 `docs/s4-field-smoke.md` 第 11 节逐条勾选 | 零运动部分已完成（见本表 P0 与阶段 1 两行），Mode B 从未在真机执行，仍需现场与授权。逐条命令、判据、产物见该文档；Pi 侧信息一律用 SSH 取（`ssh "$PI" '…'`），不靠口头转述 |
| 已完成 | v0.7.0 的退出码 `9` 与 `capture` 的 `null` 计时字段在真机上验证 | Host 运行时对 Pi 跑一次 `measure --mode rtt`，得到 `9` 与那段 stderr | 2026-10-25 已在真机触发 `9`：在 Pi 上不设 `PYTHONPATH` 时 `import lerobot` 落到已安装的旧版本，`measure --mode rtt` 实际返回 `9`；`capture` 同期表现为计时字段全 `null`。这条退出码因此不再只有单元测试背书。正常路径（`PYTHONPATH` 指向 `/home/pi5/lerobot_alohamini`）下 `1.2` / `1.3` exit 0 |
| 待办 | Gate F 第二段（换算字段不全时的拒绝）在 v2 宿主上未验证：18/18 电机字段齐全，第二段不会拒，这条拒绝路径没有真实记录 | 拿到一份第二段拒绝的记录：命令原文、stderr 原文、退出码 `6`、`send_action` 为 0；或在测试里注入换算字段不全的 metadata 并锁定拒绝 | 需构造换算字段不全的宿主，或注入测试。不得为造记录改标定文件或伪造数据，见 `docs/s4-field-smoke.md` 第 6 节与第 13 节 |
| 待办 | 4 路相机在采集时全返回零帧（`Frame is None`），另有两次 ZMQ 请求超时 | 查清零帧原因（相机节点、权限、采集进程占用），并拿到一次非空帧的采集记录 | 关节与 metadata 正常，未阻塞零运动阶段；带图像的用例会拿到空帧。见 `docs/s4-field-smoke.md` 第 13 节 |
| 待办 | `docs/s4-field-smoke.md` 与 `configs/site.yaml` 的三个数值字段口径已在本次统一：`observation_request_window`、`watchdog_timeout_ms`、`max_loop_freq_hz` 取机器人侧现场部署值 `3` / `1000` / `50`（Pi `config_alohamini.py:110`、`:122`、`:125`，见 `docs/s4-field-smoke.md` 第 8 节）。本库自己的默认值 `1` / `500` / `50` 仍在 `configs/template.yaml`，两者不是同一份东西。 | 若后续现场改配置，`docs/s4-field-smoke.md` 第 8 节与 `configs/site.yaml` 同一次改完，且 `uv run --no-sync python -c "from alohamini_cartesian.config.overlay import load_settings; print(load_settings(template='configs/template.yaml', site='configs/site.yaml'))"` 不抛 `ConfigError` | 本次只改 `configs/site.yaml` 一处，`docs/s4-field-smoke.md` 原本已记录同一组值，未改 |

## 3 待定

| 状态 | 事项 | 判据 | 备注 |
| --- | --- | --- | --- |
| 已完成 | 极大但有限的限位（`abs` 远超实际量级，如 `(2**63, 2**64)`）是否要在入口设上限，还是维持现状以 placo 的 `QPError: Infeasible QP` 收场 | 定下后要么在入口加量级上限并补用例，要么在 ADR D10 明确记录维持现状 | 评估后维持现状，见 ADR D10 |
| 已完成 | 证据目录长期位置：固定在 `~/pi-workspace/alohamini-cartesian-teleop/evidence/`，还是本库内另立 `evidence/` | 定下后在 `AGENTS.md` 第 8 节与现场文档同步 | 维持外部目录，命名约定见该目录 README 与 `docs/s4-field-smoke.md` 第 10 节 |
| 已完成 | 给 `capture` / `measure --mode rtt` 加客户端能力探测：缺 `latest_request_timing` 时显式中止并给专用退出码，禁止静默把 rtt 各列写成空值或 NaN（`host_timing_missing` 不得谎报为 False）。 | 对缺该属性的客户端，两脚本显式中止并返回专用退出码，不写出全空或 NaN 的 rtt CSV | 已完成，提交 `c919fef`（后续 `a0d82b3`、`36445e1`、`1c2d1fd` 为探测前移与测试）。现有行为：`measure --mode rtt` 返回 `9` 且不写 CSV；`capture` 继续采集并把计时字段记 `null` |
| 待定 | 为 `lerobot` 的接口依赖留下可判据的声明（当前 `pyproject.toml` 只写名字、无版本，测试夹具又同时 mock 两个属性，导致接口不兼容无法在测试期暴露）。 | 声明形式定下并写入 `pyproject.toml` 或 ADR，使接口不匹配能在安装期或测试期暴露 | 后果与证据见 `docs/s4-field-smoke.md` 第 13 节 |

## 4 已完成

| 状态 | 事项 | 证据 |
| --- | --- | --- |
| 已完成 | 迁移收口：`benchmarks/alohamini_ik`（v0.3.0）、`benchmarks/alohamini_io` 两脚本与配套测试（v0.4.0）；测试 999 通过 / 2 跳过 | 提交 `d23be22`、`f624dd5`，tag `v0.4.0` |
| 已完成 | 最小离线示例 `examples/minimal_dry_run.py`、两份配置样例 `configs/template.yaml` 与 `configs/site.yaml`、配套测试 `tests/test_examples_and_configs.py`；本轮测试 `1074 passed, 2 skipped` | 提交 `08c1318`（建示例与两份样例）、`c9baf3e`（site.yaml 证据路径更正、计数守卫加固）、`a175f49`（证据引用改绝对路径、守卫升级为 AST 与会话读数断言） |
| 已完成 | V01–V13 准入基线文档 `docs/acceptance-baseline.md`，替代旧仓库按臂归档的 Gate A–E | 逐条软件证据可点开核对；实机侧 11 条未取、2 条部分，汇总见该文 2.3 节 |
| 已完成 | S4 现场实施文档 | 提交 `41ace61` |
| 已完成 | P0 关闭：Pi 从 v1 升级到含 metadata v2 的运行时 | Pi 路径 `/home/pi5/lerobot_alohamini`，HEAD `6aee1f81d503be98972d605ed32189221a674607`；备份 `/home/pi5/lerobot_alohamini.legacy-ec65318f-20261025`；`capture` exit 0、`schema_version` 为 `2`、18/18 字段齐全；厂商测试 `60 passed` |
| 已完成 | 证据回收：v2 宿主上的指纹、RTT 与 Host 日志 | `evidence/s4-20261025/` 下 `capture/fingerprint-final-20261025.json`（7075 B）、`capture/fingerprint-merged-6aee1f81.json`（7076 B）、`rtt/rtt-loopback-merged-20261025.csv`（18781 B）、`rtt/rtt-loopback-codex-20261025.csv`（18759 B）、`stdout/{final-host,v2merged-host,v2verify-host}-20261025.log` |
