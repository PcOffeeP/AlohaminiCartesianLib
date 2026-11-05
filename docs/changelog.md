# 变更记录（v0.2.0 → v0.7.0）

## v0.7.0

v0.7.0 给两个现场基准脚本加了客户端能力探测：`measure_latency.py --mode rtt` 在 `connect()` 返回后探测 `latest_request_timing`，缺该属性时打印说明并以新退出码 `9`（`EXIT_CLIENT_UNSUPPORTED`）结束，不写 CSV（连表头都不写）；`capture_robot_fingerprint.py` 缺该属性时继续采集，把 `age_s`、`sent_monotonic_s`、`received_monotonic_s` 记为 `null`，缺 `latest_control_status` 时 `control_status` 记 `null`。不用 `getattr` 静默回退，也不拿 `latest_safety_status` 顶替 `latest_control_status`。版本号从 `0.6.0` 升到 `0.7.0`。取舍与代价见 [ADR.md](ADR.md) D12。

本轮 `pytest tests` 报 `1056 passed, 2 skipped`。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。在 Pi 当前 vendor（`main`，`ec65318f`）上跑 rtt 返回 `9` 是设计行为，不是故障。

补一份最小离线示例 `examples/minimal_dry_run.py`，与两份配置样例 `configs/template.yaml`、`configs/site.yaml` 一起入库，配套测试 `tests/test_examples_and_configs.py` 用子进程读数与 AST 守卫钉住「dry-run 零发送」。本轮 `pytest tests` 报 `1074 passed, 2 skipped`，提交 `08c1318`、`c9baf3e`、`a175f49`。现场配置的三个数值字段（`observation_request_window`、`watchdog_timeout_ms`、`max_loop_freq_hz`）按机器人侧运行时的实际配置填写，取值以 [site.yaml](../configs/site.yaml) 为准（该文件内每个字段的注释写明取自 `config_alohamini.py` 的哪一行），取数命令见 [s4-field-smoke.md](s4-field-smoke.md) 第 4.3 节。

后续两轮都落在 `benchmarks/alohamini_io/measure_latency.py` 的客户端能力探测与退出码 `9` 上。`4e6b5c5` 加了第二道探测（属性存在但内容不可用也拒），并给 Gate F 第二段的拒绝补了端到端测试；`dc5356ae` 让半填充的计时面同样被拒，把「`observation_sequence` 冻结 + 计时面为空」从退出码 `7` 改判为 `9`，并消除 0 行进展时的空转。本轮 `pytest tests` 报 `1108 passed, 2 skipped`（前一轮 1074）。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。退出码 `9` 的四种形态见 [ADR.md](ADR.md) D12：除缺属性、整轮空面、半填充（缺键）之外，第四种是「键在、值不可用」——负 `age_s`、`bool` 时间戳或年龄、非 `str` 或只有空白的 `token`、非有限数值，同样判 `9` 且不写 CSV。

准入基线改为 V01–V13，逐条给出软件证据与实机证据，见 [acceptance-baseline.md](acceptance-baseline.md)。它替代旧仓库按臂归档的 Gate A–E。

新增 `examples/robot_dry_run.py`：真厂商 client 注入 `connect(dry_run=True)` 的参考实现。`build_client_factory()` 返回签名 `(host, **kwargs) -> client` 的工厂，厂商 `AlohaMiniClient` 与 `AlohaMiniClientConfig` 的 import 落在最内层工厂函数体内；脚本读一次真观测、对升降轴下一条 dry-run 命令、断开，`send_action` 零调用。配套 `tests/test_robot_dry_run_example.py` 14 条用例：AST 守卫钉住三件事——模块顶层无 `lerobot` import、`connect(...)` 必带 `dry_run=True` 且无 `dry_run=False` 的 `connect`、源码里保留 `assert <x>.send_action_calls == 0` 形态的真断言；子进程契约也钉住三件事——主环境（无 `lerobot`）退 `0`、stdout 含「需要 vendor 环境」与 `PYTHONPATH` 跑法、stderr 无 `Traceback`。同一轮把 `tests/test_examples_and_configs.py` 契约 C 从文本级子串扫描改成 AST 作用域检查（顶层禁、函数体内允许惰性导入），并补 5 条守卫自测（4 条参数化顶层违规 + 1 条函数体内放行），守卫不再可能退化成空检查。本轮 `pytest tests` 报 `1416 passed, 2 skipped`，较本轮开工前的 `1397 passed, 2 skipped` 多 `19` 条 = `tests/test_robot_dry_run_example.py` 14 条 + 契约 C 新增守卫自测 5 条。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。

`00fa714` 给指纹 JSON 加 `lerobot_version` 字段：取值优先 `--lerobot-version` 原样传入，未传时读本机 `importlib.metadata.version("lerobot")`，两处都取不到记显式 `null`；`--lerobot-version` 传空串按用法错误退 `2`（空串不是版本，而 `null` 的语义是「取不到」，两者不得混读）。要这一列是因为采集在 Mac 侧、Pi 上跑的是 vendor 快照，缺了它这份 JSON 说不清对应哪个 `lerobot`。现场首次实采值 `lerobot_version="0.6.1"`，与手册 4.1b 读到的 Pi 侧版本逐字一致，产物 `capture/fingerprint-20261105T063600Z.json`（`docs/field-results.md` 2026-11-05 节）。

补记 tracking 模式的旁挂文件（取舍见 [ADR.md](ADR.md) D18）：`--mode tracking` 除 14 列主 CSV 之外，在同目录写一份 `<主CSV>-safety.csv`——9 列长表，一帧一电机一行，列序固定为 `step_index`、`sample_sequence`、`t_monotonic_s`、`motor`、`current_ma`、`requested_target`、`accepted_target`、`hold_active`、`target_source`。降级口径：客户端缺 `latest_safety_status`（能力缺失）或该面内容不可用（整轮计数，点名坏帧数）时，退出码不变、sidecar 照写，缺值写 `None`（空单元格），不补 `0`——`0 mA` 读起来像测到的零电流，是伪观测。`--mode rtt` 不产生 sidecar。

## v0.6.0

v0.6.0 修正了失败原因分类：`ArmIK.solve()` 结尾不再把「没到位」一律报 `JOINT_LIMIT`，而是按越限与否分流——解越出注入限位（越限判定仍含 `LIMIT_EPS_DEG` 容差）报 `JOINT_LIMIT`，未越限但位置残差超过 `profile.residual_limit_m` 报 `RESIDUAL`，两者同时成立时 `JOINT_LIMIT` 优先。`FAILURE_REASONS` 不新增枚举值，`IKResult` 字段不变。

对外行为变更一处：一部分失败的 `reason` 从 `JOINT_LIMIT` 变为 `RESIDUAL`，经控制层 `getattr(ControlError, result.reason, ...)` 映射后对外错误码同步从 `ControlError.JOINT_LIMIT` 变为 `ControlError.RESIDUAL`，按错误码分流的调用方需要调整。版本号从 `0.5.0` 升到 `0.6.0`。取舍与代价见 [ADR.md](ADR.md) D11。

本轮 `pytest tests` 报 `1031 passed, 2 skipped`。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。新增 4 条用例落在 `tests/test_arm_ik.py` 的 `T-IK-19.x` 分类矩阵（在限位内/外 × 残差达标/超标逐格），红阶段先失败、改完实现后转绿。

## v0.5.0

v0.5.0 给 `ArmIK.solve()` 的关节限位加了入口校验：两侧必须是有限实数、必须是 `(lo, hi)` 二元序列，排除 `bool` 与 `memoryview` 这类伪数字，并且必须满足 `lo < hi`，违规抛 `ValueError` 并点名关节与两侧取值。改动前，倒置限位会让 placo 抛裸 `QPError`，退化限位会被标成 `JOINT_LIMIT`，两者都不是配置错误该有的出口。多余关节键定为忽略，由用例钉住。`solve()` 的签名与 `IKResult` 的返回类型未变。版本号从 `0.4.0` 升到 `0.5.0`。对外行为变更一处：非法限位从 `RuntimeError` 或静默通过变成 `ValueError`，按异常类型分流的调用方需要调整。取舍与代价见 [ADR.md](ADR.md) D10。

本轮 `pytest tests` 报 `1027 passed, 2 skipped`。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。新增 28 条用例全部落在 `tests/test_arm_ik.py` 的 `T-IK-18.x` 系列，红阶段先失败、改完实现后转绿。

## v0.4.0

v0.4.0 把旧仓库 `benchmarks/alohamini_io` 的两个现场基准脚本与配套测试一并迁入：`benchmarks/alohamini_io/capture_robot_fingerprint.py`（473 行）与 `benchmarks/alohamini_io/measure_latency.py`（2151 行），配套测试 `tests/benchmarks/test_capture_robot_fingerprint.py`（638 行，静态 20 个测试函数）与 `tests/benchmarks/test_measure_latency.py`（3365 行，静态 110 个测试函数）。版本号从 `0.3.0` 升到 `0.4.0`。迁入原则是语义零改动：四个文件与旧仓库版本 `ast.dump` 逐一相等（4/4），字节不同处只是 `ruff format` 的折行（旧仓库 line-length 110，本库 100）。两个脚本对 `lerobot` 的 import 全在 `default_client_factory()` 函数体内（`capture_robot_fingerprint.py:140-141`、`measure_latency.py:847-848`），模块顶层只有标准库与 numpy；配套测试用依赖注入的 fake client 与 fake clock，不碰 `lerobot`、`zmq`、`socket`、线程。为让这四份文件过本库风格门，`pyproject.toml` 尾部按四条精确路径新增 `[tool.ruff.lint.per-file-ignores]`，全局 `select` / `ignore` 未改。取舍见 [ADR.md](ADR.md) D9。

v0.4.0 轮次 `pytest tests` 结果（本机实测，命令 `uv run --no-sync python -m pytest tests -o addopts="" -p no:cacheprovider`）：`999 passed, 2 skipped`。迁移前是 `823 passed, 2 skipped`，增量 176 条 = 23 条（capture 测试参数化展开）+ 153 条（measure 测试参数化展开）。跳过仍是那两个 parity 文件（缺 `lerobot`）。`tests/benchmarks` 子集 190 条通过 = 14（`test_benchmark_arm_ik.py`）+ 23 + 153。`ruff check .` 报 `All checks passed!`，`ruff format --check .` 报 `34 files already formatted`。

## v0.3.0

v0.3.0 做三件事：把 `placo` 声明为 `kinematics` extra（`placo>=0.10.1,<0.11`）并装进开发环境；迁入离线 IK 基准 `benchmarks/alohamini_ik/benchmark_arm_ik.py`；给 IK 判限位加容差，消除边界分类漂移。本轮 `pytest tests` 实跑 825 条，823 条通过、2 条跳过（两个 `lerobot` parity 文件在模块级跳过，收集阶段不计入）。

`benchmarks/alohamini_ik/benchmark_arm_ik.py` 与配套测试 `tests/benchmarks/test_benchmark_arm_ik.py` 已迁入，离线可跑。开发环境已按 `kinematics` extra 装上 `placo`，离线 IK 基准与 IK 数值用例真跑，不再整模块跳过。

## v0.2.0

v0.2.0 把控制层与运动学搬入库内：`control/`（会话与句柄、命令状态机、单臂控制器、`arm_ik`）、`kinematics/`（型号规格表、关节单位适配、`assets/am_arm200/kinematics.urdf`）。版本号从 `0.1.0` 升到 `0.2.0`，核心依赖从空改为 `numpy>=1.26`。行为变更一处：`connect()` 不再在内部 import `lerobot`，未注入 `client_factory` 时抛 `RuntimeUnavailable`。
