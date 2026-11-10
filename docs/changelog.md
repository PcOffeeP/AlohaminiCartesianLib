# 变更记录（v0.2.0 → 最新）

## v0.7.0（2026-11-08）

v0.7.0 给两个现场基准脚本加了客户端能力探测：`measure_latency.py --mode rtt` 在 `connect()` 返回后探测 `latest_request_timing`，缺该属性时打印说明并以新退出码 `9`（`EXIT_CLIENT_UNSUPPORTED`）结束，不写 CSV（连表头都不写）；`capture_robot_fingerprint.py` 缺该属性时继续采集，把 `age_s`、`sent_monotonic_s`、`received_monotonic_s` 记为 `null`，缺 `latest_control_status` 时 `control_status` 记 `null`。不用 `getattr` 静默回退，也不拿 `latest_safety_status` 顶替 `latest_control_status`。版本号从 `0.6.0` 升到 `0.7.0`。取舍与代价见 [ADR.md](ADR.md) D12。

本轮 `pytest tests` 报 `1056 passed, 2 skipped`。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。在 Pi 当前 vendor（`main`，`ec65318f`）上跑 rtt 返回 `9` 是设计行为，不是故障。

补一份最小离线示例 `examples/minimal_dry_run.py`，与两份配置样例 `configs/template.yaml`、`configs/site.yaml` 一起入库，配套测试 `tests/test_examples_and_configs.py` 用子进程读数与 AST 守卫钉住「dry-run 零发送」。本轮 `pytest tests` 报 `1074 passed, 2 skipped`，提交 `08c1318`、`c9baf3e`、`a175f49`。现场配置的三个数值字段（`observation_request_window`、`watchdog_timeout_ms`、`max_loop_freq_hz`）按机器人侧运行时的实际配置填写，取值以 [site.yaml](../configs/site.yaml) 为准（该文件内每个字段的注释写明取自 `config_alohamini.py` 的哪一行），取数命令见 [field-ops/runbook/1-prepare.md](field-ops/runbook/1-prepare.md) 第 4.3 节。

后续两轮都落在 `benchmarks/alohamini_io/measure_latency.py` 的客户端能力探测与退出码 `9` 上。`4e6b5c5` 加了第二道探测（属性存在但内容不可用也拒），并给 Gate F 第二段的拒绝补了端到端测试；`dc5356ae` 让半填充的计时面同样被拒，把「`observation_sequence` 冻结 + 计时面为空」从退出码 `7` 改判为 `9`，并消除 0 行进展时的空转。本轮 `pytest tests` 报 `1108 passed, 2 skipped`（前一轮 1074）。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。退出码 `9` 的四种形态见 [ADR.md](ADR.md) D12：除缺属性、整轮空面、半填充（缺键）之外，第四种是「键在、值不可用」——负 `age_s`、`bool` 时间戳或年龄、非 `str` 或只有空白的 `token`、非有限数值，同样判 `9` 且不写 CSV。

准入基线改为 V01–V13，逐条给出软件证据与实机证据，见 [acceptance-baseline.md](acceptance-baseline.md)。它替代旧仓库按臂归档的 Gate A–E。

新增 `examples/robot_dry_run.py`：真厂商 client 注入 `connect(dry_run=True)` 的参考实现。`build_client_factory()` 返回签名 `(host, **kwargs) -> client` 的工厂，厂商 `AlohaMiniClient` 与 `AlohaMiniClientConfig` 的 import 落在最内层工厂函数体内；脚本读一次真观测、对升降轴下一条 dry-run 命令、断开，`send_action` 零调用。配套 `tests/test_robot_dry_run_example.py` 在 `41e18a2` 时点 14 条用例：AST 守卫钉住三件事——模块顶层无 `lerobot` import、`connect(...)` 必带 `dry_run=True` 且无 `dry_run=False` 的 `connect`、源码里保留 `assert <x>.send_action_calls == 0` 形态的真断言；子进程契约也钉住三件事——主环境（无 `lerobot`）退 `0`、stdout 含「需要 vendor 环境」与 `PYTHONPATH` 跑法、stderr 无 `Traceback`。同一轮把 `tests/test_examples_and_configs.py` 契约 C 从文本级子串扫描改成 AST 作用域检查（顶层禁、函数体内允许惰性导入），该文件用例数从 19 条增至 24 条（补 5 条守卫自测：4 条参数化顶层违规 + 1 条函数体内放行），守卫不再可能退化成空检查。`41e18a2` 时点 `pytest tests` 报 `1416 passed, 2 skipped`，较开工前的 `1397 passed, 2 skipped` 多 `19` 条 = `tests/test_robot_dry_run_example.py` 14 条 + 契约 C 新增守卫自测 5 条。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。

该文件的用例在 `6bd45f8` 扩到 23 条（读数行正则取数、计数代理真下发一次的反例自证、`RobotTransport` 协议守卫、子进程环境隔离守卫），`tests/test_examples_and_configs.py` 同期从 24 条增至 32 条（5 条守卫自测扩成 10 条模块 import 期作用域违规参数 + 3 条函数体内放行参数，净 +8）。落点 `6bd45f8` 上 `pytest tests` 报 `1433 passed, 2 skipped`（`1416 + 17`）：跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。

`01a2d52` 时点按审查纠了 `examples/robot_dry_run.py` 的描述与真机路径：文件头不再把零发送归到 `control/api.py:519-520` 与 `control/controller.py:169-172`，改成实际生效的路径——命令在 priming 阶段的 adapter 门禁（`control/api.py:860-862`）被 `MotionNotEnabled` 拒绝、根本没进状态机（`_prime_state_machine()` 于 `api.py:513` 先于 `state.handle()`），另两处是 `api.py:674-675` 与 `api.py:913-914` 的 dry-run 分支；厂商 import 失败的跳过分支收紧到「确属 `lerobot` 缺件」（按 `ImportError.name` 判定，不命中就原样 `raise`），不再把 `zmq` 之类真故障伪装成「需要 vendor 环境」退 `0`；真机路径改为先 `prime_observation_request_window(include_cameras=False)` 再 `get_observation(include_cameras=False)`，避免开场图像突发与远程链路上的相机帧；结尾契约句改成 `期望 0）`，`0 次发送` 只能由实测计数插值产生。`01a2d52` 时点 `pytest tests` 报 `1415 passed, 1 failed, 2 skipped`：唯一红的是 `tests/test_robot_dry_run_example.py::test_example_print_contains_zero_send_literal`——该守卫要求 `0 次发送` 以源码常量出现，与契约句「只由真计数插值」直接冲突。该用例已随 `6bd45f8` 换成运行时守卫（读数行正则取数 + 计数代理真下发一次的反例自证 + AST 插值守卫），`test_example_print_contains_zero_send_literal` 在 HEAD 不再存在；落点见上段的 `1433 passed, 2 skipped`，工作树是绿的。

`6bd45f8` 之后又按实测改了三处口径（未发版，仍属 v0.7.0）。一、承认「只读」只约束下发：`examples/robot_dry_run.py` 与 `docs/library-guide.md` 写明脚本自身两次取帧走 `include_cameras=False`，但库内 `LiftHandle.height()`（`control/api.py:494`）与 `_prime_state_machine()` 的首帧（`control/api.py:845`）调的是无参 `get_observation()`，落回厂商默认 `include_cameras=True`（`lerobot/robots/alohamini/alohamini_client.py:535`），真机分支仍会多发 3 次相机帧请求（`height_before`、`move_by` 内部的 `height()`、priming 首帧）——离线假 client 实录的读帧序列是 `prime(cameras=False)` 之后连着 3 次不带关键字的 `get_observation()`；要真正 state-only 需改库侧调用签名，本轮不做。二、命令步的异常归因收窄到 `MotionNotEnabled`：`height_before = session.lift.height()` 在命令下发之前执行，首帧未到时它抛的是 `FeedbackLost`（`observation missing 'lift_axis.height_mm'`），那时命令还没下发，归因成「门禁拒绝」是错的；其余三类只报类型与消息。三、跳过判定改成决定性判定：不再按 `ImportError.name` / `str(exc)` 猜，只有 `importlib.util.find_spec("lerobot") is None` 才走跳过分支，`lerobot` 装了但符号改名或缺失时原样抛出（`find_spec` 遇半初始化模块抛 `ValueError` 时按「不可判」走抛出）；同时结尾的零发送门禁在 `assert` 之外补一道显式 `SystemExit`（`python -O` 下 `assert` 会被整条删掉），读数行移到门禁之前打印。行号概括也补全成「`adapter`、`ik_factory`、`clock`、`sleeper` 四者齐备才构造控制器（`control/api.py:653-672`）」与「`alohamini_client.py:160-169`」（`zmq_context.term()` 在 168、`raise` 在 169）。该时点即 `b46f33f`：本轮不改测试，`pytest tests` 仍是 `1433 passed, 2 skipped`；跳过仍是那两个 parity 文件缺 `lerobot`，`2` 不等于全部通过。

`00fa714` 给指纹 JSON 加 `lerobot_version` 字段：取值优先 `--lerobot-version` 原样传入，未传时读本机 `importlib.metadata.version("lerobot")`，两处都取不到记显式 `null`；`--lerobot-version` 传空串按用法错误退 `2`（空串不是版本，而 `null` 的语义是「取不到」，两者不得混读）。要这一列是因为采集在 Mac 侧、Pi 上跑的是 vendor 快照，缺了它这份 JSON 说不清对应哪个 `lerobot`。现场首次实采值 `lerobot_version="0.6.1"`，与手册 4.1b 读到的 Pi 侧版本逐字一致，产物 `capture/fingerprint-20261105T063600Z.json`（`docs/field-results.md` 2026-11-05 节）。

补记 tracking 模式的旁挂文件（取舍见 [ADR.md](ADR.md) D18）：`--mode tracking` 除 14 列主 CSV 之外，在同目录写一份 `<主CSV>-safety.csv`——9 列长表，一帧一电机一行，列序固定为 `step_index`、`sample_sequence`、`t_monotonic_s`、`motor`、`current_ma`、`requested_target`、`accepted_target`、`hold_active`、`target_source`。降级口径：客户端缺 `latest_safety_status`（能力缺失）或该面内容不可用（整轮计数，点名坏帧数）时，退出码不变、sidecar 照写，缺值写 `None`（空单元格），不补 `0`——`0 mA` 读起来像测到的零电流，是伪观测。`--mode rtt` 不产生 sidecar。

`c6c09ca` 补的是零发送守卫自身的「可达性自证」（只动测试，未发版）：AST 零发送断言守卫补上可达性自证——`if False:`、`if 1 == 0:`、`while False:` 这类死分支不展开，守卫不能靠一段永不执行的代码蒙过；补显式 `SystemExit` 兜住 `python -O`（该模式下 `assert` 被整条删掉）；子进程隔离取证改成核对 `subprocess.run` 实收的 `env`，而不只看是否传了参数；示例函数体内惰性 import 的白名单守卫同步加固。用例数：`tests/test_robot_dry_run_example.py` 23 → 37 条、`tests/test_examples_and_configs.py` 32 → 37 条（合计 +19）。该时点 `pytest tests` 报 `1452 passed, 2 skipped`（`1433 + 19`）；跳过仍是那两个 parity 文件缺 `lerobot`，`2` 不等于全部通过。

`777f8e9` 新增一条用例把真机分支的读帧序列钉死：预热与本脚本自身那次取帧都带 `include_cameras=False`，而库内 `LiftHandle.height()`（`control/api.py:494`）与 `_prime_state_machine()`（`control/api.py:845`）的无参 `get_observation()` 落回厂商默认（`lerobot/robots/alohamini/alohamini_client.py:535`），因而真机分支上的相机帧请求**恰好 3 次**——这个数字由用例钉住，改动库侧调用签名会先让它变红。用例数 `tests/test_robot_dry_run_example.py` 37 → 38 条。落点 `pytest tests` 报 `1453 passed, 2 skipped`（`1452 + 1`）；跳过仍是那两个 parity 文件缺 `lerobot`，`2` 不等于全部通过，工作树是绿的。

parity 夹具对齐 vendor `6aee1f81` 的不变式：`tests/test_control_api_lerobot_parity.py` / `tests/test_kinematics_lerobot_parity.py` 在 fork `6aee1f81` 快照上 **61/61 通过**（`aba8428` 修复夹具后）。口径澄清（2026-11-05 实测）：从 PyPI 装的**上游 wheel `lerobot 0.6.1` 不含 `alohamini` 模块**，所以本库的差分对照只能对 **vendor fork（`liyiteng/lerobot_alohamini`）** 成立，正文中「对上游 lerobot 的差分对照」一律按 vendor fork 读。主环境无 `lerobot` 时这两个文件整体跳过（`2 skipped`），跳过不等于差分对照通过。

把发版纪律写成证据红线（取舍与代价见 [ADR.md](ADR.md) D19）：V01 不收口不发 minor 版（`0.7.x → 0.8` 要有 V01 的实机证据）；每个发版至少新增一格实机证据并写明条件（供电 / 温度 / 容差），纯文档 / 测试修复的发版可豁免，但豁免理由必须写进 changelog；V01 收口前新增测试只走追因路径，不做 polish。要这条是因为「离线全绿」容易被当成发版依据——它只是软件侧快照，`2 skipped` 的两条 parity 差分对照并未跑过，实机侧 13 条里当时仍有 9 条「未取」。

把 V01 残余支的分级口径与达成率分母统一写进 [ADR.md](ADR.md) D20：步末残余按关节类别分级——肩类（`shoulder_pan` / `shoulder_lift`）门槛改为 `≤2°`（≈22.8 tick），轻载轴维持 `≤2 tick（0.1758°）`；达成率主分母统一为 `abs(--delta-deg) × --steps`，不再用 `Σ(cmd_deg − baseline_deg)`（`--anchor command` 下逐条 `cmd_deg − baseline_deg` 含跨步累积 lead，求和会把同一段行程记两遍以上）。要这两条是因为 2026-11-08 路 A 两轮排除「动作慢 / 跟不上」后，同一份数据用两个分母分别算出 `43.0108%` 与 `79.1209%`，跨场次、跨跑法的读数不可比；`2°` 这个量级也不是本库自造——厂商 `_JOINT_STALL_MIN_COMMAND_ERROR_DEG = 2.0` 说明它落在固件默认能容忍的残余里。

## v0.6.0

v0.6.0 修正了失败原因分类：`ArmIK.solve()` 结尾不再把「没到位」一律报 `JOINT_LIMIT`，而是按越限与否分流——解越出注入限位（越限判定仍含 `LIMIT_EPS_DEG` 容差）报 `JOINT_LIMIT`，未越限但位置残差超过 `profile.residual_limit_m` 报 `RESIDUAL`，两者同时成立时 `JOINT_LIMIT` 优先。`FAILURE_REASONS` 不新增枚举值，`IKResult` 字段不变。

对外行为变更一处：一部分失败的 `reason` 从 `JOINT_LIMIT` 变为 `RESIDUAL`，经控制层 `getattr(ControlError, result.reason, ...)` 映射后对外错误码同步从 `ControlError.JOINT_LIMIT` 变为 `ControlError.RESIDUAL`，按错误码分流的调用方需要调整。版本号从 `0.5.0` 升到 `0.6.0`。取舍与代价见 [ADR.md](ADR.md) D11。

本轮 `pytest tests` 报 `1031 passed, 2 skipped`。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。新增 4 条用例落在 `tests/test_arm_ik.py` 的 `T-IK-19.x` 分类矩阵（在限位内/外 × 残差达标/超标逐格），红阶段先失败、改完实现后转绿。

## v0.5.0

v0.5.0 给 `ArmIK.solve()` 的关节限位加了入口校验：两侧必须是有限实数、必须是 `(lo, hi)` 二元序列，排除 `bool` 与 `memoryview` 这类伪数字，并且必须满足 `lo < hi`，违规抛 `ValueError` 并点名关节与两侧取值。改动前，倒置限位会让 placo 抛裸 `QPError`，退化限位会被标成 `JOINT_LIMIT`，两者都不是配置错误该有的出口。多余关节键定为忽略，由用例钉住。`solve()` 的签名与 `IKResult` 的返回类型未变。版本号从 `0.4.0` 升到 `0.5.0`。对外行为变更一处：非法限位从 `RuntimeError` 或静默通过变成 `ValueError`，按异常类型分流的调用方需要调整。取舍与代价见 [ADR.md](ADR.md) D10。

本轮 `pytest tests` 报 `1027 passed, 2 skipped`。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。新增 28 条用例全部落在 `tests/test_arm_ik.py` 的 `T-IK-18.x` 系列，红阶段先失败、改完实现后转绿。

## v0.4.0

v0.4.0 把旧仓库 `benchmarks/alohamini_io` 的两个现场基准脚本与配套测试一并迁入：`benchmarks/alohamini_io/capture_robot_fingerprint.py`（473 行）与 `benchmarks/alohamini_io/measure_latency.py`（2151 行），配套测试 `tests/benchmarks/test_capture_robot_fingerprint.py`（638 行，静态 20 个测试函数）与 `tests/benchmarks/test_measure_latency.py`（3365 行，静态 110 个测试函数）。版本号从 `0.3.0` 升到 `0.4.0`。迁入原则是语义零改动：四个文件与旧仓库版本 `ast.dump` 逐一相等（4/4），字节不同处只是 `ruff format` 的折行（旧仓库 line-length 110，本库 100）。两个脚本对 `lerobot` 的 import 全在 `default_client_factory()` 函数体内（`capture_robot_fingerprint.py:140-141`、`measure_latency.py:1314-1315`），模块顶层只有标准库与 numpy；配套测试用依赖注入的 fake client 与 fake clock，不碰 `lerobot`、`zmq`、`socket`、线程。为让这四份文件过本库风格门，`pyproject.toml` 尾部按四条精确路径新增 `[tool.ruff.lint.per-file-ignores]`，全局 `select` / `ignore` 未改。取舍见 [ADR.md](ADR.md) D9。

v0.4.0 轮次 `pytest tests` 结果（本机实测，命令 `uv run --no-sync python -m pytest tests -o addopts="" -p no:cacheprovider`）：`999 passed, 2 skipped`。迁移前是 `823 passed, 2 skipped`，增量 176 条 = 23 条（capture 测试参数化展开）+ 153 条（measure 测试参数化展开）。跳过仍是那两个 parity 文件（缺 `lerobot`）。`tests/benchmarks` 子集 190 条通过 = 14（`test_benchmark_arm_ik.py`）+ 23 + 153。`ruff check .` 报 `All checks passed!`，`ruff format --check .` 报 `34 files already formatted`。

## v0.3.0

v0.3.0 做三件事：把 `placo` 声明为 `kinematics` extra（`placo>=0.10.1,<0.11`）并装进开发环境；迁入离线 IK 基准 `benchmarks/alohamini_ik/benchmark_arm_ik.py`；给 IK 判限位加容差，消除边界分类漂移。本轮 `pytest tests` 实跑 825 条，823 条通过、2 条跳过（两个 `lerobot` parity 文件在模块级跳过，收集阶段不计入）。

`benchmarks/alohamini_ik/benchmark_arm_ik.py` 与配套测试 `tests/benchmarks/test_benchmark_arm_ik.py` 已迁入，离线可跑。开发环境已按 `kinematics` extra 装上 `placo`，离线 IK 基准与 IK 数值用例真跑，不再整模块跳过。

## v0.2.0

v0.2.0 把控制层与运动学搬入库内：`control/`（会话与句柄、命令状态机、单臂控制器、`arm_ik`）、`kinematics/`（型号规格表、关节单位适配、`assets/am_arm200/kinematics.urdf`）。版本号从 `0.1.0` 升到 `0.2.0`，核心依赖从空改为 `numpy>=1.26`。行为变更一处：`connect()` 不再在内部 import `lerobot`，未注入 `client_factory` 时抛 `RuntimeUnavailable`。
