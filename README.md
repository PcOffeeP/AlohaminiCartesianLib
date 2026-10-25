# AlohaMini Cartesian

AlohaMini 笛卡尔遥操作的产品侧库：对机器人运行时的适配协议、能力探测与单位换算裁决、配置覆盖，以及控制层与运动学。核心只依赖 numpy。

## 为什么独立成库

旧仓库把产品代码与机器人运行时放在同一仓同一分支上。库的版本号跟着 vendor 分支漂移，产品改动与实机运行时版本互相绑死，实机换一次版本就得连产品一起动。

现场改动只能落进会被 vendor 更新的文件里。上游一覆盖，现场配置和补丁就没了；旧结构里没有 overlay 表达差异，差异只能直接改源码。

验证工具还把 metadata 的 `schema_version` 当成兼容判据。旧仓库 `benchmarks/alohamini_io/measure_latency.py` 的运动门禁只接受 `schema_version == 2`，遇到 schema 1 或字符串 `"2"` 直接拒绝，工具被版本号拦死，而真正该判的是这项能力在不在。

## 与旧仓库的分工

本库与旧仓库是两个项目：本库是产品侧的库，旧仓库是厂商运行时的 fork（从上游仓库分出来的副本）。

| 项目 | 定位 | 包含什么 | 与本库的关系 |
| --- | --- | --- | --- |
| 本库 `alohamini-cartesian-lib` | 产品侧独立 Python 库，只跑在 PC 一侧 | 把「末端目标 → 六关节角 → 指令」翻译成协议动作发给 Host；不含厂商代码；测试离线、不连设备；唯一接触面是 `adapter/protocol.py` | 本库自身 |
| 旧仓库 `/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini-cartesian-teleop` | 厂商运行时的 fork | `src/lerobot`，版本 0.6.1，remote 指向 `liyiteng/lerobot_alohamini`，当前分支 `codex/alohamini-cartesian-teleop`；该路径是 `lerobot_alohamini` 仓库的 worktree；含 Host 进程、电机总线、相机、录制、ROS 兼容；实机运行的唯一已验证路径 | 本库不修改它，只作只读参考 |

改产品逻辑进本库，改 vendor 运行时进旧仓库上游，两边不混。

## 定案一览

| 编号 | 定案 | 要点 |
| --- | --- | --- |
| D1 | 依赖形态 | 独立包发布，`lerobot` 作为可选 `extra`（`runtime`），不使用 git submodule |
| D2 | 配置格式 | YAML 单文件加环境变量覆盖，顺序为默认值 → 模板 → 现场文件 → 环境变量 |
| D3 | 迁移范围 | 只搬 `control/`、运动学（`joint_kinematic_adapter`、`model_specs`）与 `benchmarks/`，分阶段搬，v0.1.0 不含 |
| D4 | 启动方式 | 新开仓库，当前状态作为 `v0.1.0` 起点，旧仓库冻结为只读参考 |
| D5 | 迁移范围与适配边界 | 控制层与运动学入库内，控制器不直连 `lerobot`，电机语义改为库内镜像 |
| D6 | numpy 依赖 | 核心不再零依赖，只加 numpy；不引入 torch/scipy，`placo` 的依赖形态见 D8 |
| D7 | 迁移边界 | 厂商侧协议测试不迁入，本库对厂商内部协议不负回归责任 |
| D8 | `placo` 依赖形态 | 声明 `kinematics` extra（`placo>=0.10.1,<0.11`），接受 numpy 落到 2.3.x |
| D9 | 现场基准脚本迁移时机 | `benchmarks/alohamini_io` 两脚本与配套测试在 v0.4.0 迁入，不等 S4；语义零改动，`lerobot` 保持函数体内惰性导入 |
| D10 | 关节限位取值在 `ArmIK.solve()` 入口校验 | 违规抛 `ValueError`，多余键忽略；`±inf` 不放行（默认路径撞不上），`memoryview` 已排除 |

每条的背景、决定与代价见 [docs/ADR.md](docs/ADR.md)。

## 安装

核心只依赖 numpy；不需要机器人，不需要 `lerobot`。

```bash
pip install -e .
```

需要读 YAML 配置文件时加 `yaml` extra：

```bash
pip install -e ".[yaml]"
```

需要跑数值 IK 用例与离线 IK 基准时加 `kinematics` extra：

```bash
pip install -e ".[kinematics]"
```

这个 extra 会把 numpy 带到 2.3.x：`placo` 依赖 `pin==3.8.0`，它拉来的 `cmeel-boost~=1.89.0` 要求 numpy 2.3.x，详见 [docs/ADR.md](docs/ADR.md) D8。

需要接触真实运行时再加 `runtime` extra：

```bash
pip install -e ".[runtime]"
```

注意 `lerobot` 当前不在 PyPI 上发布。使用 `runtime` extra 之前，先按 `lerobot` 上游 README 把 `lerobot` 装进同一环境，例如从上游仓库以 editable 方式安装。本库的 `pyproject.toml` 只声明 `runtime = ["lerobot"]`，不指向任何可解析的分发地址。

开发依赖用 PEP 735 的 `[dependency-groups] dev`，包含 `pytest>=8.0`、`ruff>=0.6`、`PyYAML>=6.0`。这些工具放在 dependency group 里，不进入包的 `extras` 元数据，安装本库的用户不会被带上 ruff 和 pytest。两种等价装法：

```bash
uv sync                                    # 自动装 dev 组
pip install -e . pytest ruff pyyaml        # 不用 uv 时手动列出
```

## 配置

覆盖顺序：默认值 → 模板文件 → 现场文件 → 环境变量。后者覆盖前者。

`cameras` 是例外，它在模板、现场文件、环境变量三层之间按键合并：同名键以后一层为准，各层独有的键保留；其余字段一律整值覆盖。

```
模板 cameras: {wrist: /dev/video0, top: /dev/video1}
现场 cameras: {wrist: /dev/video9}
```

这两层合并后是 `{wrist: /dev/video9, top: /dev/video1}`。

文件格式以 YAML 为主，需要 `yaml` extra 里的 PyYAML；`.json` 后缀走标准库，作为不装 PyYAML 时的退路。按文件后缀分派，其他后缀抛 `ConfigError`。

校验是严格的，不做静默纠正：

- 未知键抛 `ConfigError`，不静默忽略。
- 整数字段类型不符抛 `ConfigError`，字符串和 bool 不静默转 `int`。
- `observation_request_window` 小于 1 抛 `ConfigError`；`watchdog_timeout_ms`、`max_loop_freq_hz` 同样不小于 1。
- `robot_model`、`left_port`、`right_port` 不得为空串。

环境变量前缀 `ALOHAMINI_`，键名大小写不敏感。

| 环境变量 | 对应字段 |
| --- | --- |
| `ALOHAMINI_ROBOT_MODEL` | `robot_model` |
| `ALOHAMINI_LEFT_PORT` | `left_port` |
| `ALOHAMINI_RIGHT_PORT` | `right_port` |
| `ALOHAMINI_OBSERVATION_REQUEST_WINDOW` | `observation_request_window` |
| `ALOHAMINI_WATCHDOG_TIMEOUT_MS` | `watchdog_timeout_ms` |
| `ALOHAMINI_MAX_LOOP_FREQ_HZ` | `max_loop_freq_hz` |
| `ALOHAMINI_CAMERAS_<NAME>` | `cameras[<name>]`，与文件里的 `cameras` 合并 |

`ALOHAMINI_CAMERAS_<NAME>` 同样按键合并：文件里已设的相机保留，环境变量里的同名相机覆盖。未知的 `ALOHAMINI_*` 键抛 `ConfigError`。

```python
from pathlib import Path

from alohamini_cartesian.config.overlay import load_settings

settings = load_settings(
    template=Path("configs/template.yaml"),
    site=Path("configs/site.yaml"),
    check_devices=True,
)
```

`load_settings(..., check_devices=True)` 会校验 `left_port`、`right_port` 与 `/dev/` 开头的相机路径是否存在。默认 `False`，不检查。

## 能力探测

运行时把额外信息放在观测的三个侧信道键里：`_robot_metadata`、`_control_status`、`_host_timing`。

`_robot_metadata.schema_version` 接受整数 1 和 2。缺失，或不是整数（`True`、`1.0`、`"1"`、`None`），一律抛 `CapabilityError`，fail-closed；其他整数（例如 3）同样拒绝。

schema 2 发布每个电机的 8 个字段。schema 1 缺 `homing_offset` 与 `resolution`，这两个字段填 `None`，缺失情况记在 `notes` 里。缺失字段一律填 `None`，不填 0 或 4096 冒充。

`_control_status` 与 `_host_timing` 缺失不抛异常，只把 `has_control_status`、`has_host_timing` 置为 `False`，降级由调用方决定。

单位换算按 `normalization` 取 span：

| normalization | span | 说明 |
| --- | --- | --- |
| `range_m100_100` | 200 | 臂关节默认；normalization 缺失时按此兼容 |
| `range_0_100` | 100 | gripper 等 0 到 100 区间 |
| `degrees` | 不参与公式 | 线上值已是角度，`deg_per_unit` 恒为 1.0 |

非 `degrees` 时用这条公式：

```
deg_per_unit = ((range_max - range_min) / span) * 360 / (resolution - 1)
```

换算有三个来源，优先级为 `metadata` → `calibration-file`（现场标定文件提供 range）→ `model-table`（型号分辨率表提供 resolution）。

`Conversion.source` 表示换算最终回落到哪一级。混合来源（例如 range 由标定文件补齐、resolution 由型号表补齐）时报最弱的那一级，上例为 `calibration-file`。

非 `degrees` 模式下，三源都不足以给出 range 或 resolution 时抛 `ConversionUnavailable`，异常信息指明缺哪一项。

`degrees` 模式下线上值已经是角度，`deg_per_unit` 恒为 `1.0`。如果 metadata、标定文件、型号表都没有刻度信息，`Conversion` 的 `range_min`、`range_max`、`resolution` 回填惰性占位值 `0 / 0 / 1`；这三个值不代表刻度范围，下游不得当作真实标定使用。来源真实但区间倒置时，仍然抛 `ConversionUnavailable`。

```python
from alohamini_cartesian.capability.detect import (
    detect_from_observation,
    resolve_deg_per_unit,
)

caps = detect_from_observation(observation)
conversion = resolve_deg_per_unit(caps, "left_waist.pos", model_resolution={"sts3215": 4096})
deg_per_unit = conversion.deg_per_unit
```

## 控制层与运动学

`control/` 是会话级控制 SDK。`connect(...)` 打开一次会话，返回 `AlohaMiniControl`；由这个会话取左右臂、升降、底盘、夹爪的 handle，再由 handle 发命令。命令走状态机，非法组合被拒绝，异常带原因。

`control/__init__.py` 导出：

- 会话与句柄：`AlohaMiniControl`、`ArmHandle`、`BaseHandle`、`GripperHandle`、`LiftHandle`、`connect`
- 结果与诊断：`MoveReport`、`StepDiagnostic`
- 异常：`FeedbackLost`、`MotionBlocked`、`MotionNotEnabled`、`MotionTimeout`、`OutOfRange`、`RuntimeUnavailable`

`kinematics/model_specs.py` 给出整机型号表 `ROBOT_SPECS` 与关节名表 `ARM_PROFILE_JOINTS`，并提供 `validate_robot_model` 与 `arm_state_keys`。`kinematics/joint_adapter.py` 的 `JointKinematicAdapter` 在原始电机刻度与运动学关节角之间双向换算，语义对齐 `MotorsBus` 的 `_normalize` / `_unnormalize`。

`control/arm_ik.py` 做 FK 与数值 IK。`placo` 是可选 extra `kinematics` 的依赖，函数内惰性 import，见 D8。

`control/api.py` 不含默认运行时：`connect` 未注入 `client_factory` 时抛 `RuntimeUnavailable`。真实连接由调用方自己构造运行时 client 传进来。

## 公开 API

- `alohamini_cartesian.adapter.protocol`：`RobotTransport`（`get_observation`、`send_action`、`prime_observation_request_window(*, include_cameras)`）、`RobotSpec`（`robot_model`、`left_port`、`right_port`），两个 `typing.Protocol`。
- `alohamini_cartesian.capability.detect`：常量 `SPAN_BY_NORMALIZATION`、`SUPPORTED_METADATA_SCHEMAS`；异常 `CapabilityError`、`ConversionUnavailable`；数据类 `MotorInfo`、`Capabilities`、`Conversion`；函数 `detect_from_observation`、`resolve_deg_per_unit`。
- `alohamini_cartesian.config.overlay`：常量 `ENV_PREFIX`；异常 `ConfigError`；数据类 `AlohaMiniSettings`；函数 `load_settings`。
- `alohamini_cartesian.control`：会话 `AlohaMiniControl`，句柄 `ArmHandle`、`BaseHandle`、`GripperHandle`、`LiftHandle`，入口 `connect`，结果 `MoveReport`、`StepDiagnostic`，异常 `FeedbackLost`、`MotionBlocked`、`MotionNotEnabled`、`MotionTimeout`、`OutOfRange`、`RuntimeUnavailable`。
- `alohamini_cartesian.kinematics.model_specs`：常量 `ROBOT_SPECS`、`ARM_PROFILE_JOINTS`；函数 `validate_robot_model`、`arm_state_keys`、`arm_state_keys_for_robot_model`。
- `alohamini_cartesian.kinematics.joint_adapter`：类 `JointKinematicAdapter`，方法 `observation_to_kinematic_deg`、`kinematic_deg_to_action`、`effective_limits_deg`、`sign`。

## 纪律

两条硬约束，改代码时先看这里。

1. `adapter/protocol.py` 之外禁止 import `lerobot`。`lerobot` 只能通过 `RobotTransport` 与 `RobotSpec` 两个 `Protocol` 接触，其余模块不碰 `lerobot.motors`、`lerobot.robots.alohamini` 这类深层模块。
2. 能力探测不得把版本号当兼容判据。判据是这项能力是否存在，`schema_version` 只用来决定字段怎么解析。

两条机器守卫，不靠人工 review 维持。

- `tests/test_capability.py` 用 AST 遍历断言 `src/` 内除 `adapter/protocol.py` 之外没有任何模块 import `lerobot`，并用 `sys.modules` 前后快照断言导入 `capability.detect`、`config.overlay` 这两个动作本身不会引入 `lerobot`。
- `tests/test_control_commands.py` 断言 `control/` 下除 `api.py` 之外没有模块 import 设备适配层、客户端或主机模块。它带正反两条控制用例：一条证明守卫能命中相对 import 的适配层，一条证明合法 import 不会被误判。

## 当前状态与下一步

v0.2.0 把控制层与运动学搬入库内：`control/`（会话与句柄、命令状态机、单臂控制器、`arm_ik`）、`kinematics/`（型号规格表、关节单位适配、`assets/am_arm200/kinematics.urdf`）。版本号从 `0.1.0` 升到 `0.2.0`，核心依赖从空改为 `numpy>=1.26`。行为变更一处：`connect()` 不再在内部 import `lerobot`，未注入 `client_factory` 时抛 `RuntimeUnavailable`。

v0.3.0 做三件事：把 `placo` 声明为 `kinematics` extra（`placo>=0.10.1,<0.11`）并装进开发环境；迁入离线 IK 基准 `benchmarks/alohamini_ik/benchmark_arm_ik.py`；给 IK 判限位加容差，消除边界分类漂移。本轮 `pytest tests` 实跑 825 条，823 条通过、2 条跳过（两个 `lerobot` parity 文件在模块级跳过，收集阶段不计入）。

`benchmarks/alohamini_ik/benchmark_arm_ik.py` 与配套测试 `tests/benchmarks/test_benchmark_arm_ik.py` 已迁入，离线可跑。开发环境已按 `kinematics` extra 装上 `placo`，离线 IK 基准与 IK 数值用例真跑，不再整模块跳过。

v0.4.0 把旧仓库 `benchmarks/alohamini_io` 的两个现场基准脚本与配套测试一并迁入：`benchmarks/alohamini_io/capture_robot_fingerprint.py`（473 行）与 `benchmarks/alohamini_io/measure_latency.py`（2151 行），配套测试 `tests/benchmarks/test_capture_robot_fingerprint.py`（638 行，静态 20 个测试函数）与 `tests/benchmarks/test_measure_latency.py`（3365 行，静态 110 个测试函数）。版本号从 `0.3.0` 升到 `0.4.0`。迁入原则是语义零改动：四个文件与旧仓库版本 `ast.dump` 逐一相等（4/4），字节不同处只是 `ruff format` 的折行（旧仓库 line-length 110，本库 100）。两个脚本对 `lerobot` 的 import 全在 `default_client_factory()` 函数体内（`capture_robot_fingerprint.py:140-141`、`measure_latency.py:847-848`），模块顶层只有标准库与 numpy；配套测试用依赖注入的 fake client 与 fake clock，不碰 `lerobot`、`zmq`、`socket`、线程。为让这四份文件过本库风格门，`pyproject.toml` 尾部按四条精确路径新增 `[tool.ruff.lint.per-file-ignores]`，全局 `select` / `ignore` 未改。取舍见 [docs/ADR.md](docs/ADR.md) D9。

v0.4.0 轮次 `pytest tests` 结果（本机实测，命令 `uv run --no-sync python -m pytest tests -o addopts="" -p no:cacheprovider`）：`999 passed, 2 skipped`。迁移前是 `823 passed, 2 skipped`，增量 176 条 = 23 条（capture 测试参数化展开）+ 153 条（measure 测试参数化展开）。跳过仍是那两个 parity 文件（缺 `lerobot`）。`tests/benchmarks` 子集 190 条通过 = 14（`test_benchmark_arm_ik.py`）+ 23 + 153。`ruff check .` 报 `All checks passed!`，`ruff format --check .` 报 `34 files already formatted`。

v0.5.0 给 `ArmIK.solve()` 的关节限位加了入口校验：两侧必须是有限实数、必须是 `(lo, hi)` 二元序列，排除 `bool` 与 `memoryview` 这类伪数字，并且必须满足 `lo < hi`，违规抛 `ValueError` 并点名关节与两侧取值。改动前，倒置限位会让 placo 抛裸 `QPError`，退化限位会被标成 `JOINT_LIMIT`，两者都不是配置错误该有的出口。多余关节键定为忽略，由用例钉住。`solve()` 的签名与 `IKResult` 的返回类型未变。版本号从 `0.4.0` 升到 `0.5.0`。对外行为变更一处：非法限位从 `RuntimeError` 或静默通过变成 `ValueError`，按异常类型分流的调用方需要调整。取舍与代价见 [docs/ADR.md](docs/ADR.md) D10。

本轮 `pytest tests` 报 `1031 passed, 2 skipped`。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。新增 28 条用例全部落在 `tests/test_arm_ik.py` 的 `T-IK-18.x` 系列，红阶段先失败、改完实现后转绿。

v0.6.0 修正了失败原因分类：`ArmIK.solve()` 结尾不再把「没到位」一律报 `JOINT_LIMIT`，而是按越限与否分流——解越出注入限位（越限判定仍含 `LIMIT_EPS_DEG` 容差）报 `JOINT_LIMIT`，未越限但位置残差超过 `profile.residual_limit_m` 报 `RESIDUAL`，两者同时成立时 `JOINT_LIMIT` 优先。`FAILURE_REASONS` 不新增枚举值，`IKResult` 字段不变。

对外行为变更一处：一部分失败的 `reason` 从 `JOINT_LIMIT` 变为 `RESIDUAL`，经控制层 `getattr(ControlError, result.reason, ...)` 映射后对外错误码同步从 `ControlError.JOINT_LIMIT` 变为 `ControlError.RESIDUAL`，按错误码分流的调用方需要调整。版本号从 `0.5.0` 升到 `0.6.0`。取舍与代价见 [docs/ADR.md](docs/ADR.md) D11。

本轮 `pytest tests` 报 `1031 passed, 2 skipped`。跳过仍是那两个 parity 文件缺 `lerobot`；`2` 不等于全部通过。新增 4 条用例落在 `tests/test_arm_ik.py` 的 `T-IK-19.x` 分类矩阵（在限位内/外 × 残差达标/超标逐格），红阶段先失败、改完实现后转绿。

- S4（剩余）：只做真机侧的现场验证——两个脚本对着真机 Host 跑一遍，产出配置样例。脚本迁移与离线测试在 v0.4.0 已完成，不属 S4。前置条件、逐条命令与验收判据见 [docs/s4-field-smoke.md](docs/s4-field-smoke.md)。

## 已知限制

- 未装 `kinematics` extra 时，`tests/test_arm_ik.py` 的数值参数实例、`tests/test_control_controller.py` 的 2 条真 IK 用例与 `tests/benchmarks/test_benchmark_arm_ik.py` 因缺 `placo` 跳过。开发环境已装该 extra，本机这部分真跑。
- 缺 `lerobot` 时，`tests/test_kinematics_lerobot_parity.py` 与 `tests/test_control_api_lerobot_parity.py` 在收集期整体跳过。这两个文件用真实 `FeetechMotorsBus` 与 `AlohaMiniClient` 作 oracle 做差分对照，所以与真实总线的整环对照未在最小环境执行。
- `connect(...)` 必须注入 `client_factory`。默认工厂恒抛 `RuntimeUnavailable`，没有内部 import `lerobot` 的退路。
- `benchmarks/alohamini_io` 的两个脚本已入库，配套测试离线跑通 176 条用例，但从未对着真机 Host 跑过。缺三样证据：`schema_version == 2` 的实际取值没有样本；`measure_latency.py` 的 Mode B `tracking` 会真动机器人，未跑过；Gate F（运行时门禁，决定这次测量能不能发命令）的真实拒绝路径未跑过。现场怎么补齐这些证据见 [docs/s4-field-smoke.md](docs/s4-field-smoke.md)。

## 开发与测试

pytest 已配好 `testpaths`、`pythonpath = ["src"]` 与 `addopts = "-q"`。本包已以 editable 方式装进 `.venv`；`uv run --no-sync` 会直接复用这个 `.venv`，不装包、不改环境。

```bash
uv run --no-sync python -m pytest tests   # 自动测试
uv run --no-sync ruff check .             # 代码风格
```

本仓库没有 `uv.lock`，所以一律带 `--no-sync`：不带它的 `uv run` 会自行解析依赖并新写一个 `uv.lock`。`addopts = "-q"` 已经生效，自己再加 `-q` 会变成 `-qq` 并隐藏 `N passed` 汇总行；看不到汇总行时用 `-o addopts=""`。

当前 `pytest tests` 报 `1031 passed, 2 skipped`。跳过项只剩两个 parity 文件缺 `lerobot`，构成见「已知限制」。测试不依赖真实设备，不联网。

## 目录结构

```
alohamini-cartesian-lib/
├── pyproject.toml                    # 包元数据、extras、dev 组、pytest 与 ruff 配置
├── README.md
├── TODO.md                          # 待办清单，Agent 与人都要维护
├── .gitignore
├── benchmarks/alohamini_ik/benchmark_arm_ik.py           # FK 与数值 IK 基准脚本
├── benchmarks/alohamini_io/capture_robot_fingerprint.py  # 现场采集对端 metadata 指纹，lerobot 惰性 import
├── benchmarks/alohamini_io/measure_latency.py            # 现场测时延，Mode B tracking 会真动机器人
├── docs/ADR.md                       # D1–D11 决策记录
├── docs/s4-field-smoke.md            # S4 现场实施：命令、判据、产物、退出码
├── docs/archive/migration-backlog.md # 迁移待办，已归档（迁移在 v0.4.0 完成）
├── src/alohamini_cartesian/
│   ├── __init__.py                   # __version__ = "0.6.0"
│   ├── py.typed
│   ├── adapter/protocol.py           # 对 lerobot 的唯一接触面（typing.Protocol）
│   ├── capability/detect.py          # 能力探测与降级、单位换算裁决
│   ├── config/overlay.py             # YAML/JSON 加环境变量三级覆盖
│   ├── control/
│   │   ├── __init__.py               # 控制 SDK 导出面
│   │   ├── api.py                    # 会话与句柄、connect
│   │   ├── commands.py               # 控制状态机（无 I/O）
│   │   ├── controller.py             # 单臂笛卡尔控制器
│   │   └── arm_ik.py                 # FK 与数值 IK，placo 惰性可选
│   └── kinematics/
│       ├── __init__.py
│       ├── model_specs.py            # 型号规格表
│       ├── joint_adapter.py          # 电机刻度与关节角的单位适配
│       └── assets/am_arm200/kinematics.urdf
└── tests/
    ├── benchmarks/test_benchmark_arm_ik.py         # 基准测试，缺 placo 时整模块跳过
    ├── benchmarks/test_capture_robot_fingerprint.py  # 采集脚本测试，fake client，不碰 lerobot
    ├── benchmarks/test_measure_latency.py          # 测量脚本测试，fake client 与 fake clock，不碰 lerobot
    ├── test_arm_ik.py
    ├── test_capability.py
    ├── test_config_overlay.py
    ├── test_control_api.py
    ├── test_control_api_lerobot_parity.py
    ├── test_control_commands.py
    ├── test_control_controller.py
    ├── test_kinematics_adapter.py
    └── test_kinematics_lerobot_parity.py
```
