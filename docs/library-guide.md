# 库使用与接口说明

这份文件写给要用这个库的人，讲接口与行为。决策理由在 `docs/ADR.md`，现场操作在 `docs/s4-field-smoke.md`。

## 1 配置

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

## 2 能力探测

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

## 3 控制层与运动学

`control/` 是会话级控制 SDK。`connect(...)` 打开一次会话，返回 `AlohaMiniControl`；由这个会话取左右臂、升降、底盘、夹爪的 handle，再由 handle 发命令。命令走状态机，非法组合被拒绝，异常带原因。

`control/__init__.py` 导出：

- 会话与句柄：`AlohaMiniControl`、`ArmHandle`、`BaseHandle`、`GripperHandle`、`LiftHandle`、`connect`
- 结果与诊断：`MoveReport`、`StepDiagnostic`
- 异常：`FeedbackLost`、`MotionBlocked`、`MotionNotEnabled`、`MotionTimeout`、`OutOfRange`、`RuntimeUnavailable`

`kinematics/model_specs.py` 给出整机型号表 `ROBOT_SPECS` 与关节名表 `ARM_PROFILE_JOINTS`，并提供 `validate_robot_model` 与 `arm_state_keys`。`kinematics/joint_adapter.py` 的 `JointKinematicAdapter` 在原始电机刻度与运动学关节角之间双向换算，语义对齐 `MotorsBus` 的 `_normalize` / `_unnormalize`。

`control/arm_ik.py` 做 FK 与数值 IK。`placo` 是可选 extra `kinematics` 的依赖，函数内惰性 import，见 `docs/ADR.md` D8。

`control/api.py` 不含默认运行时：`connect` 未注入 `client_factory` 时抛 `RuntimeUnavailable`。真实连接由调用方自己构造运行时 client 传进来；参考实现见 `examples/robot_dry_run.py` 的 `build_client_factory()`——它返回一个 `(host, **kwargs) -> client` 的工厂，厂商 `AlohaMiniClient` 与 `AlohaMiniClientConfig` 的 import 落在最内层工厂函数体内（惰性导入，主环境没有 `lerobot` 也能 import 本模块），工厂再经 `connect(..., client_factory=...)` 注入。

那份示例走的是 `dry_run=True`：读一次真观测、对升降轴下一条命令、断开，`send_action` 一次都不会被调用。示例**不注入 `JointKinematicAdapter`（也不建 `placo` IK）**，所以命令在 priming 阶段的 adapter 门禁就被拒：`control/api.py:860-862` 在 `session._prime_state_machine()` 里抛 `MotionNotEnabled("motion requires a kinematic adapter; ...")`，而该调用位于 `control/api.py:513`，先于 `state.handle()`——命令读完第一帧反馈就被拒，根本没有进入状态机。另两处生效的门禁是 `control/api.py:674-675`（`connect(dry_run=True)` 连初始保持都不发）与 `control/api.py:913-914`（`disconnect()` 的底座零速只在非 dry-run 下发）。`control/api.py:519-520`（升降轴到位后的下发门禁）与 `control/controller.py:169-172`（臂控制器 `_send()` 的 dry-run 早退）在本示例里**不可达 / 不适用**——前者位于 `state.handle()` 之后，无 adapter 时前面已经抛错走不到；后者属于臂控制器，而臂控制器在 `adapter`、`ik_factory`、`clock`、`sleeper` 四者齐备时才构造（构造条件见 `control/api.py:653-672`），本示例不建控制器。结尾先打印计数代理的实测调用次数，再做显式零发送门禁（`assert` 之外另有一道 `SystemExit`，`python -O` 下不会消失），不是硬编码的 0。

真机跑法（两条命令，`--host` 是机器人 Pi 的 IP/主机名；在 Pi 本机上跑可省，默认 `127.0.0.1`）：

```bash
export PYTHONPATH=<vendor fork>/src:src
.venvs/alohamini-lerobot/bin/python examples/robot_dry_run.py --host <Pi 的 IP>
```

两行都按本机环境替换：解释器路径与 `.venvs/` 目录只在本机成立、不入库（`.gitignore` 已忽略）；`--host` 填机器人 Pi 的 IP/主机名（现场记录过 `10.34.170.98`，跑通后可用 `pi5.local`，见 [s4-field-smoke.md](s4-field-smoke.md)）。

第一行末尾的 `:src` 不能省：`alohamini_cartesian` 没装进 vendor 环境，靠 `PYTHONPATH` 带上本仓库的 `src`。没有 vendor 环境时直接跑本示例，它会打印「需要 vendor 环境」的跳过文案并以退出码 `0` 结束，不连设备、不联网。那份跳过分支只看环境（`importlib.util.find_spec("lerobot")` 为 `None`），`lerobot` 装了但符号改名/缺失时原样抛出导入错误，不退 `0`。

但「只读」只约束下发，不约束观测请求：示例自身的两次取帧都走 `include_cameras=False`，而库内 `LiftHandle.height()`（`control/api.py:494`）与 `_prime_state_machine()` 的首帧（`control/api.py:845`）调的是无参 `get_observation()`，落回厂商默认 `include_cameras=True`（`lerobot/robots/alohamini/alohamini_client.py:535`），所以真机分支仍会多发 3 次相机帧请求；要做到真正 state-only 需改库侧调用签名，当前不做。

## 4 公开 API

- `alohamini_cartesian.adapter.protocol`：`RobotTransport`（`get_observation`、`send_action`、`prime_observation_request_window(*, include_cameras)`）、`RobotSpec`（`robot_model`、`left_port`、`right_port`），两个 `typing.Protocol`。
- `alohamini_cartesian.capability.detect`：常量 `SPAN_BY_NORMALIZATION`、`SUPPORTED_METADATA_SCHEMAS`；异常 `CapabilityError`、`ConversionUnavailable`；数据类 `MotorInfo`、`Capabilities`、`Conversion`；函数 `detect_from_observation`、`resolve_deg_per_unit`。
- `alohamini_cartesian.config.overlay`：常量 `ENV_PREFIX`；异常 `ConfigError`；数据类 `AlohaMiniSettings`；函数 `load_settings`。
- `alohamini_cartesian.control`：会话 `AlohaMiniControl`，句柄 `ArmHandle`、`BaseHandle`、`GripperHandle`、`LiftHandle`，入口 `connect`，结果 `MoveReport`、`StepDiagnostic`，异常 `FeedbackLost`、`MotionBlocked`、`MotionNotEnabled`、`MotionTimeout`、`OutOfRange`、`RuntimeUnavailable`。
- `alohamini_cartesian.kinematics.model_specs`：常量 `ROBOT_SPECS`、`ARM_PROFILE_JOINTS`；函数 `validate_robot_model`、`arm_state_keys`、`arm_state_keys_for_robot_model`。
- `alohamini_cartesian.kinematics.joint_adapter`：类 `JointKinematicAdapter`，方法 `observation_to_kinematic_deg`、`kinematic_deg_to_action`、`effective_limits_deg`、`sign`。

## 5 约束与机器守卫

两条硬约束，改代码时先看这里。

1. `adapter/protocol.py` 之外禁止 import `lerobot`。`lerobot` 只能通过 `RobotTransport` 与 `RobotSpec` 两个 `Protocol` 接触，其余模块不碰 `lerobot.motors`、`lerobot.robots.alohamini` 这类深层模块。
2. 能力探测不得把版本号当兼容判据。判据是这项能力是否存在，`schema_version` 只用来决定字段怎么解析。

两条机器守卫，不靠人工 review 维持。

- `tests/test_capability.py` 用 AST 遍历断言 `src/` 内除 `adapter/protocol.py` 之外没有任何模块 import `lerobot`，并用 `sys.modules` 前后快照断言导入 `capability.detect`、`config.overlay` 这两个动作本身不会引入 `lerobot`。
- `tests/test_control_commands.py` 断言 `control/` 下除 `api.py` 之外没有模块 import 设备适配层、客户端或主机模块。它带正反两条控制用例：一条证明守卫能命中相对 import 的适配层，一条证明合法 import 不会被误判。
