# 库使用与接口说明

这份文件写给要用这个库的人，讲接口与行为。决策理由在 `docs/ADR.md`，现场操作在 `docs/field-ops/00-opening.md`。

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

控制层每帧有安全门禁：关节步长门禁（默认 `2.0°`/周期）与领先夹紧（lead clamp，限制解相对反馈的超前量），两者属安全门禁与截断，不是帧序列平滑，也不按 metadata 版本切换。

速度流请用控制器持续 tick；**不要**每帧反复调 `move_by` 拼速度——`move_by` 每次锚定实测 FK，慢速下误差累积不足，重负载轴可能复现静摩擦失速（与 `measure_latency` 默认 `--anchor present` 的形态同构）。

`control/api.py` 不含默认运行时：`connect` 未注入 `client_factory` 时抛 `RuntimeUnavailable`。真实连接由调用方自己构造运行时 client 传进来；参考实现见 `examples/robot_dry_run.py` 的 `build_client_factory()`——它返回一个 `(host, **kwargs) -> client` 的工厂，厂商 `AlohaMiniClient` 与 `AlohaMiniClientConfig` 的 import 落在最内层工厂函数体内（惰性导入，主环境没有 `lerobot` 也能 import 本模块），工厂再经 `connect(..., client_factory=...)` 注入。

那份示例走的是 `dry_run=True`：读一次真观测、对升降轴下一条命令、断开，`send_action` 一次都不会被调用。无 adapter 的会话下运动命令按设计被拒——缺 kinematic adapter 时抛 `MotionNotEnabled("motion requires a kinematic adapter; ...")`，命令未进状态机；升降轴 dry-run 可以跑通（`dry_run=True` 下不下发）。结尾的零发送结论不是硬编码的 0，计数来自实测（`assert` 之外另有一道 `SystemExit`，`python -O` 下不会消失）。行级细节（哪几处 dry-run 分支生效、哪些门禁不可达）见示例文件头注释与 [examples/robot_dry_run.py](../examples/robot_dry_run.py)。

真机跑法（两条命令，`--host` 是机器人 Pi 的 IP/主机名；在 Pi 本机上跑可省，默认 `127.0.0.1`）：

```bash
export PYTHONPATH=<vendor fork>/src:src
.venvs/alohamini-lerobot/bin/python examples/robot_dry_run.py --host <Pi 的 IP>
```

两行都按本机环境替换：解释器路径与 `.venvs/` 目录只在本机成立、不入库（`.gitignore` 已忽略）；`--host` 填机器人 Pi 的 IP/主机名（现场记录过 `10.34.170.98`，跑通后可用 `pi5.local`，见 [field-ops/40-gates-params.md](field-ops/40-gates-params.md)）。

第一行末尾的 `:src` 不能省：`alohamini_cartesian` 没装进 vendor 环境，靠 `PYTHONPATH` 带上本仓库的 `src`。没有 vendor 环境时直接跑本示例，它会打印「需要 vendor 环境」的跳过文案并以退出码 `0` 结束，不连设备、不联网。那份跳过分支只看环境（`importlib.util.find_spec("lerobot")` 为 `None`），`lerobot` 装了但符号改名/缺失时原样抛出导入错误，不退 `0`。

但「只读」只约束下发，不约束观测请求：示例自身的两次取帧都走 `include_cameras=False`，而库内 `LiftHandle.height()`（`control/api.py:494`）与 `_prime_state_machine()` 的首帧（`control/api.py:845`）调的是无参 `get_observation()`，落回厂商默认 `include_cameras=True`（`lerobot/robots/alohamini/alohamini_client.py:535`），所以真机分支仍会多发 3 次相机帧请求；要做到真正 state-only 需改库侧调用签名，当前不做。

### 3.1 机器人模型（URDF）

本库用一份 URDF 做 FK/IK 的几何锚点，路径 `src/alohamini_cartesian/kinematics/assets/am_arm200/kinematics.urdf`。`control/arm_ik.py` 的 `ArmIK` 用它构造模型并算 FK/IK；读这份文件的是 `placo`（可选 extra `kinematics`，见 `docs/ADR.md` D8），不装该 extra 时运动学数值路径不可用。

**来源与 sha256 链**。该 URDF 派生自 GitHub `liyiteng/AM-ARM` 的 `am-arm200pro.urdf`，commit `5b03af1ef6c80595bd988e0137cf8e7c35811eb6`，上游文件 sha256 `c38669d46d2bf804c112a6d8ce926074c8ff65ba70917503e09b77c75e285030`。派生动作是剥离：去掉 inertial 属性、visual/collision 的 mesh 引用、不用的夹爪与相机关节和链接；保留完整 6-DOF 链，每个 link/joint 的 origin、axis 与 limit 原样留。

剥离 mesh 有一个直接后果：`placo` 模型里没有碰撞几何体，因此没有自碰撞检查，也没有桌面与工装约束。IK 解不保证末端路径不与自身或桌面相撞，碰撞判断由调用方负责。

**溯源锚点**。上游原件、`sources.json` 与 Apache-2.0 许可证副本存于退役仓库的只读目录（本库不复制该目录）：

```
/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini-cartesian-teleop/docs/alohamini-control/reference/urdf-sources/
```

核 sha256 时以该目录下的 `am-arm200pro.urdf` 为准。本库当前没有任何 `LICENSE` / `NOTICE` 文件，即这份派生 URDF 随包分发、上游许可证与署名未随包附上；合规待处理，见 `TODO.md` 的许可债行。

**关键杆长**。URDF 内的长度单位是米，下表「值」列前半照抄 URDF、后半是换算值，换算到毫米保留两位小数。

| 量 | URDF 出处 | 值 |
| --- | --- | --- |
| 肩高 | `left_shoulder_pan` origin 的 z | `0.062556` m = `62.56` mm |
| 肩俯仰偏置 | `left_shoulder_lift` origin | `(0, -0.029035, 0.058223)` m |
| 大臂 | `left_elbow_flex` origin 的 xy 模长（`√(0.041170749² + 0.128514359²)`） | `0.13495` m = `134.95` mm |
| 小臂 | `left_wrist_flex` origin 的 xy 模长（`√(0.003204703² + 0.198428485²)`） | `0.19845` m = `198.45` mm |
| 腕偏置 1 | `left_wrist_yaw_joint` origin 的模长（`(-0.000916, -0.028130, -0.000966)`） | `0.02816` m = `28.16` mm |
| 腕偏置 2 | `left_wrist_roll` origin 的模长（`(0, 0.060993, -0.024239)`） | `0.06563` m = `65.63` mm |

上表的毫米值由米值换算得来，本库没有杆长实测记录。

**单位与坐标系**。长度单位是米，角度以 rad 写在 URDF 里。根 link 是 `left_Base`，指的是**该臂的肩底座系**，**不是整机基座系**——URDF 里没有底盘、升降到肩底座的变换，`ArmIK` 输出的末端位置也在这个肩底座系下。要算世界坐标，由调用方自己叠加基座到肩底座的变换；本库不提供该变换，也不给工作区结论。

六个关节的 URDF 名：`left_shoulder_pan`、`left_shoulder_lift`、`left_elbow_flex`、`left_wrist_flex`、`left_wrist_yaw_joint`（这一条 joint 名带 `_joint` 后缀）、`left_wrist_roll`。

**限位**。URDF 内六个 joint 的 `limit` 都是 `lower=-3.14` / `upper=3.14` rad，是上游值。本库未逐关节核对真实机械行程，限位是否宽于真实行程由 `docs/ADR.md` D10 的已知代价覆盖。现场口径是 `docs/field-ops/50-experiments.md` §10.7 ①a 在真机上读 `effective_limits_deg`，URDF 里的 `±3.14` 不作为真实行程的依据。

**零实测声明**。上游 `sources.json` 自报 `hardware_verified: false`，同记 `meshes_downloaded: false`、`placo_load_tested: false`、`retrieved_on: 2026-09-21`；本库也没有任何杆长实测记录，`docs/acceptance-baseline.md` 的 V01–V13 无几何条目。因此 FK/IK 的绝对精度**未经实测验证**：它证明的是「同一份几何下解算自洽」，物理尺寸是否正确它答不了。收口方式与判据见 `TODO.md` 的「URDF 杆长零实测」行、`docs/field-ops/50-experiments.md` §10.7 与 `docs/ADR.md` D21。

## 4 公开 API

- `alohamini_cartesian.adapter.protocol`：`RobotTransport`（`get_observation`、`send_action`、`prime_observation_request_window(*, include_cameras)`）、`RobotSpec`（`robot_model`、`left_port`、`right_port`），两个 `typing.Protocol`。
- `alohamini_cartesian.capability.detect`：常量 `SPAN_BY_NORMALIZATION`、`SUPPORTED_METADATA_SCHEMAS`；异常 `CapabilityError`、`ConversionUnavailable`；数据类 `MotorInfo`、`Capabilities`、`Conversion`；函数 `detect_from_observation`、`resolve_deg_per_unit`。
- `alohamini_cartesian.config.overlay`：常量 `ENV_PREFIX`；异常 `ConfigError`；数据类 `AlohaMiniSettings`；函数 `load_settings`。
- `alohamini_cartesian.control`：会话 `AlohaMiniControl`，句柄 `ArmHandle`、`BaseHandle`、`GripperHandle`、`LiftHandle`，入口 `connect`，结果 `MoveReport`、`StepDiagnostic`，异常 `FeedbackLost`、`MotionBlocked`、`MotionNotEnabled`、`MotionTimeout`、`OutOfRange`、`RuntimeUnavailable`。
- `alohamini_cartesian.kinematics.model_specs`：常量 `ROBOT_SPECS`、`ARM_PROFILE_JOINTS`；函数 `validate_robot_model`、`arm_state_keys`、`arm_state_keys_for_robot_model`。
- `alohamini_cartesian.kinematics.joint_adapter`：类 `JointKinematicAdapter`，方法 `observation_to_kinematic_deg`、`kinematic_deg_to_action`、`effective_limits_deg`、`sign`。

## 5 约束与机器守卫

两条硬约束，改代码时先看这里。

1. `src/` 内**任何**文件都禁止 import `lerobot`，含 `adapter/protocol.py` 自身。`lerobot` 只能通过 `RobotTransport` 与 `RobotSpec` 两个 `Protocol` 接触，其余模块不碰 `lerobot.motors`、`lerobot.robots.alohamini` 这类深层模块。
2. 能力探测不得把版本号当兼容判据。判据是这项能力是否存在，`schema_version` 只用来决定字段怎么解析。

两条机器守卫，不靠人工 review 维持。

- `tests/test_capability.py` 用 AST 遍历断言 `src/` 内**任何**模块（含 `adapter/protocol.py`）都不 import `lerobot`（守卫 `tests/test_capability.py::test_no_module_outside_adapter_imports_lerobot`），并用 `sys.modules` 前后快照断言导入 `capability.detect`、`config.overlay` 这两个动作本身不会引入 `lerobot`。
- `tests/test_control_commands.py` 断言 `control/` 下除 `api.py` 之外没有模块 import 设备适配层、客户端或主机模块。它带正反两条控制用例：一条证明守卫能命中相对 import 的适配层，一条证明合法 import 不会被误判。
