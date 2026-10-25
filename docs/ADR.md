# 架构决策记录（ADR）

本文记录 `alohamini-cartesian` 自 v0.1.0 起沿用的决策，编号 D1–D11。每条按背景、决定、后果三段记录，后果同时写正面收益与已知代价。状态写作「已接受（自 vX.Y.Z 起）」，不记具体日期。新的重大取舍按顺序追加编号，已接受的条目不回改。

## 索引

| 编号 | 主题 | 状态 |
| --- | --- | --- |
| D1 | 依赖形态：独立包，lerobot 作为可选 extra | 已接受（自 v0.1.0 起） |
| D2 | 配置格式：YAML 单文件加环境变量覆盖 | 已接受（自 v0.1.0 起） |
| D3 | 迁移范围：只搬控制与运动学，分阶段搬 | 已接受（自 v0.1.0 起） |
| D4 | 启动方式：新开仓库，旧仓库冻结为只读参考 | 已接受（自 v0.1.0 起） |
| D5 | 迁移范围与适配边界：控制层与运动学入库内，电机语义改为库内镜像 | 已接受（自 v0.2.0 起） |
| D6 | numpy 依赖：核心只加 numpy，不引入 torch/scipy | 已接受（自 v0.2.0 起） |
| D7 | 迁移边界：厂商侧协议测试不迁入 | 已接受（自 v0.2.0 起） |
| D8 | placo 依赖形态：声明 kinematics extra，接受 numpy 2.3.x | 已接受（自 v0.2.0 起） |
| D9 | 现场基准脚本迁移时机：`benchmarks/alohamini_io` 两脚本与配套测试在 v0.4.0 迁入 | 已接受（自 v0.4.0 起） |
| D10 | 关节限位取值在 `ArmIK.solve()` 入口校验：违规抛 `ValueError`，多余键忽略 | 已接受（自 v0.5.0 起） |
| D11 | 失败原因分类：解越限报 JOINT_LIMIT，未越限的残差超标报 RESIDUAL | 已接受（自 v0.6.0 起） |

## D1 依赖形态：独立包，lerobot 作为可选 extra

状态：已接受（自 v0.1.0 起）

### 背景

旧仓库把产品代码与 `lerobot` vendor 副本放在同一仓同一分支。库的版本号跟着 vendor 分支漂移，产品改动与实机运行时版本互相绑死。用 git submodule 能把代码体分开，但上游代码树仍被带进本仓，submodule 指针跟着产品分支一起更新，实机换版本还是要动产品仓库。

### 决定

本库做成独立发布的包：包名 `alohamini-cartesian`，模块名 `alohamini_cartesian`，src layout，hatchling 构建。核心 `dependencies = []`，不装任何第三方包即可导入与测试。`lerobot` 放在 optional extras 的 `runtime = ["lerobot"]`，只通过 `adapter/protocol.py` 里的 `RobotTransport` 与 `RobotSpec` 两个 `Protocol` 接触。YAML 解析单独放在 `yaml = ["PyYAML>=6.0"]`。不使用 git submodule。开发工具放在 PEP 735 的 `[dependency-groups] dev`（`pytest`、`ruff`、`PyYAML`），不放 `optional-dependencies.dev`，这样测试与 lint 工具不会进入发布元数据。

### 后果

正面：核心零依赖，产品逻辑可以在没有 `lerobot`、没有硬件的环境里导入、跑测试、过静态检查；`py.typed` 随之分发，调用方拿到类型信息；两代依赖各自演进，产品按本库的版本升级。
已知代价：`lerobot` 不在 PyPI 发布，`pip install .[runtime]` 无法直接解析，必须按 `lerobot` 上游 README 从上游仓库安装（例如 editable）到同一环境；环境装配顺序由使用者负责，本库的 `pyproject.toml` 只声明名字，不声明可解析的分发地址。

## D2 配置格式：YAML 单文件加环境变量覆盖

状态：已接受（自 v0.1.0 起）

### 背景

旧结构没有 overlay 表达现场差异，现场只能改源码。而现场改的那些文件正好是 vendor 更新会覆盖的文件，上游一更新，现场配置和补丁全部丢失。

### 决定

覆盖顺序固定为：默认值 → 模板文件 → 现场文件 → 环境变量。格式以 YAML 为主，`.json` 后缀走标准库，作为零依赖退路，按文件后缀分派，其他后缀报 `ConfigError`。校验严格：未知键报错、整数字段不接受字符串与 bool、数值字段按下界检查（`observation_request_window`、`watchdog_timeout_ms`、`max_loop_freq_hz` 均不得小于 1）、字符串字段不得为空串。环境变量前缀 `ALOHAMINI_`，键名大小写不敏感。`cameras` 在模板、现场文件、环境变量三层之间按键合并（同名键后者覆盖，各层独有键保留），其余字段保持整值覆盖。未知的 `ALOHAMINI_*` 键报错。设备节点存在性校验做成 `check_devices` 开关，默认关闭，只在现场与 smoke 时打开，避免在没有 `/dev/ttyUSB*` 的开发机上卡住加载。

### 后果

正面：现场差异写在现场文件与环境变量里，上游更新不会冲掉；配置错误在加载时暴露，不进入控制回路；容器与 CI 用环境变量覆盖即可，不必生成配置文件。
已知代价：YAML 需要额外依赖，所以保留 JSON 退路，代价是同一种配置有两条解析路径要各自测试；严格校验意味着含多余键的老配置文件会直接失败，迁移时要一次性清理；`cameras` 三层按键合并，因此无法通过某一层清空其它层已设的相机，只能覆盖同名键。

## D3 迁移范围：只搬控制与运动学，分阶段搬

状态：已接受（自 v0.1.0 起）

### 背景

旧仓库的 `control/` 与运动学代码依赖 `lerobot` 的电机与机器人实现，`benchmarks/` 依赖实机。把这两块随骨架一起搬，`lerobot` 会挤进核心依赖，v0.1.0 的验收也会被硬件挡住。

### 决定

迁移范围限定为三块：`control/`、运动学（`joint_kinematic_adapter`、`model_specs`）、`benchmarks/`，分阶段搬。本轮 v0.1.0 只做骨架、适配协议、能力探测与换算裁决、配置 overlay，上述三块不搬。S3 搬 `control/`、`joint_kinematic_adapter`、`model_specs` 以及本轮尚未搬运的 `benchmarks/`。S4 做现场 smoke（实机加 vendor 运行时），验证 overlay 能否完整表达现场差异。

后续修订（v0.2.0）：S3 实际只搬控制层与运动学，`benchmarks/` 推到 S4，与现场 smoke 同轮。

### 后果

正面：v0.1.0 的边界清楚，验收只依赖 71 条不碰硬件的测试；`lerobot` 与产品代码的解耦在搬控制回路之前先立住，后面搬迁时不必再改适配面。
已知代价：迁移期间两处代码并存，旧仓库与新库都有控制相关代码，修 bug 时要判断改哪一处，同一问题可能被修两遍；S3 完成之前，运动学与控制回路仍只在旧仓库可用。

后续修订（v0.4.0）：D3 把 `benchmarks/` 推到 S4 的安排已部分提前——`benchmarks/alohamini_io` 两个脚本与配套测试在 v0.4.0 迁入，见 D9。

## D4 启动方式：新开仓库，旧仓库冻结为只读参考

状态：已接受（自 v0.1.0 起）

### 背景

在旧仓库里改造，产品发布节奏仍然受 vendor 分支牵制，库的版本号也无法独立递增。改造过程还会持续产生新的提交，与上游 vendor 更新争同一批文件。

### 决定

新开独立仓库 `alohamini-cartesian-lib`，当前状态作为 `v0.1.0` 的起点，版本号从 `0.1.0` 起算，不继承旧仓库的版本序列。旧仓库冻结为只读参考，后续改动一律落在新仓库。

### 后果

正面：本库的版本线独立演进，产品侧可以按库的版本升级而不动 vendor 分支；新仓库里没有 vendor 副本，`adapter/` 之外禁止 import `lerobot` 这条纪律可以靠目录结构直接检查。
已知代价：旧仓库不再修 bug，只作参考；只落在新库的问题修复，旧仓库的使用者需要自行迁移；两边行为差异要靠 S4 现场 smoke 确认，在此之前旧仓库仍是实机运行的唯一已验证路径。

## D5 迁移范围与适配边界：控制层与运动学入库内，电机语义改为库内镜像

状态：已接受（自 v0.2.0 起）

### 背景

S3 要把旧仓库的 `control/` 与运动学搬进本库。旧代码在 `controller.py` 里 import `lerobot.robots.alohamini.*`，`api.py` 的默认 client 工厂又 import `AlohaMiniClient`。而 D1 定的接触面与 `tests/test_capability.py` 的守卫要求：`src/` 内除 `adapter/protocol.py` 外不得 import `lerobot`。照旧代码原样搬，核心导入会连带 vendor 运行时。

### 决定

控制层（`control/`：`api.py`、`commands.py`、`controller.py`、`arm_ik.py`）与运动学（`kinematics/`：`model_specs.py`、`joint_adapter.py`）整体搬入库内。`kinematics/assets/am_arm200/kinematics.urdf` 随包入库内。

控制器对 `commands` 与 `kinematics.joint_adapter` 改用相对 import。

`api.py` 的 `_default_client_factory` 改为抛 `RuntimeUnavailable`（`RuntimeError` 子类，已进 `control.__all__`）。真实连接必须由调用方注入 `client_factory`。

电机与校准语义（`normalization`、`drive_mode`、`homing_offset`、`resolution`、`range_min/max`）改为库内镜像。`joint_adapter.py` 保留与 `MotorsBus._normalize` / `_unnormalize` 的对应关系注释。核心不再需要 `lerobot.motors`。

`placo` 保持函数内惰性可选，不声明为依赖。

### 后果

正面：核心可脱离 vendor 运行时导入、跑测试、过静态检查。控制层与运动学的行为由库内测试固定，本机测试集 786 条用例中 737 条通过、51 条跳过。两条守卫可机器检查：`tests/test_capability.py` 查 import，`tests/test_control_commands.py` 查控制层对设备适配层的依赖。

已知代价：`connect()` 的默认路径不再可用，调用方必须自带 client 工厂。缺 `placo` 时 47+2 条数值用例跳过，运动学数值行为在最小环境里没有证据。缺 `lerobot` 时两个 parity 文件整体跳过，整环量化数学失去上游 oracle 对照。

后续修订（`placo` 落地之后）：上文「786 条用例中 737 条通过、51 条跳过」是 v0.2.0 接受当时的数字。迁入 `benchmarks/alohamini_ik` 并把 `placo` 装进开发环境之后，本机测试集为 825 条，823 条通过、2 条跳过（825 = 823 + 2）。

## D6 numpy 依赖：核心只加 numpy，不引入 torch/scipy

状态：已接受（自 v0.2.0 起）

### 背景

控制层与运动学的数学以 numpy 数组为边界：4×4 位姿、FK/IK 的种子与残差范数、刻度量化。旧代码已经在用 numpy。重写成纯 Python 不现实，还会引入逐位不一致的风险。

### 决定

核心 `dependencies` 只加 `numpy>=1.26`，本机解析到 2.5.4。

不引入 torch 与 scipy。`placo` 不声明为依赖。`yaml = ["PyYAML>=6.0"]` 与 `runtime = ["lerobot"]` 仍是可选 extras。

### 后果

正面：安装体积与解析成本仍小。矩阵代码不必重写，数值行为与旧仓库逐位一致，差分对照已做。

已知代价：核心不再是零依赖。没有 numpy 的环境不能 import 本库核心，D1 里「零依赖」的表述自 v0.2.0 起失效。numpy 主版本跨代（1.x → 2.x）的行为差异要由本库的测试兜住。

后续修订（`placo` 落地之后）：上文「`placo` 不声明为依赖」与「`yaml` 与 `runtime` 仍是可选 extras」已被 D8 取代——`placo` 现声明为 `kinematics` extra，numpy 由它的传递依赖锁在 2.3.x，见 D8。

## D7 迁移边界：厂商侧协议测试不迁入

状态：已接受（自 v0.2.0 起）

### 背景

旧仓库的 `tests/robots/test_alohamini_cartesian_protocol.py` 有 49 个测试函数（数字来自旧仓库只读盘点）。这些测试测的是厂商侧协议模块，即旧仓库 `src/lerobot` 内的实现。文件 import 5 个厂商符号，是纯对象 harness（只构造对象、直接调方法，不启 zmq、不起 socket、不建线程，可以离线跑）。

本库把控制层与运动学搬进库内之后，测试集里没有对应的覆盖。于是出现一个问题：这 49 条要不要一起搬进来。

### 决定

不迁。理由两条。

1. 这 49 条测的是厂商模块的行为，本库不含这些代码。本库 `adapter/protocol.py` 只有 `RobotTransport`、`RobotSpec` 两个 `typing.Protocol` 定义，没有任何实现。
2. 厂商内部协议的行为不由本库负责。本库不对厂商侧协议负回归责任。

### 后果

正面：本库的测试集只覆盖本库拥有的代码。测试失败必然指向本库的改动，不被厂商侧变更干扰。`src/` 内「除 `adapter/protocol.py` 外禁止 import `lerobot`」这条守卫，也不会被这些测试拉破。

已知代价：厂商侧协议在本库没有回归网。厂商仓库若改了协议行为，只能回到旧仓库跑那 49 条，本库无法提前发现。本库与厂商协议的兼容性，只能靠 `adapter/protocol.py` 的窄接口约定和 S4 现场 smoke 兜。

## D8 placo 依赖形态：声明 kinematics extra，接受 numpy 2.3.x

状态：已接受（自 v0.2.0 起）

### 背景

`placo` 是数值 IK 与 FK 的计算后端（把末端位姿解成关节角的求解库）。此前它只在函数内惰性 import，不写进依赖，后果是 49 条运动学数值用例长期因缺 `placo` 跳过，运动学的数值行为在本库里没有证据。`placo` 在 PyPI 上有官方 arm64 与 cp313 轮子，本机可以直接装，已经不存在装不上的理由。

### 决定

声明 `kinematics = ["placo>=0.10.1,<0.11"]`。上限的理由：`placo` 自 0.10 系列才切到 qpmad 的 QP 后端（用二次规划解关节角的求解器），数值断言基线按 0.10 定；钉 `<0.11` 是防将来再换后端导致数值漂移，把断言变成假失败。`placo` 装进开发环境，连带接受 numpy 由 2.5.4 降到 2.3.5。核心 `dependencies` 仍只有 numpy，`placo` 只出现在 extra 里。不改厂商仓库。

### 后果

正面：那 49 条运动学数值用例真跑并通过；离线 IK 基准（`benchmarks/alohamini_ik/benchmark_arm_ik.py`）可运行。实测 0.9.15 与 0.10.1 的 IK 解一致到 1e-13 量级（12 个关节角分量最大差 4.3e-14 度），说明数值基线对 `placo` 版本不敏感。

已知代价：依赖闭包 46 个包，含 pinocchio 一整套与 `meshcat`（其传递依赖含 tornado、pyzmq、pillow、ipython）。numpy 上限被 `pin==3.8.0` 拉来的 `cmeel-boost~=1.89.0` 锁在 2.3.x——上游 1.90.0 已放开为 `numpy>=2.0`，待 `pin` 升到 4.x 后这条限制才解除。换回 `placo` 0.9.x 不能规避这条代价（0.9.26 同样要 `pin==3.8.0`）。

## D9 现场基准脚本迁移时机：`benchmarks/alohamini_io` 两脚本与配套测试在 v0.4.0 迁入

状态：已接受（自 v0.4.0 起）

### 背景

`docs/archive/migration-backlog.md` 原定把 `benchmarks/alohamini_io` 两个脚本推到 S4 现场，理由是「本机没有 Host，迁入后也只是一份跑不起来的代码」。v0.4.0 重新核查，这条依据对脚本成立，对配套测试不成立。

两个脚本对 `lerobot` 的 import 全部在 `default_client_factory()` 函数体内（`capture_robot_fingerprint.py:140-141`、`measure_latency.py:847-848`），模块顶层只有标准库与 numpy。两个配套测试只用标准库与 pytest，用依赖注入的 fake client（假的客户端对象顶上真连接）与 fake clock，不碰 `lerobot`、`zmq`、`socket`、线程。

它们在旧仓库原位跑不起来的唯一原因，是旧仓库根 `tests/conftest.py` 顶层 import `lerobot`；pytest 收集测试时必然加载这个文件。迁入本库后不再加载那个 conftest。

### 决定

把两个脚本与配套测试在 v0.4.0 一并迁入，不等 S4。

迁入原则是语义零改动：与原文件 `ast.dump` 逐一相等（实测 4/4），只允许 `ruff format` 的机械折行与版本号同步。字节不同处只是折行，原因是旧仓库 line-length 110、本库 100。

为让这四份文件通过本库的风格门（ruff 的风格检查与格式校验），`pyproject.toml` 尾部新增 `[tool.ruff.lint.per-file-ignores]`，按四条精确路径配置，不 exclude 目录，不放宽全局 `select` / `ignore`。脚本两条豁免 `D103, D107, E501`；测试两条豁免 `ANN001/ANN201/ANN202/ANN204`（capture）与 `ANN001, ANN003, ANN201, ANN202, ANN204, ANN205, D403, E501`（measure）。

脚本里的 `from lerobot` 保持函数体内惰性导入，`lerobot` 仍不进 `dependencies`。

### 后果

正面：130 个测试函数（静态计数，20 + 110）进入本库测试集，参数化展开后是 176 条用例。`pytest tests` 从 `823 passed, 2 skipped` 变为 `999 passed, 2 skipped`，跳过的仍是两个 parity 文件（缺 `lerobot`）。`tests/benchmarks` 子集 190 条通过 = 14（`test_benchmark_arm_ik.py`）+ 23 + 153。这套采集与测量逻辑此后由本库回归网守着，不再依赖旧仓库。

已知代价：迁入的是「有测试、无实测」的代码。两个脚本从未对着真机 Host 跑过，`schema_version == 2` 的实际取值、Mode B 的真实运动、Gate F 的真实拒绝路径都还没有证据。per-file-ignores 让这四份文件在 docstring、类型注解与行宽三条规则上不受本库标准约束，长期看是风格例外。

## D10 关节限位取值在 `ArmIK.solve()` 入口校验：违规抛 `ValueError`，多余键忽略

状态：已接受（自 v0.5.0 起）

### 背景

`ArmIK.solve()` 的入参 `joint_limits_deg` 是「关节名 → `(min, max)` 度」的字典。v0.4.0 及之前，入口只检查每个必需关节**有没有这个键**（F2，「必需关节缺键就报错」这道检查），对键对应的**取值**不做任何校验，直接 `np.deg2rad` 后交给 placo。于是调用方把限位写错时，失败路径取决于错法：

| 写错的方式 | v0.4.0 实测行为 |
| --- | --- |
| 倒置 `(90.0, -90.0)` | `RuntimeError('QPError: Problem: Infeasible QP ...')`，栈顶 `arm_ik.py:242`（v0.4.0 文件行号，语句为 `solver.solve(True)`） |
| 退化 `(0.0, 0.0)` | 不抛，返回 `ok=False` / `reason='JOINT_LIMIT'` |
| 含 `NaN` | `RuntimeError('QPError: ... NaN in the QP solution')` |
| 含 `inf` | 不抛，返回 `ok=True`——上界被静默忽略 |
| 长度不是 2 | 元组解包抛 `ValueError: not enough/too many values to unpack`，消息不含关节名 |
| 非实数 `("a","b")` | `np.deg2rad` 抛 `TypeError: ufunc 'deg2rad' not supported ...` |
| `bool`（`(True, False)`） | 被当成 `(1.0, 0.0)`，于是 `RuntimeError('QPError: ...')` |

`IKResult` 是求解结果（成功/失败）的出口契约，两种结果都写进 `ok` / `reason` 字段；入参错误走 `ValueError`，这两条之外不该有第三种异常形态（本决策收敛的是**入参错误**这一个来源；在限位合法且有限时，求解过程本身对越界/退化的目标位姿仍抛 placo 的裸 `RuntimeError`，实测见「后果」）。裸 `RuntimeError` 直接从求解器内部穿出来，调用方拿到的既不是 `IKResult` 也不是 `ValueError`，而是一个未包装的求解器异常（QPError 是 placo 底层二次规划求解器的报错，指「这组约束无解」），于是「限位写反了」被伪装成「这个目标解不出来」。而 `FAILURE_REASONS`（`src/alohamini_cartesian/control/arm_ik.py:36`）里只有 `NON_FINITE` / `JOINT_LIMIT` / `RESIDUAL` 三类，没有「限位非法」这一类——限位本身是调用方给的常量，不是一次求解的结果，本来就不该占用失败原因。

### 决定

`solve()` 在 F2 那道键检查的同一个循环里、任何数值工作（构造 placo 机器人、首次求解）之前，对 `self.profile.joint_names` 里的 6 个必需关节逐个校验限位取值：

1. 取值是长度 2 的序列（`collections.abc.Sequence`；`str` / `bytes` / `bytearray` / `memoryview`、迭代器与生成器一律不算，实现见 `arm_ik.py:367` 的 `isinstance(limit, (str, bytes, bytearray, memoryview))`），否则报 `ValueError`；
2. 两个元素的类型在显式白名单里——Python `int` / `float`，以及 numpy 数值标量（`np.integer` / `np.floating`）；**`bool` 与 `np.bool_` 一律拒绝**（`bool` 是 `int` 的子类，`(True, False)` 会被数值比较放行成 `(1.0, 0.0)`）。白名单之外的类型全部拒绝，两种代表性输入已实测：`Fraction` 是 `numbers.Real` 的子类（实测 `isinstance(Fraction(1, 2), numbers.Real)` 为 `True`），能过 `numbers.Real` 检查，却让 `np.deg2rad(Fraction(1, 2))` 抛裸 `TypeError: loop of ufunc does not support argument 0 of type Fraction ...`；`Decimal` **不是** `numbers.Real` 的子类（实测 `isinstance(Decimal("0.5"), numbers.Real)` 为 `False`），走的是同一个「不在白名单」分支。两者都被 `ValueError` 拒绝，而不是让下游 `np.deg2rad` 抛裸 `TypeError`；
3. 两个元素都有限（`math.isfinite`，排除 `NaN` 与 `±inf`）。`±inf` 继续被拒绝、**不为它写特例代码**，依据是默认路径撞不上这组取值：`connect()` 的 `adapter` 默认是 `None`（不给适配器就不会有 `(-inf, inf)` 这类限位）；`src/` 内零个 `JointKinematicAdapter(` 构造点（全库 `grep -rn "JointKinematicAdapter(" --include=*.py .` 命中 8 处，全部在 `tests/` 下，`src/` 内除 `kinematics/joint_adapter.py` 自身的类定义/文档外计数为 0）；真实 Host 默认 `use_degrees=False`，臂关节 `normalization` 取 `range_m100_100`（缺省值，`capability/detect.py:20`），现场实测限位 `±174.07°`。反证：若放行 `(-inf, inf)`，`_clamp_to_joint_limits` 的 `within_limits`（`arm_ik.py:138`、`arm_ik.py:142`）会恒真、`JOINT_LIMIT` 永不触发，限位校验整条失效；
4. 满足 `lo < hi`（倒置与退化都拒绝）。

限位值限定为上面第 2 条那份白名单（`bool`、`np.bool_`、`Fraction`、`Decimal` 与其余一切非白名单类型一律拒绝），`solve()` 不修改调用方传入的限位字典：不复制、不写回，取值按调用方给的原始类型只流向**第二次**求解，也就是注入调用方限位那次（`arm_ik.py:443`）。`solve()` 里的第一次求解（`arm_ik.py:420`）用的是内部按 URDF 构造的 `urdf_limits_deg`（`arm_ik.py:409-412`），调用方给的取值不参与。

违反上述四条的一律抛 `ValueError`，消息点名关节并给出实际取值（`joint_limits_deg['left_elbow_flex'] must satisfy lo < hi, got lo=90.0, hi=-90.0`）。第 3 条「有限」覆盖超出 `float` 范围的整数：`math.isfinite` 先转 `float`（`arm_ik.py:391`），`(10**400, 10**401)` 会在那里抛 `OverflowError: int too large to convert to float`，入口把它并进既有的 `must be finite` 分支（`arm_ik.py:390-397`），实测 `(10**400, 10**401)` 抛 `ValueError`、消息为 `joint_limits_deg['left_elbow_flex'] must be finite, got lo=<10**400>, hi=<10**401>`（`repr` 回显的是这两个值的十进制写法，不作位数口径的换算）、抛出位置 `arm_ik.py:395`（消息在 `arm_ik.py:396`）。这只覆盖「调用方把参数写错」这一类，属于入参错误，不是求解失败。

多余的键（不在 `profile.joint_names` 里的关节）**明确忽略**：校验只遍历必需关节，不遍历整个字典。这条契约由两条用例共同守护——`T-IK-18.4`（多余键取值合法时，7 键与 6 键的求解结果逐字段一致）与 `T-IK-18.7`（多余键取值**非法**时仍被忽略，`solve()` 正常返回、`ok is True`，且与 6 键基线逐字段一致）。只有 18.4 杀不动「遍历整个字典校验」这类错误实现：它给多余键填的是合法值，全表校验照样通过，判别落在 18.7。

「类型白名单」这条契约由两条用例共同守护——`T-IK-18.10`（numpy 数值标量被接受，`np.float64`/`np.float32` 与 `np.int64` 两组取值各一条：`solve()` 正常返回 `ok is True`，且 `q_deg` / `position_error_m` / `iterations` 与同值 Python `float` 基线一致）与 `T-IK-18.11`（`Fraction` / `Decimal` 抛 `ValueError` 而**不是** `TypeError`，消息属于 `must hold real numbers` 那一支并把收到的对象 repr 回显）。前者钉住白名单不能收窄成 `(int, float)`（收窄后 `np.int64` / `np.float64` 被误拒），后者钉住白名单不能放宽成 `numbers.Real`（放宽后 `Fraction` 会走到 `np.deg2rad` 抛裸 `TypeError`）。`np.bool_` 的拒绝由 `T-IK-18.13` 单独守护（`(np.bool_(True), np.bool_(False))` 与 `(np.float64(-90), np.bool_(True))` 两组：抛 `ValueError`，消息落在 `must hold real numbers` 这一支并带 `got lo=` / `got hi=` 前缀；后一组是鉴别点——只按数值比 `lo < hi` 的实现会接受 `-90.0 < 1.0` 并照常求解；`T-IK-18.6` 只覆盖 Python `bool`，守不住白名单放宽到 `np.bool_` 这一刀）。

「迭代器/生成器不算序列」这条契约由 `T-IK-18.12` 守护：迭代器 / 生成器 / `None` / `dict` / 裸标量五组输入一律抛 `ValueError`，消息点名关节并落在 `must be a (lo, hi) pair` 这一支（形态不对），而不是「非实数」或「有限性」那两支。删掉 `not isinstance(limit, Sequence)` 后，前几组会先撞 `len()` 的裸 `TypeError: object of type '...' has no len()`，而两键 `dict` 会因 `len() == 2` 通过形态检查、解包成键 `("lo", "hi")` 再落进「非实数」那一支——异常类型与消息分支都不对。

残差失败被归类成 `JOINT_LIMIT`（`arm_ik.py` 的 `ok = position_error_m <= residual_limit_m and within_limits` 之后那段 `IKResult(... reason="JOINT_LIMIT")`）是另一个独立问题，**不在本决策范围**，另案处理。

### 后果

正面：配置错误在入口暴露，调用方不必为「限位取值写错」去兜 placo 的异常——限位取值这类**入参错误**收敛为 `ValueError`（你给的参数不对，改参数），求解没成功走 `IKResult(ok=False, reason=...)`（看原因）；在限位合法且有限时，求解过程本身仍抛 placo 的裸 `RuntimeError`，调用方仍需保留兜底（实测：`target_pose` 平移 `1e300` m 抛 `RuntimeError: QPError: Problem: Infeasible QP (check your hard inequality constraints)`，`target_pose = np.zeros((4, 4))` 抛 `RuntimeError: QPError: Problem: NaN in the QP solution`，两者抛出位置都是 `solver.solve(True)`，当前行号 `arm_ik.py:252`）。`inf` 限位不再被静默接受，`NaN` 不再等求解器报错，`bool` 不再被当 1/0；报错消息带关节名与实际取值，现场改配置时不必再从一段 QP 报错里猜是哪个关节。

已知代价之一：这是对外行为变更——同样的错误输入，以前炸 `RuntimeError`（或根本不炸），现在炸 `ValueError`，按异常类型分流的调用方需要调整捕获类型。实测既有用例零改动即全绿（`tests/test_arm_ik.py` 115 条 = 89 条既有 + 26 条新增 T-IK-18，无一条依赖旧行为、无需逐条豁免；全库 `1025 passed, 2 skipped`）。

本库内部**有**承载旧行为的调用路径，不是「只在入口加校验、内部调用不受影响」：`kinematics/joint_adapter.py` 的 `JointKinematicAdapter(urdf_limits=None)`（默认不注入 URDF 限位，`joint_adapter.py:144`、`joint_adapter.py:152`）配合六关节 `normalization == "degrees"` 时，`_effective_q_range` 返回 `(-inf, inf)`（`joint_adapter.py:431`），`effective_limits_deg()`（`joint_adapter.py:443`）在这组配置下返回六个 `(-inf, inf)`（实测：`JointKinematicAdapter(signs=…, zeros=…)` + 六关节 `"degrees"` metadata → 六个 `(-inf, inf)`）。这组限位在 v0.4.0 下让 `solve()` 返回 `ok=True`（实测），现在会在入口抛 `ValueError: joint_limits_deg['left_shoulder_pan'] must be finite, got lo=-inf, hi=inf`（实测，抛出位置 `arm_ik.py:395`）。两个内部调用点会把这个值交给 `solve()`：`control/controller.py:472` 取限位、`controller.py:482` 调 `solve()`，外层的 `except Exception` 把它转成 `ControlError.IK_FAILED`（`controller.py:492`，`sent=False`，不发送动作）；`control/api.py:184` 的路径预检取限位后直接调 `solve()`，这个 `ValueError` 穿出 `_precheck_straight_path`（`api.py:158`，调用点在 `api.py:332`，该处无 `try/except`），到达 `ArmHandle.move_to` 的调用方。

取向：**限位必须有限是本决策的前提**，不因这条路径放宽。`effective_limits_deg()` 给出 `(-inf, inf)` 属于「未注入 URDF 限位」的配置缺口，应由调用方构造 `JointKinematicAdapter` 时注入有限限位解决；不改 adapter 现有行为，不在本决策范围内。

已知代价之二：合法调用形态被收窄——`np.ndarray` 限位对（如 `np.array([-90.0, 90.0])`）在 v0.4.0 可用（实测 `solve()` 返回 `ok=True`），现在因 `ndarray` 不是 `collections.abc.Sequence` 而被 `ValueError` 拒绝（实测消息 `joint_limits_deg['left_shoulder_pan'] must be a (lo, hi) pair, got array([-90.,  90.])`，抛出位置 `arm_ik.py:371`）。这是有意的收窄：契约只接受长度 2 的序列（元组/列表），代价是偶尔用数组传限位的调用方需要改成 `tuple(...)`。

已知代价之三：校验只做「形状 + 类型白名单 + 有限 + `lo < hi`」四条，不做语义合理性检查（例如限位是否宽于真实机械行程、是否与 URDF 一致），这类越界仍靠求解结果与 `JOINT_LIMIT` 分类体现。

已知代价之四（已修复）：`memoryview` 等 buffer 形态曾被当成一对整数放行（`memoryview(b"ab")` 是 `collections.abc.Sequence`、`len == 2`、元素是 `int`），绕过形态检查与类型白名单；本轮已把它与 `str` / `bytes` / `bytearray` 同列排除（`arm_ik.py:367`，`isinstance(limit, (str, bytes, bytearray, memoryview))`）。`memoryview(b"ab")` 现在抛 `ValueError`，消息落 `must be a (lo, hi) pair` 一支并回显收到的对象（`got <memory at 0x...>`，即该对象的 `repr`）；`memoryview(b"abcd")`（`len() != 2`）与 `bytearray(b"ab")` 行为不变，仍走同一条拒绝路径。由 `T-IK-18.14` 守护（把 `memoryview(b"ab")` 作为 `left_elbow_flex` 的限位传入，断言抛 `ValueError`、消息点名关节、落在形态分支并回显 `repr`）。

已知代价之五：入口校验通过后，限位两端在 `_solve_once` 里先统一 `float` 归一，再进 `np.deg2rad`（`arm_ik.py:234-236`，即 `robot.set_joint_limits` 那几行），因此「极大但有限」的整数不再以裸 `TypeError: loop of ufunc does not support argument 0 of type int ...` 穿出——`(2**63, 2**64)` 这般输入能过入口校验。代价是这类输入会以 placo 的 `RuntimeError: QPError: Problem: Infeasible QP` 收场，**属求解不可行，不是入参校验覆盖范围**；真实限位的量级（≤ 若干百度）不会触到。由 `T-IK-18.15` 守护（断言「不得有 `TypeError` 逃出 `solve()`」，允许 `ValueError` / placo `RuntimeError` / 正常返回）。评估后维持现状：为「极大但有限」的限位设量级上限需要一个没有现场依据的阈值，因此不在入口加这条上限，仍以 placo 的 `QPError: Infeasible QP` 收场；真实限位的量级在几百度以内，触不到这条。

## D11 失败原因分类：解越限报 JOINT_LIMIT，未越限的残差超标报 RESIDUAL

状态：已接受（自 v0.6.0 起）

### 背景

`ArmIK.solve()` 结尾把「没到位」一律报 `reason="JOINT_LIMIT"`，已经算好的 `within_limits` 判定被丢弃。问题报告记录了一次现场观测：六个关节的值全部落在注入区间内、位置误差 `0.074 m`（本仓库 `evidence/` 目录无该次运行的书面记录，未复核，只作为现象描述），报告仍写 `JOINT_LIMIT`。

为什么这会误导处置：`JOINT_LIMIT` 读起来是「机器人关节到极限了，得改限位或换姿态」，`RESIDUAL` 读起来是「目标够不到，得改目标」——两者对应的处置方向相反。`src/alohamini_cartesian/control/controller.py` 约 `:497` 用 `getattr(ControlError, result.reason, ControlError.IK_FAILED)` 把 `reason` 翻成对外错误码，因此误标会一路传到调用方的分流逻辑上。

### 决定

`reason` 按下表分流，不附加第三类条件：

| 情况 | `reason` |
| --- | --- |
| 解越出注入限位（越限判定仍含 `LIMIT_EPS_DEG` 容差） | `JOINT_LIMIT` |
| 未越限，但位置残差超过 `profile.residual_limit_m` | `RESIDUAL` |
| 两者同时成立 | `JOINT_LIMIT`（越限是硬失败，优先） |

`FAILURE_REASONS` 不新增枚举值。入口校验、先用 URDF 限位求解判物理可达性的前置分支、`_clamp_to_joint_limits()` 的夹紧与容差语义、`ok=True` 分支、`IKResult` 字段均不变。

### 后果

正面：`reason` 与事实一致，调用方能按「改目标」与「改限位」两条路分流；`ok` 与 `reason` 仍然同源（`ok=False` ⟺ `reason` 是两个失败词之一）。

已知代价之一（对外行为变更）：一部分失败的 `reason` 从 `JOINT_LIMIT` 变成 `RESIDUAL`，经 `controller.py` 映射后对外错误码从 `ControlError.JOINT_LIMIT` 变成 `ControlError.RESIDUAL`，按错误码分流的调用方需要调整。实测受影响的既有用例 2 条：原 `test_t_ik_4_1_physically_reachable_but_over_synthetic_limits_reports_joint_limit`（已改名为 `..._too_tight_limits_reports_residual`，残差 `0.024745237494851028` > `0.005`、原始未夹紧解仅外溢 `1.78e-15` 度）与 `T-IK-11.1[joint_limit]`（label 改为 `over_tight_limits`）；这两条旧期望值断言的就是本决策要修掉的误标，已按新契约改正并补了自证前提。

已知代价之二：真实 qpmad 在注入限位下几乎不外溢（实测 1e-15 度级），所以端到端用真实求解器已经造不出 `JOINT_LIMIT`；该分类的覆盖现在依赖按对象身份区分两次 `_solve_once` 的假求解器用例（`T-IK-17.11`、`T-IK-19.1`、`T-IK-19.2`）。这是构造性限制，非覆盖遗漏，但 `_solve_once` 若被重构，这组用例会以「前提破了」的形式成片失败。
