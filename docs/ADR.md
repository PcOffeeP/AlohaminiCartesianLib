# 架构决策记录（ADR）

本文记录 `alohamini-cartesian` 自 v0.1.0 起沿用的决策，编号 D1–D19。每条按背景、决定、后果三段记录，后果同时写正面收益与已知代价。状态写作「已接受（自 vX.Y.Z 起）」；已落地但尚未发版的条目写作「已接受（未发版：…）」并写明当前 `pyproject.toml` 版本与 `docs/changelog.md` 缺行，不记具体日期。新的重大取舍按顺序追加编号，已接受的条目不回改。

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
| D12 | 基准脚本客户端能力探测：计时面不存在或填不满时 rtt 中止并返回退出码 9，采集脚本相关字段记 `null` | 已接受（自 v0.7.0 起） |
| D13 | 离线基准的预算判据抽成纯函数 `within_budget()`，测试不再对真实墙钟断言 | 已接受（自 v0.7.0 起） |
| D14 | 左夹爪标定暂不重做：按现有标定值继续，先定性再修 | 已接受（自 v0.7.0 起） |
| D15 | V11 的「帧平滑回退」不做：v1 兼容范围界定为「能解析 + 能换算可落」 | 已接受（自 v0.7.0 起） |
| D16 | lerobot 依赖声明加版本区间：只支持 `lerobot>=0.6.1,<0.7` | 已接受（自 v0.7.0 起） |
| D17 | 设计规范定为历史设计来源，本库权威以 ADR 与验收基线为准 | 已接受（自 v0.7.0 起） |
| D18 | tracking 模式的 safety 快照单独成 sidecar（9 列长表），不进 14 列主表 | 已接受（未发版：`pyproject.toml` 仍为 `0.7.0`；`docs/changelog.md` 已补记（v0.7.0 段）） |
| D19 | 证据红线：V01 不收口不发 minor 版，每个发版留一格实机证据 | 已接受（未发版：`pyproject.toml` 仍为 `0.7.0`、`git tag -l` 只到 `v0.6.0`；`docs/changelog.md` 尚未记本条，留待发版时补） |

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

已知代价：迁入时的状态是「有测试、无实测」。2026-10-25 已在真机上补上三项证据：`schema_version = 2` 的指纹（18 个电机、换算字段齐全）、Mode B 受控试动五次（腕部达成 87.9%，肩部两轴未跟随）、Gate F 第一段真实拒绝（退出码 6、零命令）。仍未取到的：Gate F 第二段、跨机 RTT、右臂试动、相机非空帧。汇总见 `docs/field-results.md`。per-file-ignores 让这四份文件在 docstring、类型注解与行宽三条规则上不受本库标准约束，长期看是风格例外。

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

## D12 基准脚本客户端能力探测：计时面不存在或填不满时 rtt 中止并返回退出码 9，采集脚本相关字段记 `null`

状态：已接受（自 v0.7.0 起）

### 背景

Pi 上跑的 vendor 是 `main` 分支（`ec65318f`）。它的 `AlohaMiniClient` 提供 `latest_host_timing`，不提供 `latest_request_timing`。它也提供 `latest_safety_status`，与 `latest_control_status` 不是同一帧，字段结构不同（`version`、`control_owner`、`host_session_id`、`command_watchdog_timeout_s`）。

两个现场基准脚本此前按属性名直读计时面。在 Pi 上跑 rtt 模式，计时列只能拿到空单元格，而记录「主机计时缺失」的 `host_timing_missing` 仍为 `False`，同一行数据自相矛盾。

### 决定

两个脚本在 `connect()` 返回后探测客户端有没有要用的属性，按探测结果决定继续还是中止，不按假设直读。

`measure_latency.py --mode rtt`：探测分两处。第一道在 `connect()` 返回之后、`_prime_state_only_window()` 之前，问的是「客户端有没有 `latest_request_timing` 这个属性」；第二道在采样循环里问「有的那一面填满了没有」，判据是 `REQUIRED_REQUEST_TIMING_KEYS`（`token`、`sent_monotonic_s`、`received_monotonic_s`、`age_s`）逐键可用，缺一个就拒。放在预热之前是有意的：预热本身会发出一整个窗口的 state-only 请求，跑完再拒绝等于白放一窗请求。

退出码 `9` 因此有三种形态，三者都是 `9`、都不写 CSV，连表头都不写：

| 形态 | 何时触发 | 是否发过请求 |
| --- | --- | --- |
| 属性不存在 | `connect()` 返回后、窗口预热之前 | 没发过 |
| 属性存在但从未填充（整轮是空 dict，`observation_sequence` 不增长） | 窗口预热之后，采样循环放弃那一刻才判，要等满 `--timeout-s` | 发过 1 次 state-only 窗口预热，并至少观测 1 帧 |
| 半填充（缺 `token` / `sent_monotonic_s` / `received_monotonic_s` / `age_s` 任一） | 写行之前逐帧复查，`observation_sequence` 每次增长都查一次 | 已发过请求，并在该帧拒绝 |

stderr 文案区分形态，并点名缺的键：属性不存在时是 `it publishes no latest_request_timing` 那一支；属性存在但填不满时是 `it publishes latest_request_timing but never fills it completely`，后跟 `missing key(s): …` 列出缺的键。现场只靠退出码分不出这三种，要读 stderr 那一段。

`capture_robot_fingerprint.py`：缺 `latest_request_timing` 时继续采集，把 `age_s`、`sent_monotonic_s`、`received_monotonic_s` 记为 `null`，每次调用只打一行警告并点名缺失的属性；缺 `latest_control_status` 时 `control_status` 记 `null`。

明确排除两种做法。

1. 用 `getattr` 静默回退。rtt CSV 的 `token`、`sent_monotonic_s`、`received_monotonic_s`、`age_s`、`net_rtt_*`（`net_rtt_est_s`、`net_rtt_low_s`、`net_rtt_high_s`）会变成空单元格或 `NaN`，而 `host_timing_missing` 仍为 `False`。跑出来的表看起来是一次正常测量，等于谎报主机计时存在。
2. 用 `latest_safety_status` 顶替 `latest_control_status`。两者不是同一帧，结构不同（`version`、`control_owner`、`host_session_id`、`command_watchdog_timeout_s`），映射会改变 payload 在同一键下的语义。

依据是证据纪律：缺的数据标缺，不填 0、不填 `NaN`、不装作正常。

### 后果

正面：缺能力在跑之前暴露，不再产出看起来正常、实际没有计时数据的表；capture 的 payload 里 `null` 直接说明这一面缺失，读表的人不必去猜空单元格的含义。

用户可见契约有三处变化：

1. 新增退出码 `9`（`EXIT_CLIENT_UNSUPPORTED`）。在缺 `latest_request_timing` 的客户端上，rtt 模式以 `9` 结束，不产出 CSV；属性存在但填不满（整轮为空或半填充）同样以 `9` 结束，也不产出 CSV。三种形态的触发点与是否发过请求见「决定」那张表。
2. `capture` 的 payload 在缺面时出现 `null` 字段（`samples[].age_s`、`samples[].sent_monotonic_s`、`samples[].received_monotonic_s` 与 `control_status`）。10 个顶层键与 `PAYLOAD_SCHEMA_VERSION = 1` 不变。
3. 版本号从 `0.6.0` 升到 `0.7.0`。

已知代价：第二道探测做得严格，代价有三处。① 整轮为空的面要到采样循环放弃那一刻才拒，所以冻结形态的 `9` 要等满 `--timeout-s`，期间没有任何提前提示；半填充面则在写行前那一帧拒，代价小。② 首帧未计时即被拒：探测在它看的那一帧上分不清「这台宿主从不给请求计时」与「这台宿主还没计时」，等下一帧再判会重新打开运行开局，因此选择拒，可能拒掉一个合法客户端；真实 vendor 是否出现该形态未验证。③ 逐帧复查隐含假定 `observation_sequence` 增长与计时面填充在同一次 `get_observation` 里原子完成；若真实客户端由后台线程先加序号再写计时面，逐帧严格探测会偶发误拒，该原子性未验证。

已知代价（2026-10-26 追加，离线段）：「填满没有」的判据从「值能不能转换」改成「值能不能给出有限且物理上说得通的值」，于是退出码 `9` 多出第四种形态 —— 键在、值不可用：`bool` 时间戳或年龄（`float(True)` 得 `1.0`，写进 CSV 是一个纪元起点的戳）、负 `age_s`（请求不可能在发出之前被回答，且 `net_rtt_estimate_s` 会把净估值夹到 0，读起来像一次很快的往返）、非 `str` 或只有空白的 `token`（`str(123)` 是 `"123"`，`"   "` 非空但是空单元）、`float()` 溢出（`10**400` 抛 `OverflowError`）、`repr()` 抛异常。它们与缺键同码 `9`、同样不写 CSV（连表头都不写），代价是一批看似能转换的客户端会在这几条上被拒；真机 vendor 是否真的出现这些值形态没有现场证据。判据落在 `missing_request_timing_keys`，实现是 `_timing_key_problem` 与 `_request_timing_key_problems`；`_finite_or_none` 的全局语义未动，其它消费者继续依赖它接受 `bool` 与数值字符串。

已知代价的第二处是 stderr 的项形态契约。缺键子句的每一项要么是裸键名（该键不存在），要么是 `键名 (原因)`（值在但不可用）；原因是强制的，只有括在里面的**引用值**可选 —— `repr` 超过 32 字符、含 `", "` 或非可打印（含换行）时只丢引用。两个失败方向不能互相牺牲：子句必须仍能按 `", "` 切分并还原出键名（引用里带 `", "` 会被读成一个不存在的键，换行会把 `, so ` 及其后的补救句切掉），而丢掉原因会让运维把「值坏」读成「键缺」，被支使去查一个根本不存在的字段。因此引用不可靠时只丢引用、不丢原因。

已知代价的第三处在节拍。重复帧（`observation_sequence` 没增长的帧）不再各等一个完整周期，而是追下一个行期限 `start + len(rows) * period`，用地板 `IDLE_POLL_FLOOR_S`（1 ms）防忙转；还没有行时仍等一个完整周期（那时没有节拍可追，0 行冻结的轮询上界靠这一条成立）；落后超过一个周期（序列已冻结，期限永远停在过去）退回整周期等待。这是「不忙转」与「不按周期节流重复帧」两个约束的交点：修前默认 30 Hz / 30 s 的冻结尾部要 29,903 次 `get_observation`，修后 933 次。代价是冻结后的第一个周期内仍按 1 ms 地板轮询（100 Hz / 1 s 实测 109 次，上界 220），周期短于地板的速率（`--rate-hz 1e6`，1 µs）整段按地板轮询。

仍未验证（离线段）：真机 vendor 上的上列值形态与冻结形态都没有现场证据；`achieved_rate_hz` 的分母从循环入口起算、第 0 行不花周期，因此天然偏高 `N/(N-1)`（`--samples 3` 实测 264.2 Hz、请求 200.0 Hz），保持既有口径未改。该偏置只与行数有关、与速率无关（倍率恒为 `N/(N-1)`），适用范围：① 直接可用——同一 `--samples` 下两次运行互比（偏置同量抵消），以及 `--samples` 足够大时按原值读（500 行约 +0.2%）；② 须先按 `(N-1)/N` 修正——拿它判断是否达到 `--rate-hz`，或把 `1/achieved_rate_hz` 当作平均请求周期用；行数很少时偏差不可忽略（`--samples 3` 报 264.2 Hz 对请求 200.0 Hz，1.32×，理论上限 1.50×）。

相关提交：实现 `c919fef`；探测前移到窗口预热之前 `a0d82b3`；测试 `36445e1`、`1c2d1fd`；措辞修正 `acc6912`；第二道探测与 Gate F 第二段端到端测试 `4e6b5c5`；半填充面同样拒绝、冻结加空面改报 `9` 并消除空转 `dc5356ae`。

## D13 离线基准的预算判据抽成纯函数 `within_budget()`，测试不再对真实墙钟断言

状态：已接受（自 v0.7.0 起）

### 背景

`tests/benchmarks/test_benchmark_arm_ik.py` 的 `14_x` / `15_x` 系列在机器满载时偶发 1–5 条红，与业务改动没有因果关系。根因是这些用例对真实墙钟（wall clock，即真实流逝的时间）做绝对毫秒断言：本机负载一抖，测到的 p99 就抬起来，判据随之翻红，红的那几条与代码有没有变慢无关。

这类断言有两个后果。测试结果随机器状态漂移，红绿灯失去判别力；为了让它稳定，只能调宽阈值或加大规模，而阈值一旦放宽就不再守得住性能。

### 决定

把预算判据从「真跑一次看墙钟是否超时」改成对纯函数求值。

`benchmarks/alohamini_ik/benchmark_arm_ik.py` 里新增模块级纯函数 `within_budget(solve_p99_ms, step_p99_ms)`（`benchmark_arm_ik.py:150`），判据只有一条：`solve_p99_ms <= 7.0 and step_p99_ms <= 15.0`（常量 `SOLVE_BUDGET_MS`、`STEP_BUDGET_MS`，对应 30 Hz 下 p99 求解不超过 7 ms、整步不超过 15 ms）。`run_benchmark()` 仍真跑真计时，只把实测到的两个 p99 交给这个函数判定，报告里的 `within_budget` 字段与进程退出码都从它来。

测试分两层：判据的正确性由 `T-IK-16.1`–`T-IK-16.5` 用**构造的**耗时直接验证（阈值常量钉住、闭区间边界包含、单侧超限、非有限百分位、参数顺序）；脚本层面只断言结构与自洽（报告字段齐全、退出码与 `within_budget` 一致），不施加任何绝对毫秒上限，也不再要求小规模运行必须 exit 0。

### 后果

正面：满载下 `tests/benchmarks/test_benchmark_arm_ik.py` 连跑 20 次全绿，红灯重新只指向真实回归。判据只有一处实现，报告的 `within_budget` 与退出码同源（exit 0 当且仅当 `within_budget=True`），参数传反这类错误由 `T-IK-16.5` 单独钉住。`T-IK-14` 那类「不可用 URDF 必须非零退出」的用例仍守着退出码语义，不会被这条改动掩盖。

已知代价：绝对毫秒数不再由测试守住。性能预算是否真的满足，只能靠基准脚本真跑一次看退出码，CI 里跑测试不再构成性能证据。退出码 `1` 的语义从「测试失败」变成「跑完了但超出预算」，读日志时要按这条区分。

相关提交：`e85e828`（同时消除 `T-IK-4.2` 的恒真断言）。

## D14 左夹爪标定暂不重做：按现有标定值继续，先定性再修

状态：已接受（自 v0.7.0 起）

### 背景

离线核对（`docs/field-results.md` 的「离线核对（无新采集）· 左右夹爪行程差异」节）发现左夹爪 `arm_left_gripper` 的 `range` 只有 51 tick，`range_0_100` 下折合 4.48° 行程；右夹爪是 1469 tick、129.14°。18 个电机里唯一离群的就是这一台（第二窄的真实行程是右夹爪 1469，再是 `arm_right_wrist_yaw` 1826）。`range` 中心 2046.5 与归零位 2047 几乎重合，指向标定采集时夹爪基本没开。

问题出在哪一段没有定论：标定采集环节与写盘环节都能解释这个数值。ID7 的寄存器从未读过，所以「数据层已写错」这条只是推断，**未验证**。

### 决定

暂不重新标定左夹爪，控制侧与现场按现有标定值继续。不因此改动任何标定文件，不写任何寄存器——包括 ID7 的 `Min_Position_Limit` / `Max_Position_Limit`。

理由两条：① 重标定属硬件侧动作，当前不在本轮范围内；② ID7 寄存器从未读过，问题出在标定采集环节还是写盘环节尚未有定论，此时重标定会一并覆盖采集与写盘两条路径，可能掩盖真正原因。

### 后果

正面：A 阶段主线无阻。A 阶段的完成判据是六关节跟随，左夹爪不在其中；先收口主线，把有限的现场窗口留给尚未定位的问题。

已知代价：左夹爪可用行程按 4.48° 计，`open()`（100）与 `close()`（0）在物理上分不开夹爪；指令超出行程两端静默截断、链路不报错，属静默失效（本库 `control/commands.py:103-104` 把夹爪钳在 `[0,100]`，厂商的 `_unnormalize` 钳在区间内）。涉及抓取的用例与 `docs/acceptance-baseline.md` V02 的夹爪部分受限，重标定前不可用。

触发重新评估的条件（达成任一条即重新评估；未达成前继续按现有标定值跑）：① 只读读出 ID7 的 `Min_Position_Limit` / `Max_Position_Limit` 确认为 2021/2072，即固件限位就是这个窗口，说明问题在上游标定采集而非写盘；② 出现必须用夹爪的验收场景或实机任务。

相关提交：本轮文档提交（无代码提交）。

## D15 V11 的「帧平滑回退」不做：v1 兼容范围界定为「能解析 + 能换算可落」

状态：已接受（自 v0.7.0 起）

### 背景

`docs/acceptance-baseline.md` 里 V11 的判据第三款要求「v1 格式帧平滑回退」。这句话的出处是退役仓库的验收表（`docs/alohamini-control/Alohamini-implementation-plan.md:80`），原文只有「v1 格式帧平滑回退」7 个字，从无展开定义：平滑对象、系数、启用范围都没有说。

本库 `src/alohamini_cartesian/capability/` 内既没有跨帧平滑实现，也没有相关原语。没有 EMA、没有低通、没有 slew 限速，也没有「保持上一帧」的缓存；`detect_from_observation` 与 `resolve_deg_per_unit` 逐帧无状态。`tests/test_capability.py::test_v11_no_frame_smoothing_primitive_in_the_package` 用词边界正则扫全包，当前零命中。

v1 帧真正拿到的兼容是两件事。一是 schema 1 的 metadata 能被收下：缺字段填 `None`，`notes` 点名缺了哪些。二是换算按字段三级回落（metadata → calibration-file → model-table），缺到无处可落时抛 `ConversionUnavailable`，不猜 4096。这两件事之外，「帧平滑回退」没有可判的判据。

### 决定

不实现帧平滑回退。V11 的 v1 兼容范围界定为「能解析 + 能换算可落」，不含任何平滑、滤波、保持上一帧的语义。

理由三条：

1. 原始需求只有 7 个字、没有展开定义。凭这 7 个字定不出平滑对象（观测帧、命令序列还是换算结果）、系数与启用范围，写出来的实现也无法判断对不对。
2. 真实需求已被覆盖。旧系统发来的 v1 帧要能被收下并算出关节量，这件事由解析与换算回落两段承担，用例已锁定。
3. 凭空加滤波器会把抖动来源藏进本库。观测抖动的原因若在对端时序，滤波器只会掩盖它，掩盖之后现场就找不到抖动的来源。

控制层有两处看着像平滑的东西：`control/controller.py` 的 `_last_accepted_q` 与每帧关节步长门禁（`_max_joint_step_deg`，默认 `2.0`），以及 lead clamp（`control/api.py` 传 `30.0`）。它们的作用都在帧内，是安全门禁与截断，不碰帧与帧之间的序列；也不按 metadata 版本切换，不属于本款范围。本条决策不改动它们。

### 后果

正面：V11 判据第三款按「不做」收口，不再挂「待决定」。capability 层继续保持逐帧无状态，v1 帧与 v2 帧走同一条解析与换算路径，行为可预测。`tests/test_capability.py` 里三款既有的 v1 用例（`test_v11_v1_frame_parses_and_conversion_falls_back_by_field`、`test_v11_consecutive_frames_are_parsed_independently_no_smoothing`、`test_v11_no_frame_smoothing_primitive_in_the_package`）的身份从「缺口守卫」改为「取舍的守卫」：它们此后防的是将来无声引入跨帧平滑。

已知代价：如果现场出现 v1 帧抖动引发的控制问题，本库不做补偿，需要在调用侧处理。抖动若来自对端时序，本库也不会暴露任何可调的平滑参数。

触发重新评估的条件（达成任一条即重新评估）：① 出现需要平滑的真实需求，例如观测抖动导致控制不稳，且有现场记录能证明抖动幅度与频率；② 对端给出明确以版本号区分的平滑契约。届时先定平滑对象（观测帧 / 命令序列 / 换算结果），再落实现与测试，不凭旧术语开工。

相关提交：本轮文档提交（无代码提交）。

## D16 lerobot 依赖声明加版本区间：只支持 `lerobot>=0.6.1,<0.7`

状态：已接受（自 v0.7.0 起）

### 背景

**订正（2026-11-05 实测）**：本段原写「上游不在 PyPI 发布、`pip install .[runtime]` 解析不到分发地址」，与实测不符，按实测改写；后文「依据」里下限 `0.6.1` 取自厂商 fork `pyproject.toml` 的论证不变。

D1 把 `lerobot` 放在 optional extra `runtime` 里，并且有意只写名字、不写版本。2026-11-05 实测：从 PyPI **能**解析到 `lerobot 0.6.1` 并装成功、零报错；但它拉 `numpy>=2.0,<2.3`，把环境里已装的 `numpy 2.5.4` **静默降级到 2.2.6**，两边共存、没有任何告警；该上游 wheel **不含 `alohamini` 模块**，所以它能装、但不能当运行时用——**能装 ≠ 能用**，运行时必须是 vendor fork（`liyiteng/lerobot_alohamini`）。这条决定留下一个缺口——「只声明名字」等于「装哪一版都算合法」。接口不匹配要到运行时才暴露；两个 parity 文件（`tests/test_control_api_lerobot_parity.py`、`tests/test_kinematics_lerobot_parity.py`）需要 vendor fork 才能跑，在当前环境里始终跳过，也起不到约束作用。

依据：下限 `0.6.1` 取自厂商 fork 的 `pyproject.toml` 里 `version = "0.6.1"`（由上游提交 `8a74e0ac`「chore(dependencies): Bump lerobot to 0.6.1 (#3957)」引入），并且这正是 Pi 现场运行的版本线（`/home/pi5/lerobot_alohamini` HEAD `ec65318f`）。上限 `<0.7` 取自上游没有「不破坏兼容」的承诺：项目 classifier 仍为 `Development Status :: 3 - Alpha`，历史上 0.3→0.6 出现过接口重构，跨 minor 一律未验证。

### 决定

`pyproject.toml` 的 `runtime` extra 从 `["lerobot"]` 改成 `["lerobot>=0.6.1,<0.7"]`，约束落在 extra 上；核心 `dependencies` 仍不含 `lerobot`，D1 的「核心零第三方包可导入」不变。**订正（2026-11-05 实测）**：原文写「`lerobot` 不在 PyPI 发布、需按上游 README 自行安装」。按实改写后保留的是「运行时要 vendor fork」这条依据，不是「PyPI 上没有」这条已被实测推翻的描述——PyPI 上的 `lerobot 0.6.1` 能装、但缺 `alohamini` 模块，运行时仍必须按 fork README 安装 vendor fork。版本区间只额外限定范围、不改变安装方式。

新增模块 `src/alohamini_cartesian/adapter/lerobot_version.py`，提供三个公开函数与四个常量，只用标准库（其中 `check_lerobot_version` 是纯函数，`installed_lerobot_version` 读包元数据、有 I/O，`assert_lerobot_version_supported` 是入口）：

- `check_lerobot_version` 校验一个版本串，区间内返回去掉首尾空白的写法。
- `installed_lerobot_version` 读环境里的版本，只调 `importlib.metadata.version("lerobot")`，不 import `lerobot`（导入本模块不会把 `lerobot` 拉进 `sys.modules`）。metadata 用属性访问方式调用，测试才能打桩。
- `assert_lerobot_version_supported` 是入口：不传参读 metadata，传了显式版本就不再碰 metadata。

区间判定按**整数段**比较，不按字符串比：`0.6.10 > 0.6.9` 是整数段比较的结果；判别力最强的是 `0.60.0`——字符串比较下 `"60" > "6"`（不被下限拦）且 `"60" < "7"`（不被上限拦），会被错误接受，整数段比较下 `60 > 7`，正确判拒。`0.10.0` 作为越界回归项保留，但它不具判别力：字符串比较下会先撞下限（`"10" < "6"`），同样判拒。预发布日期**排序**按 PEP 440 纯序（`a` < `b` < `rc` < 同号正式版）；**成员判定**则比 PEP 440 的 `<` 更严：上界连同它的预发布一律判拒，`0.7.0rc1` / `0.7.0a1` / `0.7.0b2` / `0.7rc1` / `0.7a1` 都不通过，不因为排序上 `0.7.0rc1 < 0.7` 就放行——声明窗口 `<0.7` 在 PEP 440 下排除它们（`packaging.SpecifierSet(">=0.6.1,<0.7").contains("0.7.0rc1")` 为 `False`；上述实测取自 packaging 26.3，`<V` 不接纳 V 自身预发布的结论由 PEP 440 规则给出、与 packaging 版本无关），检查不得比声明宽松。下界一侧不受这条收紧影响：只接纳同号正式版及其以上的预发布（`0.6.1rc1` 判拒、`0.6.9rc1` / `0.6.9a1` 通过）。

接受的写法只有 `X.Y` / `X.Y.Z` + 可选 `aN` / `bN` / `rcN`（小写）；数字段必须是 ASCII 十进制、无前导零（`0` 本身合法）、每段至多 9 位，`0.6.01`、`0.06.1`、`00.006.0001`、`٠.٦.١`（阿拉伯-印度数字）、`０.６.１`（全角数字）与几千位的数字段一律 `InvalidVersionString`。长度检查写在正则里、先于整数转换，所以 5000 位的数字段不会漏成 `int()` 的裸 `ValueError`（`Exceeds the limit (4300 digits)`）。其余（`v0.6.1`、`0.6.x`、`0.6.1.post1`、`0.6.1.dev2`、`0.6.1+local`、`0.6.1.2`、`0.6.1rc`、空串、`latest`）同样判非法。

本条取代 D1 与 D6 正文中 `runtime = ["lerobot"]` 的字样（D1、D6 原文按「已接受的条目不回改」保留不动）。

三种失败各有专门异常，都不静默降级、不返回默认值：无法解析抛 `InvalidVersionString`（`ValueError` 子类），越界抛 `LerobotVersionUnsupported`（`RuntimeError` 子类），缺包抛它的子类 `LerobotNotInstalled`，因此缺包也能被 `LerobotVersionUnsupported` 一并捕获。越界消息带出实测版本与要求区间（`>=0.6.1` 与 `<0.7`）；缺包消息只有要求区间（缺包时没有版本可取）与安装指引；非法版本串的消息带出输入原文。非 `str` 入参（`None`、`6.1`、`bytes`、`list`）也走 `InvalidVersionString` 并带 `repr`，不漏成 `AttributeError` / `TypeError`；`assert_lerobot_version_supported(None)` 的 `None` 是「读 metadata」语义，不受此条影响。

### 后果

正面：接口依赖从「只有名字」变成「有区间、有判据」。`tests/test_lerobot_version.py` 同时钉住常量取值、`pyproject.toml` 的 `runtime` extra 与 ADR 里那条 requirement：前两处是结构化断言，改写与删除都会红；ADR 那条只是子串存在性检查（`lerobot>=0.6.1,<0.7` 是否出现），能挡改写与删除，挡不住「D16 之外另加一条相反表述」，防漂移的最后一环仍靠人读。检查全程离线、不需要装 `lerobot`，也不会因为检查而把机器人运行时载入进程。三条失败路径都是显式异常，调用方不会拿到一个假版本号继续往下跑。另（2026-11-05 实测）：parity 对照只对 vendor fork 成立——fork `6aee1f81` 快照上两个 parity 文件 **61/61 通过**（`aba8428` 修复 parity 夹具后）；从 PyPI 装的上游 wheel `lerobot 0.6.1` 不含 `alohamini` 模块，装上也跑不了这两条对照。

已知代价：

1. 现场指纹 JSON（`evidence/s4-20261025/capture/fingerprint-final-20261025.json`）里没有软件版本字段。所以「Pi 上跑的是 0.6.1」是从源码树 `pyproject.toml` 推出来的结论，**不是采集到的字段**，也没有现场校验记录。
2. 区间内的未来 patch（如 0.6.9）仍可能改接口，区间约束挡不住 patch 级的不兼容。
3. 检查函数目前没有被任何运行时入口调用：`connect` 之前没有自动校验，只有测试直接调用它，所以「装了不兼容版本」现在不会在装配期被拦下。
4. 本库声明的 `numpy>=1.26` 与 `lerobot` 要求的 `numpy>=2.0,<2.3` 不同。**订正（2026-11-05 实测）**：原写「开发环境因此不装 `lerobot`」是一刀切说法——实测不是装不上，而是**装得上、但会把已装的 numpy 静默降到 2.2.6**（环境里原为 2.5.4，两边共存、无任何告警），版本漂移无声，本机因此仍选择不装；端到端只有 mock 层证据：`importlib.metadata.version` 被打桩，真实 vendor 环境的版本串形态未验证。

触发重新评估的条件（达成任一条即重新评估）：① Pi 现场换到 0.7 及以上，或厂商 fork 的 `version` 越过 0.6.x；② 出现需要跨 minor 的接口变化并有现场需求；③ 现场采集到真实的 `lerobot` 软件版本字段，与本文推断不符。

相关提交：本轮提交（新增 `src/alohamini_cartesian/adapter/lerobot_version.py`，改 `pyproject.toml` 的 `runtime` extra 与本条目）。

## D17 设计规范定为历史设计来源，本库权威以 ADR 与验收基线为准

状态：已接受（自 v0.7.0 起）

### 背景

设计规范 `/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini-cartesian-teleop/docs/alohamini-control/Alohamini-control-design.md` 位于已退役仓库（D4 冻结为只读参考），文件内自标 SSOT（唯一的权威来源）。本库文档长期与它并存：`docs/background.md` §6 曾写「冲突时以设计规范为准」，`AGENTS.md` 曾把它挂为待决事项，优先关系一直没有裁决。

核对之后发现三类实质错位，靠「谁先写的」定不下优先关系。

| 错位 | 设计规范的说法 | 本库现状 |
| --- | --- | --- |
| 路径与环境过期 | 控制层模块在 `src/lerobot/robots/alohamini/control/`（该文件 `:235-240` 的模块清单），示例在 `examples/alohamini/control_basic.py`，运行环境为 `Python 3.12+`（`:295`、`:302`） | 本库是独立包 `alohamini_cartesian`，src layout，控制层模块在 `src/alohamini_cartesian/control/`（D1） |
| Gate A–E 与 D018 归档 JSON 在本库无承载物 | 设计规范 D008 定 Gate A–D（离线）与 Gate E（受控试动）两道门槛；D018 把归档格式定为 `docs/alohamini-control/gates/<side>-gate-<letter>.json`，`schema_version` 为 1，由 `tools/alohamini_gate_record.py` 校验 | 验收改用 V01–V13（`docs/acceptance-baseline.md` §2.1）；按 Gate 归档的验收与归档工具已随 Gate 系统一起删除，归档格式无法产出 |
| V 编号不同义 | 设计规范的 `V05` 指断网保位的核验口径（D012），`V10` 指 Gate 资格门禁的归档格式（D018） | 本库 `V05` 是反馈异常拦截与时效守护，`V10` 是两道 Gate 资格门禁（判的是本库基准脚本里的 Gate C 与 Gate F 运行时门禁）。同名不同义，逐字对照会误读 |

### 决定

1. 设计规范定为历史设计来源与溯源参考，**不具现行权威**。
2. 本库权威为本文件（ADR）与 `docs/acceptance-baseline.md`。两份文档之间分工不变：取舍与代价看 ADR，验收看基线。
3. 设计规范与本库权威文档冲突时，**以本库为准**，差异按本文件格式新增条目记录，不回改设计规范正文。本条即该规则的第一次适用，差异记录就是本条。
4. 不修改退役仓库里的设计规范（只读参考，见 D4）：不迁入本库，不撤销其内部的 SSOT 标注。
5. 旧仓库按臂归档的 Gate A–E 退出历史，V01–V13 是唯一验收口径。两边 V 编号撞车时以本库为准，只在对照处标注，不做全局重编号。

### 后果

**（A）继承清单**

设计规范 D001–D018 中仍指导本库行为的条目共 16 条，全部照旧生效。下表编号是设计规范的编号，与本文件 D1–D19 不是一套；落点列引用本文件条目时写作「本库 D10」这类形式以示区分。

| 来源编号 | 内容 | 落点（本库对应位置/条目） |
| --- | --- | --- |
| 设计规范 D001 | 首版以受控运动验证为目标，不做完整自主规划与取放 | `docs/background.md` §1 目标、§5 边界首行 |
| 设计规范 D002 | 机械臂控制原点取该臂肩底座 | `src/alohamini_cartesian/kinematics/assets/am_arm200/kinematics.urdf` 的根链接为 `left_Base`，六关节链从该链接起算；专项 ADR 与验收条目未逐条对号 |
| 设计规范 D003 | 底盘、升降与机械臂分阶段互锁 | V03；`src/alohamini_cartesian/control/commands.py` 的 `ControlStateMachine` |
| 设计规范 D004 | 停机优先，停稳后条件恢复，普通受限可有界反向退让 | V04（停止优先与条件恢复）；V06 与 `control/controller.py`（过流 hold 的有界退让） |
| 设计规范 D005 | 保留 Host 升降初始化，PC 端不额外对机械臂归位 | V07（`connect(...)` 不发动作、不求解 IK） |
| 设计规范 D006 | 姿态软约束 weight=0.01，优先位置精度 | V08；`src/alohamini_cartesian/control/arm_ik.py` |
| 设计规范 D007 | 复用底层硬件保护，引入只读 `control_status` 回传 | V06；`control/controller.py` 按 `_control_status.joint_holds` 做过流退让 |
| 设计规范 D009 | 交付主体为 Python 控制库，键盘降为上层调试示例 | `docs/background.md` §4.3：Python 控制库 SDK 行「已实现」，KeyboardTeleop 行「部分」（离线示例已入库，键盘调试层未实现） |
| 设计规范 D010 | IK 解耦为独立组件，增加离线基准保障 30 Hz 无卡顿 | V13；`benchmarks/alohamini_ik/benchmark_arm_ik.py`；预算判据的形态见本文件 D13 |
| 设计规范 D011 | 扩展直线步进（`move_to` / `move_by`）与升降、底盘接口 | `src/alohamini_cartesian/control/api.py` 的 `ArmHandle.move_to` / `move_by`、`LiftHandle`、`BaseHandle`；`docs/background.md` §4.3「Base / Lift action」行 |
| 设计规范 D012 | V05 验收口径以 Host watchdog 既定行为为准（超时后位置保位、底盘清速、停升降） | 本库无「断网保位」条目，未逐条对号。本库 V05 与之同名不同义：它判的是陈旧或冻结反馈的拦截与时效守护，与断网保位无关 |
| 设计规范 D013 | 命令间隙保活由调用方负责，Client 不内建心跳 | `control/controller.py` 的 `keepalive_interval_s`（默认 `None` 即关闭）与 `control/api.py:304` 的到期重发；周期由调用方传入 |
| 设计规范 D014 | 底盘显式零速，不依赖 watchdog 停车 | `control/controller.py` 的 `_tick_base_gap`（显式零速，`ACTIVE(ARM)` 期间除外）；V09 底盘 `stop` 的 dry-run 零发送用例 |
| 设计规范 D015 | 异常契约：运动类拒绝抛 `MotionBlocked`，`resume()` / reset 与 `stop()` 不抛 | `control/commands.py` 的 `MotionBlocked` 与 `ControlStateMachine`；`control/api.py:62` 的注释直接引 D015 |
| 设计规范 D016 | FAULT 语义：反馈序号倒退锁存，唯一恢复入口 `reset()`，禁止 FAULT 直跳 READY | `control/commands.py` 的 FAULT 锁存与 `ControlStateMachine.reset`；V05 行引的 `test_b52_sample_id_regression_from_active_faults` 与 `test_t_c12_sample_id_regression_from_non_active_state_faults` |
| 设计规范 D017 | 命令构造期校验，禁止静默 clamp | 本库 D10（限位取值在 `ArmIK.solve()` 入口校验，违规抛 `ValueError`）；`control/commands.py` 的 frozen 命令构造期校验（例如零时长加非零速度报 `ValueError`，`commands.py:132-133`） |

**（B）被本库取代或另定**

- 设计规范 D008 与 D018（Gate A–E 门槛与归档 JSON 格式）→ 归档，验收改 V01–V13。理由：归档工具随 Gate 系统删除，格式无法产出，门槛没有可核对的载体。
- 设计规范未涉及、由本库自定的五条：本库 D7（厂商侧协议测试不迁入）、本库 D10（关节限位入口校验，违规抛 `ValueError`）、本库 D12（客户端能力缺失判退出码 9）、本库 D15（不做帧平滑回退）、本库 D16（`lerobot` 版本区间）。这几条不因本 ADR 失效。它们不与设计规范冲突，属于设计规范没讲的空白处，由本库自己定。

**已知代价**

退役文件里仍自标 SSOT，单独读它的人可能误判优先级；纠正只能靠本 ADR 与 `docs/background.md` §6。另有一条：V 编号撞车没有做全局重编号，只在对照处标注，读旧文档时仍需人工分辨哪个 V05 是哪一边的。

相关提交：本轮文档提交（无代码提交）。

## D18 tracking 模式的 safety 快照单独成 sidecar：9 列长表，不进 14 列主表

状态：已接受（未发版：`pyproject.toml` 仍为 `0.7.0`、`git tag -l` 只到 `v0.6.0`，本条随本轮改动落地但不升版；`docs/changelog.md` 已补记（v0.7.0 段））

### 背景

`benchmarks/alohamini_io/measure_latency.py --mode tracking`（受控试动）能回答「这一步走没走到」，回答不了「目标有没有在半路被兜住」。厂商 Host 的安全载荷 `latest_safety_status` 里带着每个电机的电流、请求目标、采纳目标与 hold 标记（读法与键空间见 `src/lerobot/robots/alohamini/alohamini.py:983-988`），是查「肩部目标是否被过流 hold 扣住」的直接材料，而 tracking 主 CSV 一个字段都没记它。

两条并列约束决定了落点：

- 主 CSV 是 14 列、一行一步（`TRACKING_CSV_FIELDS`），`evidence/s4-20261025/tracking/` 下五份现场证据文件与它们的 report 都按 14 列读。改列数等于让既有证据文件与下游读法同时失效。
- D12 已定：`latest_safety_status` 与 `latest_control_status` 是不同帧、结构不同，禁止互相顶替。safety 面要进来，只能落在自己的列上，不能借道任何既有列。

### 决定

1. **单独成 sidecar（贴着主文件写的副产物）**。`--mode tracking` 在同目录额外写一份 `<主文件名>-safety.csv`（`safety_csv_path`，`benchmarks/alohamini_io/measure_latency.py:1256`）：与主 CSV 同目录，主文件名去掉尾部 `.csv` 再加 `-safety.csv`；名字里的点保留（`a.b.csv` → `a.b-safety.csv`），不以 `.csv` 结尾的名字也照加。主 CSV 仍是 14 列，`--mode rtt` 不产生 sidecar。
2. **9 列，顺序固定**（`SAFETY_CSV_FIELDS`，`:606`）：`step_index`、`sample_sequence`、`t_monotonic_s`、`motor`、`current_ma`、`requested_target`、`accepted_target`、`hold_active`、`target_source`。`target_source` 是末列，追加在 `hold_active` 之后；原有 8 列的顺序与含义一字未动（主 CSV 与既有证据文件按位置读，只有 sidecar 长了这一列）。加这一列的理由：`hold_active` 在钉住的 vendor 客户端上**结构上恒为 `False`**（`_joint_hold_goal` 只有 `alohamini.py:298` 的初始化与 `:985`/`:1076`/`:1088`/`:1128`/`:1131` 的读取/pop，无写入点，故 `joint_holds` 恒为 `{}`），因此它答不了「肩部是否被扣住」；判「是否被扣住」以 `target_source`（`none` / `command` / `watchdog` / `protection`）与 `current_ma` 的走势为准。取值口径：`target_source` 是**整帧属性**（`face["target_source"]`，不是逐电机属性），同一帧的每一行填同一个值；缺失、读不出、或值不是这四个字符串之一（非字符串、`bool`、数字、列表、未知字符串一律算）时记 `None`，即空单元格——不猜、不补默认值、不写空串。空格是「这帧没说到」，`none` 是「这帧说没有层产出目标」，两者不得混读。降级行（`currents_ma` 无电机表 → 唯一 `motor=None` 行）也填帧值，因为它不是逐电机属性。
3. **长表，一帧一电机一行**，不摊成宽表。理由：厂商帧的电机集合由帧自己决定（`safety_frame_rows` 按 `face["currents_ma"]` 的键集合出行），宽表得预先知道列名；帧只带 3 个电机时宽表要么留空列、要么改表头。代价对比：

   | 形态 | 行数 | 加一个电机的代价 | 加一个指标的代价 | 本库的选择 |
   | --- | --- | --- | --- | --- |
   | 长表（每帧每电机一行） | 帧数 × 电机数 | 每帧多一行 | 加一列 | 采用 |
   | 宽表（每帧一行，电机与指标进列名） | 帧数 | 加 4 列（该电机的 4 个值列） | 加 18 列（每个电机一列） | 不采用 |

   9 列不是「每个电机 9 列」：其中 3 列是帧级列（`step_index` / `sample_sequence` / `t_monotonic_s`，宽表里也只有这一组，不随电机增加）、1 列是电机名列（`motor`，宽表里进列名、不单独占列）、`target_source` 也是帧级列（整帧同值，宽表里同样只占一列），真正随电机增加的只有 `current_ma` / `requested_target` / `accepted_target` / `hold_active` 这 4 个值列。所以宽表加一个电机是加 4 列；加一个指标才是每个电机各加一列（18 电机 = 18 列）。

4. **只记新帧**：sidecar 复用 tracking 既有的陈旧检测，只在该次观测的 `observation_sequence` 增长时记行，缓存重复帧不重复记；`_run_tracking_mode` 正式跑之前那次 preflight 观测（读序号取基线）不计行。取舍是「不引入第二套去重判据」——代价是 sidecar 的帧数定义跟着主循环走，主循环判陈旧的口径变了，sidecar 的帧数也跟着变。
5. **时间列**：`t_monotonic_s` 优先取该帧到达时刻（`latest_request_timing["received_monotonic_s"]`，`_arrival_monotonic_s`，`:2429`），取不到回落到该次观测的轮询时钟读数；不新增时钟读取，所以节拍、settle 预算与达成率仍用改动前那几次读数计算。
6. **键空间差异**：`currents_ma` / `joint_holds` 的键是裸电机名，`requested_targets` / `accepted_targets` 的键是 `<motor>.pos`（两者分别由 `_arm_goal_positions` / `_arm_sent_positions` 填入，`alohamini.py:987-988`）。`_safety_target`（`:2502`）两种形状都认；同一电机两键并存时**裸名优先**，因为 `motor` 列打的就是裸名，行要由它自己的名字描述。`motor` 已带 `.pos` 时不再二次加后缀。
7. **空单元格，不补 0**：`current_ma` / `requested_target` / `accepted_target` 只在能转成有限数值时写数值，否则留空；`bool` 不算数值（`float(True) == 1.0` 会凭空写出一条 1 mA 的电流）。`0 mA` 读起来像「测到的零电流」、`0` 目标读起来像「Host 采纳过 0」，两者都是伪观测，所以拒绝填充。
8. **降级判据用整轮计数**，不用「开局探一次」：能力缺失（分支 A，点名缺的属性）与内容不完整（分支 B，点名 `latest_safety_status` 并带 `frames=<int>, frames_with_motor_table=<int>`）两个分支互斥，所以**降级**警告一次运行最多一条（sidecar 写盘失败的警告是另一件事实，见第 9 条，它会在降级警告之外再加一条）；退出码不变，sidecar 照写。理由：部署的客户端属性一直存在（初值 `{}`），只会内容为空，所以「某一时刻探一次」看不到「4 帧里坏 3 帧」这种形态；整轮计数把坏帧数点出来。
9. **写盘失败只警告**：sidecar 的 `OSError` 打一条 stderr 警告，不落 `EXIT_WRITE_FAILED`——主 CSV 才是主产物，判据顺序（hold 失败 > 写失败 > 中止）只看主 CSV；中断分支尽力重写一次（`after_interrupt=True`）。
10. **与 D12 的关系**：本 sidecar 给 safety 面单独开列，不顶替任何东西；`latest_control_status` 的读取路径一行未改，`SAFETY_CLIENT_CAPABILITIES` 与 `CLIENT_CAPABILITIES` 是两个并列元组，任一面缺失都不会被另一个面的读数掩盖。
11. **时间戳回落要点名，整轮恰好一条**：一次运行里只要有至少一个记录帧的 `t_monotonic_s`（或 90% 交叉点的到达时刻）落到轮询时钟，`run_tracking_steps` 就在 `summary["timestamp_fallback"]` 里落下这个事实，由 `_run_tracking_mode` 在 stderr 打一条提示：本轮**至少一处**时间戳是轮询时刻而非客户端到达时间戳——可能是 sidecar 行的 `t_monotonic_s`，也可能是 90% 交叉点的到达时刻，不必然每一处都回落；原因是 `latest_request_timing` 答不出可用的 `received_monotonic_s`。整轮一条、不是每帧或每次轮询一条（回落是运行级事实，不是帧级事实），一个回落都没发生时一条都不打。这条提示是独立的 `if`，不塞进那两个 safety 降级分支的互斥链：safety 降级的两条彼此是 `if` / `elif`，互斥，一次运行最多一条；timing 提示与 safety 降级那一对**互斥分支**互不抑制、可区分，两者可以同时出现（timing 提示讲 `latest_request_timing`、不含 `latest_safety_status`）。点名不改变退出码、abort 判定、节拍、settle 预算与达成率：它只描述 sidecar 的 `t_monotonic_s`，以及主 CSV 里 `received_monotonic_s` 留空、`rise_90_time_s`（及报告里的 `rise_90_ms`）改用轮询戳这件事。

### 后果

正面：tracking 的现场证据从此能回答「电流多少、目标有没有被改、hold 表说了什么」，而 14 列主表与既有五份证据文件的读法一字不改。空单元格与不补 0 的纪律把「缺值」与「测到 0」分开，读表的人不会把伪观测当测量。降级只降 sidecar 的内容，不影响退出码与主产物。

**已知代价**

1. **无真机验证**：sidecar 只有离线测试覆盖（`tests/benchmarks/test_measure_latency_safety.py`，假 client），真机上的帧形态、`motor` 空行占比与行数从未观测。
2. **Pi 上 `t_monotonic_s` 是轮询时刻，不是到达时刻**：厂商客户端只有 `latest_host_timing`、没有 `latest_request_timing`（`docs/ADR.md` D12），`_arrival_monotonic_s` 取不到到达戳，回落到该次观测的轮询时钟。回落已在 stderr 点名（第 11 条），但触发有条件：Pi 的 vendor 没有 `latest_request_timing`，所以**只要这轮至少记录到一帧，这条提示就会出现**。`timestamp_fallback` 的置位有两处，两处都在 `run_tracking_steps` 里：由 **sidecar 行**触发的那一支只在新帧分支里走（条件 `sequence > last_sequence` 在 `measure_latency.py:3020`，置位在 `:3029-3031`）；另一支在 90% 交叉点（交叉点判定在 `:3142`，置位在 `:3146-3148`），与 `observation_sequence` 无关，陈旧帧上也会走到。因此「在记录到任何新帧之前就中止 → 不会打这条提示」只在**不含交叉点置位**的路径上成立，不是无条件结论。它说明的是「这一列是什么」，不是「这次运行有问题」，不要按故障处理。
3. **`hold_active` 在现场恒 `False`**：钉住的 vendor（本地 fork HEAD `ec65318f`）里 `_joint_hold_goal` 没有写入点（`alohamini.py:298` 初始化，`:985`/`:1076`/`:1088`/`:1128`/`:1131` 只读或 pop），`joint_holds` 恒为 `{}`。`False` 的诚实含义是「可读的 hold 表没列这个电机」，**不是**「确认没被扣过」；因此该列不能用来判「肩部目标是否被过流 hold 扣住」，全列 `False` 不得读成「没被扣」。替代判据是 `target_source`（`protection` 即保护路径接管）与 `current_ma` 的逐帧走势，再加 `requested_target` 与 `accepted_target` 是否相等。
4. **整轮计数会把单帧偶发损坏也打成一条警告**：一帧坏、其余全好也告警，可能被读成噪音。
5. **内存与 CSV 体量随「帧数 × 电机数」增长**：行数 = 记录帧数 × 电机数（18 电机即 ×18），帧数随 `--steps`、`--settle-timeout-s`、`--control-hz` 增长；行在内存里攒到跑完再写盘，与主 CSV 同构。按 50 Hz × 每步 1 s 的 settle 预算算术推算，20 步约 1000 帧 × 18 电机 ≈ 1.8 万行（推算，非实测）。
6. **以下键有意不记**：本 sidecar 只覆盖 9 列，`latest_safety_status` 里的下列键本轮**有意**未记，每条写明为什么不记、以及它可能是谁的证据。
   - `joint_hold_events`（`alohamini.py:984` 发的是 `self._joint_hold_events`）：不记是因为它是全轮累计计数、不分电机，长表的行放不下一个整轮量；它是「这轮发生过关节 hold」的证据，能把「从没 hold 过」与「hold 过又释放了」分开。
   - `watchdog_active`（`alohamini_host.py:386`，初值 `:257`）：不记是因为它是逐帧布尔、与 `hold_active` 同形但语义是命令看门狗接管而非关节 hold 表，混进同一列会把两种「扣住」读成一种；它是「目标被 watchdog 接管」的证据。
   - `watchdog_events`（`alohamini_host.py:387`，初值 `:258`，`:356` 累加）：不记是因为它同样是全轮累计计数、按事件而非按电机；它是「看门狗这轮触发过几次」的证据。
   - `control_owner` / `control_epoch`（`alohamini_host.py:390-391`；`alohamini_client.py:665` 只在键存在时读，所以是「若宿主发布」）：不记是因为它们是控制权归属而不是安全面测量值；它是「同一轮里有另一个客户端抢过控制权」的证据——若 `requested_target` 与主 CSV 的 `cmd_units` 对不上，这一对键能区分「Host 自己改了目标」与「别的客户端下了目标」。
   - `gripper_holds`（`alohamini.py:986` 发的是 `self._gripper_hold_goal`）：**与 `joint_holds` 不同，厂商在 `alohamini.py:1040` 有写入点**（`_limit_gripper_goal_by_current` 里写 `self._gripper_hold_goal[motor] = hold_goal`），所以现场很可能非空，而 `joint_holds` 恒空。不记是因为它与本次要回答的问题（肩部目标是否被扣住）无关，混进同一列还会让读者以为关节也能 hold。它可能是「夹爪接触后按电流 hold 住」的证据。它不影响 `hold_active`：那一列只读 `joint_holds`（`_safety_hold_flag`），夹爪 hold 不会让它变 `True`。

触发重新评估的条件（达成任一条即重新评估）：① 现场首跑后 `current_ma`、目标列或 `target_source` 的形态与本文描述不符；② 厂商换到有 `_joint_hold_goal` 写入点的版本，`hold_active` 重新具备判读价值。

相关提交：随本次 `feat:` 提交同批落地（提交哈希见 `git log -- docs/ADR.md`）。代码改动在 `benchmarks/alohamini_io/measure_latency.py`，与本条文档同批提交。

## D19 证据红线：V01 不收口不发 minor 版，每个发版留一格实机证据

状态：已接受（未发版：`pyproject.toml` 仍为 `0.7.0`、`git tag -l` 只到 `v0.6.0`；`docs/changelog.md` 尚未记本条，留待发版时补）

### 背景

2026-11-05 定下这条红线。为什么现在定：本阶段的发版依据一直缺一条硬约束。

V01 是本阶段唯一挡住「能不能说不接 leader 就算通」的条目（leader 指遥操作主臂，V01 要求脱离主臂、纯 PC 侧控制）。现状是实机侧只有腕部一个关节拿到跟随证据（2026-10-25 `arm_left_wrist_roll` 达成 `87.9%`、两步 `settled=True`；2026-11-05 同一关节 `87.91%`，与上次逐字相同），肩部两轴不跟随（2026-11-05 `arm_left_shoulder_pan` `4.40%`、`arm_left_shoulder_lift` `0.00%`，三轴 `settled=False`），原因未定性。数字出处：`docs/field-results.md` 的 2026-11-05 节「达成率三轴只有腕部接近指令」一行。

没有红线时容易把「离线全绿」当成发版依据：本机 `pytest tests` 报 `1453 passed, 2 skipped`，那是软件侧快照；跳过的 2 条是 vendor fork 的 parity 对照，跳过不等于差分对照通过；实机侧 13 条里 9 条仍是「未取」。测试数增长不构成「能力可用」的证据。

### 决定

四条红线，按本文字面执行，不放宽。

1. **V01 不收口不发 minor 版**：`0.7.x → 0.8` 需要 V01 有实机证据才能发版。
2. **每个发版至少新增一格实机证据**，并写明条件（供电 / 温度 / 容差）。**例外**：纯文档 / 测试修复的发版可豁免，豁免必须在 changelog 里写明理由。
3. **V01 收口前新增测试只允许走追因路径**（诊断 / 证据链），**不允许 polish 路径**（打磨体验、补覆盖广度、美观化断言）。
4. **配套 A 阶段「受阻」口径**：肩部若定性为硬件不可修，A 阶段记「受阻」而非「完成」，V01 保持未取，按臂 / 关节资格档案降级推进 B（与 `docs/acceptance-baseline.md` §2.1 互相引用）。

口径衔接：红线 4 替代 `docs/acceptance-baseline.md` §2.1 旧写法里的逃逸口——旧写法允许把「在手册里写明结构性原因」当作 A 阶段的交差方式；新口径要求先定性到硬件不可修才记「受阻」，且明确「受阻 ≠ 完成」，V01 不因降级而算通过。

### 后果

正面：发版判断有了可核对的落点，不再靠「测试数涨了多少」推断能力；每条实机证据都带条件（供电 / 温度 / 容差），下次现场不必重新推当时的口径；「受阻」这条出口让肩部卡住时仍能推进 B 阶段，同时不把未达成的验收条目记成完成。

**已知代价**

1. 红线 3 会限制 V01 收口前的测试增补类型：只允许追因（诊断与证据链）的用例，正常润色型改动（补覆盖广度、改断言可读性、补边界美观度）要推迟到 V01 收口之后。代价是这期间测试集质量提升被冻结，个别读起来不顺的断言要带着走。
2. 红线 2 的豁免依赖 changelog 里那句理由：写了理由才算豁免，缺理由等于违规自行判断——这条红线拦得住「什么都不写就发版」，拦不住「理由写得很宽松」，防漂移的最后一环仍靠人读 changelog。
3. 红线 1 把版本节奏绑在实机窗口上：现场窗口不可得时（例如远程会话无可控断电通道、无人在机器人现场），minor 发版就被挡住，只能留在 `0.7.x`。这是有意取舍——离线改动照旧可以走 patch 与文档，只是不换 minor。
4. 红线 4 的降级不是免费出口，但「受阻」不等于「已定性为硬件不可修」：记「受阻」指**当前无法推进**（例如必须有人员到场才能做 teleop / M1 定性），它不等于「已定性为硬件不可修」，更不等于「完成」，**V01 始终保持未取**。定性本身需要现场动作（M1 / teleop 观测）；只有经 M1 / teleop 定性为硬件不可修，才走「按臂 / 关节资格档案降级推进 B」这条出口。若始终无人到场，A 阶段就停在「受阻」，既不得记「完成」，也不得因为记了「受阻」就跳过 B 阶段的前置。

触发重新评估的条件（达成任一条即重新评估）：① V01 拿到实机证据并收口，届时重新判断红线 2 / 3 是否还要维持现有强度；② 发版节奏与现场窗口长期冲突，且冲突原因是外部不可控（断电通道、人手），需要把「每版一格实机证据」改为按阶段结算；③ 厂商给出过流事件接口或外接测量方案到位，V06 的取数方法从「未定」变成可执行，红线 2 里「实机证据」的范围需要重新划定。

相关提交：本轮文档提交（无代码提交）。同批改动：`docs/acceptance-baseline.md` 的 V01 判据、V06 / V13 实机证据格、§2.1 降级出口、§2.3 快照与 parity 措辞、新增 §2.4；`pyproject.toml` 的依赖注释订正。
