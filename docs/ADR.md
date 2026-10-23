# 架构决策记录（ADR）

本文记录 `alohamini-cartesian` 自 v0.1.0 起沿用的决策，编号 D1–D9。每条按背景、决定、后果三段记录，后果同时写正面收益与已知代价。状态写作「已接受（自 vX.Y.Z 起）」，不记具体日期。新的重大取舍按顺序追加编号，已接受的条目不回改。

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

`docs/migration-backlog.md` 原定把 `benchmarks/alohamini_io` 两个脚本推到 S4 现场，理由是「本机没有 Host，迁入后也只是一份跑不起来的代码」。v0.4.0 重新核查，这条依据对脚本成立，对配套测试不成立。

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
