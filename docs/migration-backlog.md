# 迁移待办

本清单记录从旧仓库迁入的进度与剩余事项，数字来自对旧仓库的只读盘点。已迁入但没有现场证据的项也列在这里，状态写「已迁、待现场实测」。

| 项 | 现状 | 阻塞条件 | 计划 |
| --- | --- | --- | --- |
| `benchmarks/alohamini_io/capture_robot_fingerprint.py` | 已迁入本库 `benchmarks/alohamini_io/capture_robot_fingerprint.py`，473 行；配套测试 `tests/benchmarks/test_capture_robot_fingerprint.py`（638 行，静态 20 个测试函数，参数化展开后 23 条用例）已迁入，离线可跑 | 剩现场实测：需要 Pi 上常驻 Host 进程；对端 metadata 的 `schema_version` 必须为 2（Schema 2），否则脚本以退出码 6 退出（fail-closed，条件不满足就拒绝执行） | 剩现场实测，随 S4 现场窗口完成 |
| `benchmarks/alohamini_io/measure_latency.py` | 已迁入本库 `benchmarks/alohamini_io/measure_latency.py`，2151 行；配套测试 `tests/benchmarks/test_measure_latency.py`（3365 行，静态 110 个测试函数，参数化展开后 153 条用例）已迁入，离线可跑 | 剩现场实测：需要真机，Mode B `tracking` 会真动机器人；Gate F 要求 `schema_version == 2` | 剩现场实测，随 S4 现场窗口完成 |
| 现场 smoke（S4） | 未开始，需要一次到现场的窗口。范围已缩小：脚本迁移与配套测试在 v0.4.0 完成，不属 S4 | 需要 AlohaMini 2 Pro 真机、Pi 上 Host 常驻、PC 与 Pi 同网段、现场授权、Gate A–E 归档（一臂一 Gate 一 JSON） | 一次现场窗口内完成真机实测、Gate A–E 归档、配置样例采集 |
| 旧仓库厂商侧协议测试 `tests/robots/test_alohamini_cartesian_protocol.py` | 留在旧仓库，49 个测试函数 | 决定不迁，理由见 `docs/ADR.md` D7 | 不迁（见 D7） |
| 配置样例 | 未完成。D2 规定「模板文件 → 现场文件」两层 YAML 覆盖，两仓库都没有 YAML 实例文件，旧仓库靠 draccus 命令行参数传配置。这是缺口，不是已完成项 | 现场取值的真实样例要现场采集才可信 | S4 随现场 smoke 一起产出模板与现场两份样例 |

## 1 `benchmarks/alohamini_io/capture_robot_fingerprint.py`

现状：文件已在本库 `benchmarks/alohamini_io/capture_robot_fingerprint.py`，473 行。配套测试已在本库 `tests/benchmarks/test_capture_robot_fingerprint.py`，638 行，静态 20 个测试函数，参数化展开后 23 条用例。测试用 fake client（假的客户端对象顶上真连接）与依赖注入，可以离线跑，不碰 `lerobot`、`zmq`、`socket`、线程。

数字口径：旧仓库只读盘点的口径是约 469 行、20 个测试函数；本库口径是 473 行、20 个静态测试函数。行数差异未逐行核对，语义等价以 `ast.dump` 全等为准（四个文件 4/4）。

本轮已迁，迁移判断的依据是什么：原计划以「本机没有 Host，迁入后也只是一份跑不起来的代码」为由推到 S4。重新核查后，这条依据对脚本成立，对配套测试不成立。

- 脚本对 `lerobot` 的 import 全部在 `default_client_factory()` 函数体内（`capture_robot_fingerprint.py:140-141`），模块顶层只有标准库与 numpy。
- 配套测试只用标准库与 pytest，用依赖注入的 fake client。
- 两者在旧仓库跑不起来的唯一原因是旧仓库根 `tests/conftest.py` 顶层 import `lerobot`，pytest 收集测试时必然加载它。迁入本库后不再加载那个 conftest。

判断与取舍见 `docs/ADR.md` D9。

为过本库风格门做的豁免：`pyproject.toml` 的 `[tool.ruff.lint.per-file-ignores]` 给 `benchmarks/alohamini_io/capture_robot_fingerprint.py` 豁免 `D103, D107, E501`，给 `tests/benchmarks/test_capture_robot_fingerprint.py` 豁免 `ANN001, ANN201, ANN202, ANN204`。四条忽略都是精确文件路径，全局 `select` / `ignore` 未改。

仍未验证的现场项：脚本从未对着真机 Host 跑过。对端 metadata 的 `schema_version` 实际取值没有样本。退出码 3（连接失败）、4（样本不足）、5（写文件失败）、6（metadata 无效）的真实触发路径只在测试里验过，没有现场记录。产物目录是 `ARTIFACT_DIR = Path("artifacts") / "alohamini-io"`，相对当前工作目录，不硬编码绝对路径；这个目录的实际产出同样没有样本。

计划：S4 现场窗口内对着真机 Host 跑一遍取 fingerprint 的完整流程，把退出码与产物一起归档。

## 2 `benchmarks/alohamini_io/measure_latency.py`

现状：文件已在本库 `benchmarks/alohamini_io/measure_latency.py`，2151 行。配套测试已在本库 `tests/benchmarks/test_measure_latency.py`，3365 行，静态 110 个测试函数，参数化展开后 153 条用例。测试用 fake client 与 fake clock（假时钟，用来固定时间推进）与依赖注入，可以离线跑。

数字口径：旧仓库只读盘点的口径是约 2116 行、110 个测试函数；本库口径是 2151 行、110 个静态测试函数。行数差异未逐行核对，语义等价以 `ast.dump` 全等为准（四个文件 4/4）。

本轮已迁，迁移判断的依据是什么：原计划以「本机没有真机，Mode B 无法验证」为由推到 S4。这条依据对现场实测成立，对代码入库与离线测试不成立。

- 脚本对 `lerobot` 的 import 全部在 `default_client_factory()` 函数体内（`measure_latency.py:847-848`），模块顶层只有标准库与 numpy。
- 配套测试只用标准库与 pytest，不碰 `lerobot`、`zmq`、`socket`、线程。
- 两者在旧仓库跑不起来的唯一原因是旧仓库根 `tests/conftest.py` 顶层 import `lerobot`，pytest 收集测试时必然加载它。迁入本库后不再加载那个 conftest。

判断与取舍见 `docs/ADR.md` D9。

为过本库风格门做的豁免：`pyproject.toml` 的 `[tool.ruff.lint.per-file-ignores]` 给 `benchmarks/alohamini_io/measure_latency.py` 豁免 `D103, D107, E501`，给 `tests/benchmarks/test_measure_latency.py` 豁免 `ANN001, ANN003, ANN201, ANN202, ANN204, ANN205, D403, E501`。四条忽略都是精确文件路径，全局 `select` / `ignore` 未改。

仍未验证的现场项：

- Mode A `rtt`（不动机器人）从未对着真机跑过。
- Mode B `tracking` 会真动机器人，从未跑过。
- Gate F 是运行时门禁（决定这次测量能不能发命令的检查），要求 `schema_version == 2`；它的真实拒绝路径没有现场记录。
- `schema_version` 的实际取值没有样本。
- 退出码 1（step 响应中断）、3（连接失败）、5（写文件失败）、6（运动门禁拒绝）、7（样本不足）、8（hold 失败）的真实触发路径只在测试里验过。

计划：S4 现场窗口内两种模式都跑一遍；Mode B 要在现场授权与安全前提下执行。

## 3 现场 smoke（S4）

现状：未开始，需要一次到现场的窗口。范围已缩小：两个脚本的迁移与配套测试在 v0.4.0 完成，不属 S4。S4 只剩真机实测、Gate A–E 归档与配置样例采集。

为什么现在不做：这一项依赖现场条件，本机无法替代。条件是 AlohaMini 2 Pro 真机、Pi 上 Host 进程常驻、PC 与 Pi 在同一网段、现场授权，以及 Gate A–E 归档（一臂一 Gate 一 JSON）。

现场窗口要验的四件事：

| 编号 | 要验的项 | 判据 |
| --- | --- | --- |
| 1 | 真机 metadata 取值 | 取到对端 `_robot_metadata`，记录 `schema_version` 的实际取值（整数 1 或 2），这是本库第一条真实样本 |
| 2 | Mode A / Mode B 测量 | Mode A `rtt` 与 Mode B `tracking` 各跑一遍；Mode B 会真动机器人，要在现场授权与安全前提下执行 |
| 3 | Gate F 拒绝路径 | Gate F 是运行时门禁（决定这次测量能不能发命令的检查）；`schema_version` 不是 2 时是否按退出码 6 拒绝，要有一次真实触发记录 |
| 4 | 配置样例采集 | 按 D2 的「模板文件 → 现场文件」两层产出两份 YAML 样例，取值来自现场 |

本机无法验证的部分（脚本已迁入，但以下都没有证据）：真机动静态行为；Host 侧 metadata 字段的实际取值；时延与跟踪精度；Gate F 的实际拒绝路径；现场 overlay 能否完整表达差异。

计划：在一次现场窗口内把这四项一起做完，结果进入实测证据目录。

## 4 旧仓库厂商侧协议测试 `tests/robots/test_alohamini_cartesian_protocol.py`

现状：留在旧仓库，49 个测试函数。import 5 个厂商符号，是纯对象 harness，不碰 zmq、socket、线程，可以离线跑。

数字来源：旧仓库只读盘点，测试函数 49 个。

结论：决定不迁，理由写在 `docs/ADR.md` D7。这 49 条测的是厂商模块的行为，本库不含这些代码；本库对厂商内部协议不负回归责任。

计划：不迁（见 D7）。

## 5 配置样例缺口

现状：D2 规定「模板文件 → 现场文件」两层 YAML 覆盖，但两仓库都没有 YAML 实例文件。旧仓库靠 draccus 命令行参数传配置。这是缺口，不是已完成项。

为什么现在不做：样例里的取值要能代表现场，没有现场数据写出来的样例不可信。

计划：S4 随现场 smoke 一起产出两份样例，一份模板，一份现场。
