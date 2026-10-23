# 迁移待办

本清单记录尚未从旧仓库迁入本库的内容，数字来自对旧仓库的只读盘点。

| 项 | 现状 | 阻塞条件 | 计划 |
| --- | --- | --- | --- |
| `benchmarks/alohamini_io/capture_robot_fingerprint.py` | 仍在旧仓库 `benchmarks/alohamini_io/`，约 469 行；配套测试约 20 例，用 fake client，本身可离线跑 | 需要 Pi 上常驻 Host 进程；对端 metadata 的 `schema_version` 必须为 2（Schema 2），否则脚本以退出码 6 退出（fail-closed） | S4 现场 smoke |
| `benchmarks/alohamini_io/measure_latency.py` | 仍在旧仓库 `benchmarks/alohamini_io/`，约 2116 行；配套测试约 110 例 | 需要真机：Mode B `tracking` 会真动机器人；Gate F 要求 `schema_version == 2` | S4 现场 smoke |
| 现场 smoke（S4） | 未开始，需要一次到现场的窗口 | 需要 AlohaMini 2 Pro 真机、Pi 上 Host 常驻、PC 与 Pi 同网段、现场授权、Gate A–E 归档（一臂一 Gate 一 JSON） | 一次现场窗口内完成 |
| 旧仓库厂商侧协议测试 `tests/robots/test_alohamini_cartesian_protocol.py` | 留在旧仓库，49 个测试函数 | 决定不迁，理由见 `docs/ADR.md` D7 | 不迁（见 D7） |
| 配置样例 | D2 规定「模板文件 → 现场文件」两层 YAML 覆盖，两仓库都没有 YAML 实例文件，旧仓库靠 draccus 命令行参数传配置。这是缺口，不是已完成项 | 现场取值的真实样例要现场采集才可信 | S4 随现场 smoke 一起产出模板与现场两份样例 |

## 1 `benchmarks/alohamini_io/capture_robot_fingerprint.py`

现状：文件仍在旧仓库 `benchmarks/alohamini_io/`，配套测试在旧仓库 `tests/benchmarks/test_capture_robot_fingerprint.py`，有 20 个测试函数。测试用 fake client（假的客户端对象顶上真连接），本身可以离线跑。

数字来源：旧仓库只读盘点，行数约 469 行，测试函数 20 个。

为什么现在不迁：脚本跑起来要在 Pi 上有一个常驻的 Host 进程（机器人侧的主程序），本机没有 Host。脚本还要求对端 metadata 的 `schema_version` 为 2（Schema 2），不满足就以退出码 6 退出，这是 fail-closed（条件不满足就拒绝执行，不做降级）。本机无法验证脚本能否真取到 metadata，所以迁入后也只是搬进一份跑不起来的代码。

计划：S4 现场 smoke 时迁入，并在真机环境实测一遍取 fingerprint 的完整流程。

## 2 `benchmarks/alohamini_io/measure_latency.py`

现状：文件仍在旧仓库 `benchmarks/alohamini_io/`，配套测试在旧仓库 `tests/benchmarks/test_measure_latency.py`，有 110 个测试函数。

数字来源：旧仓库只读盘点，行数约 2116 行，测试函数 110 个。

为什么现在不迁：脚本有两种测量模式。Mode A `rtt` 不动机器人；Mode B `tracking` 会真动机器人。Gate F 是运行时门禁（决定这次测量能不能发命令的检查），要求 `schema_version == 2`。本机没有真机，Mode B 无法验证，`schema_version` 的实际取值也没有样本。迁进来只能验证 Mode A 的代码路径。

计划：S4 现场 smoke 时迁入，两种模式都在现场跑一遍。

## 3 现场 smoke（S4）

现状：未开始，需要一次到现场的窗口。

为什么现在不做：这一项依赖现场条件，本机无法替代。条件是 AlohaMini 2 Pro 真机、Pi 上 Host 进程常驻、PC 与 Pi 在同一网段、现场授权，以及 Gate A–E 归档（一臂一 Gate 一 JSON）。

本机无法验证的部分：

- 真机动静态行为。
- Host 侧 metadata 字段的实际取值。
- 时延与跟踪精度。
- Gate F 的实际拒绝路径。
- 现场 overlay 能否完整表达差异。

计划：在一次现场窗口内把这些一起做完，结果进入实测证据目录。

## 4 旧仓库厂商侧协议测试 `tests/robots/test_alohamini_cartesian_protocol.py`

现状：留在旧仓库，49 个测试函数。import 5 个厂商符号，是纯对象 harness，不碰 zmq、socket、线程，可以离线跑。

数字来源：旧仓库只读盘点，测试函数 49 个。

结论：决定不迁，理由写在 `docs/ADR.md` D7。这 49 条测的是厂商模块的行为，本库不含这些代码；本库对厂商内部协议不负回归责任。

计划：不迁（见 D7）。

## 5 配置样例缺口

现状：D2 规定「模板文件 → 现场文件」两层 YAML 覆盖，但两仓库都没有 YAML 实例文件。旧仓库靠 draccus 命令行参数传配置。这是缺口，不是已完成项。

为什么现在不做：样例里的取值要能代表现场，没有现场数据写出来的样例不可信。

计划：S4 随现场 smoke 一起产出两份样例，一份模板，一份现场。
