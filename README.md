# AlohaMini Cartesian

AlohaMini 笛卡尔遥操作的产品侧库：对机器人运行时的适配协议、能力探测与单位换算裁决、配置覆盖。核心依赖为空。

## 为什么独立成库

旧仓库把产品代码与机器人运行时放在同一仓同一分支上。库的版本号跟着 vendor 分支漂移，产品改动与实机运行时版本互相绑死，实机换一次版本就得连产品一起动。

现场改动只能落进会被 vendor 更新的文件里。上游一覆盖，现场配置和补丁就没了；旧结构里没有 overlay 表达差异，差异只能直接改源码。

验证工具还把 metadata 的 `schema_version` 当成兼容判据。旧仓库 `benchmarks/alohamini_io/measure_latency.py` 的运动门禁只接受 `schema_version == 2`，遇到 schema 1 或字符串 `"2"` 直接拒绝，工具被版本号拦死，而真正该判的是这项能力在不在。

## 四项定案

| 编号 | 定案 | 要点 |
| --- | --- | --- |
| D1 | 依赖形态 | 独立包发布，`lerobot` 作为可选 `extra`（`runtime`），不使用 git submodule |
| D2 | 配置格式 | YAML 单文件加环境变量覆盖，顺序为默认值 → 模板 → 现场文件 → 环境变量 |
| D3 | 迁移范围 | 只搬 `control/`、运动学（`joint_kinematic_adapter`、`model_specs`）与 `benchmarks/`，分阶段搬，v0.1.0 不含 |
| D4 | 启动方式 | 新开仓库，当前状态作为 `v0.1.0` 起点，旧仓库冻结为只读参考 |

每条的背景、决定与代价见 [docs/decisions.md](docs/decisions.md)。

## 安装

核心零依赖，可在没有机器人、没有 `lerobot` 的环境里导入和测试。

```bash
pip install -e .
```

需要读 YAML 配置文件时加 `yaml` extra：

```bash
pip install -e ".[yaml]"
```

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

文件格式以 YAML 为主，需要 `yaml` extra 里的 PyYAML；`.json` 后缀走标准库，作为零依赖退路。按文件后缀分派，其他后缀抛 `ConfigError`。

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

## 公开 API

- `alohamini_cartesian.adapter.protocol`：`RobotTransport`（`get_observation`、`send_action`、`prime_observation_request_window(*, include_cameras)`）、`RobotSpec`（`robot_model`、`left_port`、`right_port`），两个 `typing.Protocol`。
- `alohamini_cartesian.capability.detect`：常量 `SPAN_BY_NORMALIZATION`、`SUPPORTED_METADATA_SCHEMAS`；异常 `CapabilityError`、`ConversionUnavailable`；数据类 `MotorInfo`、`Capabilities`、`Conversion`；函数 `detect_from_observation`、`resolve_deg_per_unit`。
- `alohamini_cartesian.config.overlay`：常量 `ENV_PREFIX`；异常 `ConfigError`；数据类 `AlohaMiniSettings`；函数 `load_settings`。

## 纪律

两条硬约束，改代码时先看这里。

1. `adapter/` 之外禁止 import `lerobot`。`lerobot` 只能通过 `RobotTransport` 与 `RobotSpec` 两个 `Protocol` 接触，其余模块不碰 `lerobot.motors`、`lerobot.robots.alohamini` 这类深层模块。
2. 能力探测不得把版本号当兼容判据。判据是这项能力是否存在，`schema_version` 只用来决定字段怎么解析。

第一条有机器守卫，不靠人工 review 维持。`tests/test_capability.py` 里有一条测试用 AST 遍历断言除 `adapter/protocol.py` 之外没有任何模块 import `lerobot`，并断言导入 `capability.detect`、`config.overlay` 后 `sys.modules` 里没有 `lerobot`。

## 当前状态与下一步

v0.1.0 已完成：仓库骨架、适配协议、能力探测与换算裁决、配置 overlay、71 条不依赖硬件的测试。

- S3（下一步）：迁移 `control/`、`joint_kinematic_adapter`、`model_specs`，以及本轮尚未搬运的 `benchmarks/`。
- S4：现场 smoke（实机加 vendor 运行时），验证 overlay 能否完整表达现场差异。

## 开发与测试

pytest 已配好 `testpaths`、`pythonpath = ["src"]` 与 `addopts = "-q"`，不用先装包也能直接跑。

```bash
python -m pytest tests      # 用 uv 时改 uv run pytest
ruff check .
```

`addopts = "-q"` 已经生效，自己再加 `-q` 会变成 `-qq` 并隐藏 `N passed` 汇总行。看不到汇总行时加 `-o addopts=""`。

当前 71 条用例全部通过。测试不依赖真实设备，不联网。

## 目录结构

```
alohamini-cartesian-lib/
├── pyproject.toml             # 包元数据、extras、dev 组、pytest 与 ruff 配置
├── README.md
├── .gitignore
├── docs/decisions.md          # D1–D4 决策记录
├── src/alohamini_cartesian/
│   ├── __init__.py            # __version__ = "0.1.0"
│   ├── py.typed
│   ├── adapter/protocol.py    # 对 lerobot 的唯一接触面（typing.Protocol）
│   ├── capability/detect.py   # 能力探测与降级、单位换算裁决
│   └── config/overlay.py      # YAML/JSON 加环境变量三级覆盖
└── tests/
    ├── test_capability.py
    └── test_config_overlay.py
```
