# 3 只读体检（原 §4，不含 §4.3）

本文件管什么：不动机械臂的只读体检——取 lerobot 版本、capture 指纹、校准字段与断电期间移位比对、左夹爪与全 18 电机寄存器，到库真机 dry-run。

## 4 只读确认（不动机器人）

本节全部是只读动作：4.1 只请求观测，4.1b 只读 JSON，4.2 只读伺服寄存器（Host 已停、串口空闲），4.3 只在 Pi 上读代码与配置。

<a id="sec-4-1b-lerobot"></a>
### 4.1b-lerobot-version 取 Pi 上 lerobot 版本（只读，先于 4.1 执行）

4.1 的指纹 JSON 要记 Pi 端 Host 用的 lerobot 版本；Mac 上没装 lerobot，所以先从 Pi 取，4.1 时把取到的值传进去。

```bash
run 4.1b-lerobot-version ssh "$PI" '~/miniconda3/envs/lerobot_alohamini/bin/python -c "import importlib.metadata; print(importlib.metadata.version(\"lerobot\"))"'
```

把取到的值收进 shell 变量，下一步 4.1 要引用：

```bash
export PI_LEROBOT_VERSION=$(tr -d '[:space:]' < "$S4DIR/stdout/4.1b-lerobot-version-$STAMP.log")
echo "PI_LEROBOT_VERSION=$PI_LEROBOT_VERSION"
```

判据：`.log` 里只有一行形如 `0.6.1` 的版本号，`$PI_LEROBOT_VERSION` 非空。若出现 `Traceback` / `PackageNotFoundError` / 空行，说明该解释器没装 lerobot 或路径变了 → 记「未取到」，**不要**手填版本号，4.1 就**不要**传 `--lerobot-version`（JSON 里会记 `null`，那是诚实的缺失记录）。

产物：`stdout/4.1b-lerobot-version-$STAMP.{cmd,log,exit}`。

<a id="sec-4-1"></a>
### 4.1 capture 指纹

```bash
run 4.1-capture uv run --no-sync python benchmarks/alohamini_io/capture_robot_fingerprint.py --host "$PI_HOST" --lerobot-version "$PI_LEROBOT_VERSION" --out "$S4DIR/capture/fingerprint-$STAMP.json"
```

判据：exit 0，`$S4DIR/capture/fingerprint-$STAMP.json` 写出，JSON 里 `robot_metadata.schema_version` 为 `2`。P0 就靠这一条判定。

第二条判据：JSON 里 `lerobot_version` 非 `null`，且与 `4.1b-lerobot-version` 取到的 `$PI_LEROBOT_VERSION` **逐字一致**。

`4.1` 的默认值：`--samples 5`、`--timeout-s 30.0`、`--port 5556`；`--lerobot-version` 不给时回落到本机已装的 `lerobot`，本机也没有就记 `null`；空串会被拒绝（exit 2）。

产物：`capture/fingerprint-$STAMP.json`、`stdout/4.1-capture-*`。

#### capture 在 Pi 侧跑（2026-11-05 实战做法）

本次实战里 capture 是**在 Pi 上跑的**（Mac 主环境没有 `lerobot`），因此 JSON 的 `host` 字段记的是 `127.0.0.1`。命令形态是「脚本经 stdin 投递 + Pi 侧 venv 解释器 + `PYTHONPATH=/home/pi5/lerobot_alohamini/src`」，产物落 Pi 的 `/tmp`，再用 `scp` 取回 `$S4DIR/capture/`。命令原文（路径按现场替换）：

```bash
ssh "$PI" 'cd /tmp && PYTHONPATH=/home/pi5/lerobot_alohamini/src ~/miniconda3/envs/lerobot_alohamini/bin/python - --host 127.0.0.1 --lerobot-version 0.6.1 --samples 5 --timeout-s 30.0 --out /tmp/fingerprint-$STAMP.json' < benchmarks/alohamini_io/capture_robot_fingerprint.py
```

判据补充：`host` 字段是 `127.0.0.1` 时，说明 capture 在被测机本机回环上跑，不是 Mac 到 Pi 的跨机路径——不要当成网络路径写进记录。

#### capture 的 JSON 有 11 个顶层键

| 键 | 含义 |
| --- | --- |
| `schema_version` | payload 格式版本，恒为 `1`（源码常量 `PAYLOAD_SCHEMA_VERSION = 1`） |
| `captured_utc` | 抓取时刻，ISO 8601，秒精度 |
| `host` | `--host` 传入的地址 |
| `requested_robot_model` | `--robot-model` 传入的模型名 |
| `read_only` | 恒为 `true` |
| `sample_count` | `samples` 的实际行数 |
| `samples` | 一行一个被接受的观测请求，字段 `observation_sequence`、`age_s`、`sent_monotonic_s`、`received_monotonic_s` |
| `robot_metadata` | 宿主 metadata 原样拷贝 |
| `control_status` | 最后一帧观测带的宿主控制状态帧，宿主没发时为 `null` |
| `observation_state` | 连接后取到的一帧 state-only 观测，只保留有限数值 |
| `lerobot_version` | Pi 端 Host 用的 lerobot 版本（`--lerobot-version` 原样传入）；取不到时为 `null`，不伪造 |

#### 这个 JSON 里有两个 `schema_version`

- 顶层 `schema_version` 是 `1`，它表示 payload 格式版本，永远不是 2。
- 要等于 2 的是 `robot_metadata.schema_version`。

现场最常见的误判是把顶层那个 `1` 当成失败原因。脚本只校验 `robot_metadata.schema_version == 2` 与 `motors` 非空映射，不校验每个电机的 8 个字段是否齐全。8 字段是 `id`、`model`、`normalization`、`drive_mode`、`range_min`、`range_max`、`homing_offset`、`resolution`，齐不齐由宿主决定，现场逐电机记录实际覆盖情况。

v2 宿主（HEAD `6aee1f81`）上 18 个电机的 `homing_offset` 与 `resolution` 全部有值，无 `null`。升级前的 v1 宿主给每个电机只写 6 个键，没有这两个字段。

<a id="sec-4-1b"></a>
### 4.1b 校准字段齐全性探测（只读）

先读 4.1 的 JSON，逐电机看校准字段是否齐全。这条不动机器人，只读盘上的文件：

```bash
run 4.1b-probe python - <<'PY'
import json, os
from pathlib import Path
s4 = Path(os.environ["S4DIR"])
paths = sorted(s4.glob("capture/fingerprint-*.json"))
assert paths, "先跑 4.1，拿到 fingerprint JSON"
meta = json.loads(paths[-1].read_text())["robot_metadata"]
motors = meta.get("motors") or {}
print(f"motors: {len(motors)}")
for name, fields in sorted(motors.items()):
    missing = [k for k in ("normalization", "drive_mode", "range_min", "range_max", "homing_offset", "resolution")
               if fields.get(k) is None]
    bad = []
    try:
        if fields.get("range_max") is not None and fields.get("range_min") is not None \
           and float(fields["range_max"]) <= float(fields["range_min"]):
            bad.append("range_max<=range_min")
        if fields.get("resolution") is not None and float(fields["resolution"]) <= 1.0:
            bad.append("resolution<=1")
    except (TypeError, ValueError):
        bad.append("non-numeric")
    print(f"{name}: missing={missing} bad={bad}")
PY
```

判断规则：

- 只有当某个电机的 `missing` 或 `bad` 非空时，它对应的关节才会在代码 Gate F 第二段被拒。
- v2 宿主上 18/18 电机字段齐全，Gate F 第二段不会被触发。此时只记录「v2 宿主下 Gate F 未触发」这一事实，理由写进 `$S4DIR/stdout/`。
- 记录证据时必须写明命中的是哪一段。两种拒绝要分开：`schema_version` 不是 2 → Gate F 第一段拒绝；换算字段不全 → Gate F 第二段拒绝。

产物：`stdout/4.1b-probe-*`。历史同类产物是 `stdout/5.1-probe21-20261025T094457Z.log`，输出为 18 个电机逐项 `missing=[] bad=[]`，即第二段未触发。

<a id="sec-4-1c"></a>
### 4.1c 断电期间移位检查：标定字段与位置分开比对（只读）

什么时候跑：**上一轮收工断电 → 本轮上电、Host 起来之后**，第一件只读动作就是这一节。它回答两个互不混同的问题：

1. 标定有没有被改过（`Min_Position_Limit` / `Max_Position_Limit` / `Homing_Offset` / `Drive_Mode` 一类）。任一不等 = **事故**，不是滑移。
2. 断电期间关节有没有滑移（位置变了、标定没变）。

前置：[§1](1-prepare.md#sec-1) P0–P5 全过；Host 常驻（`capture` 要连 5556）；本轮授权含只读与启停 Host。本节不动机器人。

#### 第一步：采一份新指纹

沿用 [§4.1](#sec-4-1) 的 `capture`，产物名换掉，避免覆盖基线：

```bash
run 4.1c-capture uv run --no-sync python benchmarks/alohamini_io/capture_robot_fingerprint.py --host "$PI_HOST" --out "$S4DIR/capture/fingerprint-postpower-$STAMP.json"
```

命名口径：过程指纹固定 `fingerprint-$STAMP.json`，收尾断电前固定 `fingerprint-postpower-$STAMP.json`，两者后缀不同、互不覆盖；同一 STAMP 下还要采 [§4.1c](#sec-4-1c) 的一份时，另存 `fingerprint-postpower-$STAMP.4.1c-preserved.json` 并在台账点名（2026-11-08 现场即此情形）。

判据：exit 0，`robot_metadata.schema_version` 为 `2`。**本节故意不传 `--lerobot-version`**：那个变量由 [§4.1b-lerobot-version](#sec-4-1b-lerobot) 导出，而本节常被当成「上电后第一件事」跑；传空串会被 `capture_robot_fingerprint.py:310-315` 以 exit 2 拒（原文 `--lerobot-version must not be empty`）。本节只判 `schema_version`，不判版本字段，所以整条标志省掉最稳；`lerobot_version` 记成 `null` 是诚实缺失，不是失败（口径同 [§4.1](#sec-4-1)）。
若主环境没有 `lerobot` 而以 **exit 3** 结束，改用 [§4.1](#sec-4-1)「capture 在 Pi 侧跑」的同一形态（Pi 侧解释器 + `PYTHONPATH=/home/pi5/lerobot_alohamini/src`），产物再用 `scp` 取回 `$S4DIR/capture/`。拿到 exit 2 说明命令本身写错了（多半是给了一个空变量），先修命令，别当成设备问题。

#### 第二步：一行能抄的比对命令

比对基准是**上一次收工断电前**采的现场指纹，现指向 `~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261106/capture/fingerprint-postpower-20261106T070942Z.json`（`captured_utc=2026-11-06T07:10:31+00:00`，`robot_metadata.schema_version=2`，18 个电机（左 11 + 右 7），`lerobot_version="0.6.1"`，`host=127.0.0.1`，`sample_count=5`；同一 STAMP 的产物另带 `stdout/4.1c-capture-20261106T070942Z.*` 与 `stdout/4.1c-capture-scp-20261106T070942Z.*`）。上一份 `fingerprint-postpower-20261106T064429Z.json`（`captured_utc=2026-11-06T06:45:05+00:00`）**已作废为位置基准**：2026-11-06 07:00Z 后用户换新电池并重新上电（`stdout/scope-append-20261106T070942Z.md` ①），位置跨过换电不再连续，它只剩标定字段可作历史对照；换电那一轮因此**判不了「断电期间移位」**，这份新基线服务的是下一轮断电。**基准必须采于上一次收工断电前，且此后没有未记录的运动**——否则位置组差值里混着那些运动，超阈也归不了因：2026-11-05 的旧指纹 `evidence/s4-20261104/capture/fingerprint-20261105T063600Z.json` 采于那轮试动之前、跨过了试动，位置组 8 只电机超 3 tick 却无法归因，已作废为**位置**基准（标定字段仍可作历史对照）。做法沿用 2026-11-05 已验证的 `robot_metadata.motors` 子树 diff 思路，但**把标定与位置拆成两段**，判据不同。

**该基线的位置组（B 组）只作归因，不作判据**：`fingerprint-postpower-20261106T070942Z.json`（`captured_utc=2026-11-06T07:10:31Z`）之后另有 07:19–09:06Z 六轮记录在案的运动，所以它的 B 组位置差值只能归到「记录在案的运动」（并据此识别未记录的运动），**不能当「断电期间滑移」的判据**；新基准在本次上电后重采。A 组（标定字段）不受影响，仍可判「标定有没有被改过」。

先把脚本落到 `$S4DIR/stdout/` 并在同一份指纹上自检一次（两侧同一文件，期望两组计数都是 0）：

```bash
cat > "$S4DIR/stdout/4.1c-postpower-diff.py" <<'PY'
#!/usr/bin/env python3
"""§4.1c 断电期间移位检查：标定字段与当前位置分开比对。只读两个指纹 JSON，不连设备。"""
import json
import sys
from pathlib import Path

CALIB_FIELDS = (
    "id", "model", "normalization", "range_min", "range_max",
    "homing_offset", "drive_mode", "resolution",
)
ABSENT = "<absent>"
SPEED_KEYS = ("x.vel", "y.vel", "theta.vel")


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def to_tick(norm, motor):
    """drive_mode=0 情形下与厂商 _unnormalize 同式的换算（motors_bus.py:892-902）。

    厂商实现还有 drive_mode 取反与 ±100 / 0..100 钳制两步，本脚本没有；本机 18 个电机的
    drive_mode 全是 0（指纹可核），所以数值等价。若某轴 drive_mode 变成 1，B 组的 Δ 符号会反
    ——那种变更由 A 组（drive_mode 字段）先拦下来。
    """
    lo, hi = motor["range_min"], motor["range_max"]
    if motor["normalization"] == "range_0_100":
        return int((norm / 100.0) * (hi - lo) + lo)
    return int(((norm + 100.0) / 200.0) * (hi - lo) + lo)


def main(argv):
    if len(argv) != 4:
        print("usage: 4.1c-postpower-diff.py <baseline.json> <today.json> <out.txt>", file=sys.stderr)
        return 2
    base, today, out = load(argv[1]), load(argv[2]), Path(argv[3])
    bm = base["robot_metadata"]["motors"]
    tm = today["robot_metadata"]["motors"]
    bs = base.get("observation_state") or {}
    ts = today.get("observation_state") or {}

    lines = [
        f"# baseline: {argv[1]}  captured_utc={base.get('captured_utc')}",
        f"# today   : {argv[2]}  captured_utc={today.get('captured_utc')}",
        "# rule: calibration fields compared verbatim; positions compared in tick, threshold 3 tick",
    ]

    lines.append("")
    lines.append("== A. 标定字段（任一不等 = 事故，停） ==")
    calib_diffs = 0
    for name in sorted(set(bm) | set(tm)):
        old, new = bm.get(name) or {}, tm.get(name) or {}
        rows = [
            f"{field}: {old.get(field, ABSENT)} -> {new.get(field, ABSENT)}"
            for field in CALIB_FIELDS
            if old.get(field, ABSENT) != new.get(field, ABSENT)
        ]
        if rows:
            calib_diffs += len(rows)
            lines.append(f"{name}:")
            lines.extend(f"  {row}" for row in rows)
    lines.append(f"CALIB_FIELD_DIFFS: {calib_diffs}")

    lines.append("")
    lines.append("== B. 当前位置（tick；阈值 3 tick） ==")
    lines.append(f"{'motor':<26} | {'baseline_tick':>13} | {'today_tick':>10} | {'delta':>6} | verdict")
    pos_over = 0
    for name in sorted(bm):
        key = f"{name}.pos"
        if key not in bs or key not in ts:
            continue
        try:
            old_tick = to_tick(float(bs[key]), bm[name])
            new_tick = to_tick(float(ts[key]), tm.get(name) or bm[name])
        except (KeyError, TypeError, ValueError) as exc:
            lines.append(f"{name:<26} | 换算失败 {type(exc).__name__}: {exc}")
            continue
        delta = new_tick - old_tick
        if abs(delta) > 3:
            pos_over += 1
        lines.append(
            f"{name:<26} | {old_tick:>13} | {new_tick:>10} | {delta:>+6} | "
            f"{'>3 tick，记台账' if abs(delta) > 3 else 'ok'}"
        )
    lines.append(f"POSITION_MOTORS_OVER_3_TICK: {pos_over}")

    lines.append("")
    lines.append("== C. 不在 .pos 口径里的量（原样比，不折 tick） ==")
    for key in ("lift_axis.height_mm", *SPEED_KEYS):
        if key in bs or key in ts:
            lines.append(f"{key}: {bs.get(key, ABSENT)} -> {ts.get(key, ABSENT)}")

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {out}")
    print(f"CALIB_FIELD_DIFFS: {calib_diffs}")
    print(f"POSITION_MOTORS_OVER_3_TICK: {pos_over}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
PY

export BASEFP=~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261106/capture/fingerprint-postpower-20261106T070942Z.json
run 4.1c-selfcheck uv run --no-sync python "$S4DIR/stdout/4.1c-postpower-diff.py" "$BASEFP" "$BASEFP" "$S4DIR/stdout/4.1c-selfcheck-$STAMP.txt"
```

判据（自检）：exit 0，输出 `CALIB_FIELD_DIFFS: 0` 与 `POSITION_MOTORS_OVER_3_TICK: 0`。自检不过说明脚本本身有问题，先修脚本再比真数据（本脚本的两组计数在本地各做过一次自检与一次变异自检：改 `arm_left_gripper.homing_offset` +1 只进 A 组记 1 条；把 `arm_left_shoulder_pan.pos` 加 0.5 只进 B 组并算成 `+6 tick`）。

比对真数据：

```bash
run 4.1c-postpower-diff uv run --no-sync python "$S4DIR/stdout/4.1c-postpower-diff.py" "$BASEFP" "$S4DIR/capture/fingerprint-postpower-$STAMP.json" "$S4DIR/stdout/4.1c-postpower-diff-$STAMP.txt"
```

#### 比对键与判据

| 组 | 比对什么 | 来源字段 | 判据 | 落空时怎么记 |
| --- | --- | --- | --- | --- |
| A1 标定 | `range_min`、`range_max`、`homing_offset`、`drive_mode`（= 寄存器 `Min_Position_Limit` / `Max_Position_Limit` / `Homing_Offset` / `Drive_Mode`） | `robot_metadata.motors.<motor>.*` | **任一不等 → 停**，标定变动属事故，禁止继续试动、禁止改标定文件去「对齐」 | 记录电机名与字段、两侧原值 |
| A2 拓扑 | `id`、`model`、`normalization`、`resolution` | `robot_metadata.motors.<motor>.*` | 任一不等 → 记录并核对 vendor 升级记录（换型号 / 换分辨率是升级的正常结果，但必须能解释） | 记录电机名与字段、两侧原值，并附 vendor 升级记录原文 |
| B 位置 | 14 个 `arm_*_<joint>.pos` 的 tick 值（用厂商 `_unnormalize` 的同一条公式换算：`range_m100_100` 用 `int(((v+100)/200)*(max-min)+min)`，`range_0_100` 用 `int((v/100)*(max-min)+min)`，`motors_bus.py:892-902`） | `observation_state` | 每电机 \|Δtick\| ≤ **3** → 记「断电无滑移」；> 3 → 把电机名、基准 tick、本次 tick、差值写进台账。3 tick 是本项目自定的「非滑移」上界，不是厂商值：两侧各自 `int()` 截断各带 ≤1 tick 误差，余 1 tick 作为机械回差余量 | 只列超出 3 tick 的电机 |
| C 其它 | `lift_axis.height_mm`（mm，不折 tick）、底盘三轴 `x.vel` / `y.vel` / `theta.vel`（原样比） | `observation_state` | 原样记录；本轮不设阈值 | 记两侧原值 |

三条口径要点，写记录时必须带上：

- **Punch 不在指纹里**。Feetech 的 Punch（起步力，寄存器名 `Minimum_Startup_Force`）只由 [§4.2](#sec-4-2) 的 `EXTRA_REGS` 读到，`robot_metadata.motors` 没有这个字段。要比 Punch，先跑 [§4.2](#sec-4-2) 并把两轮读数并列，不能拿指纹冒充。
- **B 组只覆盖 14 个臂电机**。指纹 `observation_state` 的观测键里 `lift_axis` 是 `lift_axis.height_mm`（进 C 组），底盘三轴只有速度键、没有位置键（本机指纹里 `base_left_wheel` / `base_back_wheel` / `base_right_wheel` 在 `observation_state` 里没有键）。所以「18 电机位置比对」这个说法要写清是「14 个臂电机按 tick 比 + 升降轴按 mm 比 + 底盘三轴只有速度」。
- **左夹爪的 tick 分辨率极粗**：`range_0_100` 下 `range_min..range_max` 是 `2021..2072`（51 tick），归一化值每变 1 单位约 0.51 tick，小位移会在 tick 上被吞掉。夹爪读数只能当「有没有大位移」看，不能当精细滑移证据。

产物：`capture/fingerprint-postpower-$STAMP.json`、`stdout/4.1c-postpower-diff.py`、`stdout/4.1c-selfcheck-$STAMP.txt`、`stdout/4.1c-postpower-diff-$STAMP.txt`，以及每步的 `.cmd` / `.log` / `.exit` 三件套。


<a id="sec-4-2"></a>
### 4.2 左夹爪 ID7 与全 18 电机只读确认

前置条件：[§1](1-prepare.md#sec-1) P0–P5 全过；按本节「现场确认项」表的 `3.7b2-serial-links` 确认两个串口别名在（`/dev/am_arm_follower_left` → `ttyACM0`、`/dev/am_arm_follower_right` → `ttyACM1`）；Host 已停（串口被 Host 占着时读不到）；授权记录里写明了「向 Pi 写文件」与「停 Host」。

为什么别名要单独核：[§3.7](1-prepare.md#sec-3) 的 `ls -l /dev/serial/by-id/` 只列出 by-id 名（2026-11-06 实测 `usb-1a86_USB_Single_Serial_5B14029263-if00 → ../../ttyACM0`、`…5B14111703-if00 → ../../ttyACM1`，`stdout/3.7b-serial-pi-20261106T062506Z.log`），从这份列表核不出 `am_arm_follower_*` 这两个别名是否存在、指向哪个口——两套名字不能互相代替。

背景一句：离线核对（`docs/field-results.md`「离线核对（无新采集）· 左右夹爪行程差异」）发现 `arm_left_gripper` 的 `range` 是 2021-2072，只有 51 tick（`range_0_100` 下 4.48°），而 `arm_right_gripper` 是 2010-3479，1469 tick（129.14°）。两侧同为 `sts3250`、`drive_mode` 同为 0，根因未确认。`docs/ADR.md` D14 已定本轮不重标定，本步只做只读确认，判成因落在哪一侧。

做法：以证据目录里的 `stdout/reg_read_1792923639.py` 为模板，在 `MOTORS` 里给左臂总线加上夹爪，其余不动：

```python
PORT = "/dev/am_arm_follower_left"
MOTORS = {
    "shoulder_pan": Motor(1, "sts3250", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3095", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3095", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(6, "sts3250", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(7, "sts3250", MotorNormMode.RANGE_0_100),  # 新增
}
```

要看的寄存器：`Min_Position_Limit`、`Max_Position_Limit`、`Present_Position`、`Goal_Position`、`Homing_Offset`、`Torque_Enable`、`Operating_Mode`、`Model_Number`、`Status`。

2026-11-05 的实战做法：把探测目标从 `shoulder_pan`（ID1）扩到 `gripper`（ID7），`MOTORS` 里加 `"gripper": Motor(7, "sts3250", MotorNormMode.RANGE_0_100)`，并把存活探测（`probe` 与波特率重试）换成对 ID7 读 `Model_Number`；左右两臂各跑一次（两个脚本，右臂脚本的 `PORT` 换成右臂总线别名）。

执行方式有三件事，其中两件已定值，一件仍需现场确认：

1. 投递方式：有「向 Pi 写文件」授权就走下面的 scp 支；没有授权就走 stdin 支（不向 Pi 落盘）。
2. 解释器：`/home/pi5/miniconda3/envs/lerobot_alohamini/bin/python`，即 Host 自己用的解释器（[§6.1](2-host.md#host-start) 启动命令原文，上次现场已验证存在）。
3. 端口别名：脚本里 `PORT = "/dev/am_arm_follower_left"` 是别名，原脚本注释写它指向 `ttyACM0`。[§3.7](1-prepare.md#sec-3) 的输出若显示别名与实际节点不同，按实际改。

把改好的脚本放到 Pi 的 `/tmp/` 属于向 Pi 写文件，需在 [§3.9](1-prepare.md#sec-3) 的授权记录里写明。

本脚本 2026-11-05 已在真机执行两次：左臂、右臂各一次，两次都是 `EXIT_CODE: 0`（`stdout/4.2-gripper-regs-*`、`stdout/4.2-gripper-regs-right-*`）。实测读数：左臂 ID7 `Min_Position_Limit = 2021` / `Max_Position_Limit = 2072`，命中判据第一支（固件限位生效，成因在标定值，D14 维持不重标定）；右臂 ID7 `2010` / `3479`；两只都是 `Torque_Enable=0`、`Lock=1`。可比性限定：本轮 `Torque_Enable=0` 与 2026-10-25 读到的 ID1/2/3/6 `Torque_Enable=1` 不可比——测量对象不同（本轮读的是夹爪 ID7）。

#### 第一步：在 Mac 上建脚本

只在本地写文件，不涉及 Pi：

```bash
cat > "$S4DIR/stdout/4.2-gripper-regs-$STAMP.py" <<'PY'
# -*- coding: utf-8 -*-
"""READ-ONLY servo register dump for alohamini left arm (ttyACM0).

This script only reads registers. It issues no servo commands other than the
read instruction. Torque state is left exactly as found.
"""
from __future__ import annotations

import json
import sys
import time

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.feetech.tables import MODEL_NUMBER_TABLE
from lerobot.motors.motors_bus import Motor, MotorNormMode

PORT = "/dev/am_arm_follower_left"   # -> ttyACM0 (left arm). ttyACM1 (right) is never opened.
BAUDRATE = 1_000_000
WAIT_S = 0.5

MOTORS = {
    "shoulder_pan": Motor(1, "sts3250", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3095", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3095", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(6, "sts3250", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(7, "sts3250", MotorNormMode.RANGE_0_100),  # 新增：左臂夹爪 ID7
}

REQUIRED_REGS = [
    "Model_Number", "Firmware_Major_Version", "Firmware_Minor_Version",
    "Present_Load", "Torque_Limit", "Max_Torque_Limit",
    "P_Coefficient", "I_Coefficient", "D_Coefficient",
    "Goal_Position", "Present_Position", "Torque_Enable",
    "Operating_Mode", "Protection_Current",
]

EXTRA_REGS = [
    "Present_Velocity", "Present_Current", "Present_Temperature", "Present_Voltage",
    "Status", "Moving", "Minimum_Startup_Force", "CW_Dead_Zone", "CCW_Dead_Zone",
    "Overload_Torque", "Protective_Torque", "Protection_Time", "Unloading_Condition",
    "Max_Temperature_Limit", "Homing_Offset", "Min_Position_Limit", "Max_Position_Limit",
    "Goal_Velocity", "Lock", "Velocity_closed_loop_P_proportional_coefficient",
]

DYNAMIC_REGS = ["Present_Position", "Present_Load", "Present_Velocity", "Moving"]

ALL_REGS = REQUIRED_REGS + EXTRA_REGS

results = {}   # name -> {reg: {"ok": bool, "value": int|None, "err_type":..., "err":...}}
pass2 = {}


def read_one(bus, name, reg):
    try:
        v = bus.read(reg, name, normalize=False, num_retry=0)
        return {"ok": True, "value": v, "err_type": None, "err": None}
    except Exception as exc:  # noqa: BLE001 - one bad register must not abort the sweep
        return {"ok": False, "value": None, "err_type": type(exc).__name__, "err": str(exc)}


def main():
    print("READ-ONLY register sweep -- target:", PORT, "baud:", BAUDRATE, flush=True)
    bus = FeetechMotorsBus(port=PORT, motors=MOTORS, calibration=None)
    bus.connect(handshake=False)          # openPort() only; no servo packet emitted
    print("connected:", bus.is_connected, flush=True)
    bus.set_baudrate(BAUDRATE)            # local UART setting only, no servo packet

    # quick liveness probe on ID1 only
    probe = read_one(bus, "shoulder_pan", "Model_Number")
    print("probe shoulder_pan Model_Number ->", probe, flush=True)
    if not probe["ok"]:
        for br in (1_000_000, 500_000, 115_200):
            bus.set_baudrate(br)
            p2 = read_one(bus, "shoulder_pan", "Model_Number")
            print(f"retry baud={br} ->", p2, flush=True)
            if p2["ok"]:
                break

    for name in MOTORS:
        results[name] = {}
        m = MOTORS[name]
        print(f"--- ID{m.id} {name} (configured model={m.model}) ---", flush=True)
        for reg in ALL_REGS:
            r = read_one(bus, name, reg)
            results[name][reg] = r
            if r["ok"]:
                extra = ""
                if reg == "Model_Number":
                    extra = "  (MODEL_NUMBER_TABLE sts3215=777 sts3250=2825 sts3095=2569)"
                print(f"  {reg:52s} = {r['value']}{extra}", flush=True)
            else:
                print(f"  {reg:52s} = FAIL  {r['err_type']}: {r['err']}", flush=True)

    print(f"--- wait {WAIT_S}s, sample #2 ---", flush=True)
    time.sleep(WAIT_S)
    for name in MOTORS:
        pass2[name] = {}
        print(f"--- ID{MOTORS[name].id} {name} ---", flush=True)
        for reg in DYNAMIC_REGS:
            r = read_one(bus, name, reg)
            pass2[name][reg] = r
            print(f"  {reg:52s} = {r['value'] if r['ok'] else 'FAIL  ' + str(r['err_type']) + ': ' + str(r['err'])}", flush=True)

    # disconnect without touching torque: the keyword below DISABLES the torque-off path
    bus.disconnect(disable_torque=False)
    print("after disconnect: bus.is_connected =", bus.is_connected, flush=True)

    print("=== JSON ===", flush=True)
    print(json.dumps({
        "port": PORT,
        "baudrate": BAUDRATE,
        "wait_s": WAIT_S,
        "motors": {n: {"id": m.id, "configured_model": m.model} for n, m in MOTORS.items()},
        "required_regs": REQUIRED_REGS,
        "extra_regs": EXTRA_REGS,
        "pass1": results,
        "pass2": pass2,
        "model_number_table": MODEL_NUMBER_TABLE,
        "is_connected_after_disconnect": bus.is_connected,
    }, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    except BaseException as exc:  # noqa: BLE001
        print("FATAL:", type(exc).__name__, exc, flush=True)
        rc = 2
    print("EXIT_CODE:", rc, flush=True)
    sys.exit(rc)

PY
```

#### 第二步：投递到 Pi 的 `/tmp`（**需「向 Pi 写文件」授权**）

```bash
run 4.2-scp scp "$S4DIR/stdout/4.2-gripper-regs-$STAMP.py" "$PI:/tmp/4.2-gripper-regs-$STAMP.py"
```

#### 第三步：在 Pi 上跑（与 Host 同环境解释器）

```bash
run 4.2-gripper-regs ssh "$PI" "~/miniconda3/envs/lerobot_alohamini/bin/python /tmp/4.2-gripper-regs-$STAMP.py"
```

#### 没有「向 Pi 写文件」授权时的替代（方案需现场确认）

```bash
run 4.2-gripper-regs ssh "$PI" '~/miniconda3/envs/lerobot_alohamini/bin/python -' < "$S4DIR/stdout/4.2-gripper-regs-$STAMP.py"
```

脚本经 stdin 送入，不向 Pi 落盘，因此不需要写盘授权；若现场不允许 stdin 送脚本，改为在 Pi 上手工输入同一脚本。

本步只读、不写扭矩寄存器。

判据（二选一，记录里必须写明走了哪一支）：

| 读数 | 结论 | 现场怎么处理 |
| --- | --- | --- |
| `Min_Position_Limit` / `Max_Position_Limit` 等于 `2021` / `2072` | 固件限位已生效，左夹爪被锁在 51 tick 内，成因在标定值 | 记为「已确认，不处理」。D14 已定本轮不重标定 |
| 读出别的值 | 问题在写盘或写盘之后的覆盖 | 记入待办清单，本窗口不再动标定文件 |

同一轮再读一次 `Torque_Enable` 与 `Operating_Mode`，只记录，不改。

参照读数（2026-10-25 停 Host 后读左臂 ID 1/2/3/6）：`Torque_Enable=1`、`Lock=0`、`Operating_Mode=0`、`Status=0`、`Torque_Limit` 与 `Max_Torque_Limit` 都是 1000、`P`/`I`/`D` 都是 16/0/32，原文在 `stdout/reg_read_1792923639.log`。这组状态**写入者未识别**，所以本步只读、不写扭矩寄存器。

产物：`stdout/4.2-gripper-regs-$STAMP.{cmd,log,exit}` 与脚本副本 `stdout/4.2-gripper-regs-$STAMP.py`。

#### 全 18 电机静态体检（左臂 + 右臂 + 底盘 3 轮 + 升降轴）

上一小节只读左夹爪 ID7。这一小节把同一套只读脚本扩到全部 18 个电机，一轮拿齐静态读数（`Present_Voltage` / `Present_Temperature` / `Status` / 限位 / 偏置），供后续所有「先查电压温度再谈运动」的判据使用。

**寄存器口径：34 项 = `REQUIRED_REGS` 14 项 + `EXTRA_REGS` 20 项**（本节脚本的清单，2026-11-05 现场核实口径）。文档里其它口径的项数一律以这一条为准。

**总线与 ID 分配**（出处是 vendor 源码 + `configs/site.yaml` 的已实测设备路径，**不是现场逐台复核**；见后面的现场确认项）：

| 总线（`PORT`） | 电机名 | ID | model | 归一化 | 出处 |
| --- | --- | --- | --- | --- | --- |
| 左 `/dev/am_arm_follower_left`（→ `ttyACM0`） | `arm_left_shoulder_pan` … `arm_left_gripper` | 1–7 | 1 `sts3250`、2/3 `sts3095`、4/5/6 `sts3250`、7 `sts3250` | 1–6 `range_m100_100`、7 `range_0_100` | vendor `alohamini.py:155-163`（`am-follower-6dof-hd` 档）+ `configs/site.yaml:14` |
| 左（**同一条总线**） | `base_left_wheel` / `base_back_wheel` / `base_right_wheel` | 8 / 9 / 10 | `sts3250` | `range_m100_100` | vendor `alohamini.py:209-215`（`left_bus_motors` 里同时有 `arm_left_*` 与 `base_*`），vendor 那份型号表 `model_specs.py:75` 的 `base_motor`（`ec65318f` 的 `alohamini2pro` 段，值 `sts3250`） |
| 左（**同一条总线**） | `lift_axis` | 11 | `sts3095` | `degrees` | 同上，`lift_motor` |
| 右 `/dev/am_arm_follower_right`（→ `ttyACM1`） | `arm_right_*` | 1–7 | 同左臂 | 同左臂 | `configs/site.yaml:16` + 同一份 `am-follower-6dof-hd` 档 |

一句话人话：底盘三个轮子和升降轴**不在独立总线上**，它们挂在左臂那条总线（`left_port`）上——vendor 把 `arm_left_*`、`base_*`、`lift_axis` 放进同一个 `left_bus_motors` 字典，再传给 `FeetechMotorsBus(port=self.config.left_port, ...)`（`alohamini.py:209-222`）。左总线 11 个电机、右总线 7 个，合起来 18 个。

脚本改动只有 `MOTORS` 一段（其余——`REQUIRED_REGS` / `EXTRA_REGS` / 存活探测 / 第二遍 `pass2` / `disconnect(disable_torque=False)`——一个字不改）。两段按总线各跑一次：

左总线（11 个电机）：

```python
PORT = "/dev/am_arm_follower_left"
MOTORS = {
    "arm_left_shoulder_pan": Motor(1, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_left_shoulder_lift": Motor(2, "sts3095", MotorNormMode.RANGE_M100_100),
    "arm_left_elbow_flex": Motor(3, "sts3095", MotorNormMode.RANGE_M100_100),
    "arm_left_wrist_flex": Motor(4, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_left_wrist_yaw": Motor(5, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_left_wrist_roll": Motor(6, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_left_gripper": Motor(7, "sts3250", MotorNormMode.RANGE_0_100),
    "base_left_wheel": Motor(8, "sts3250", MotorNormMode.RANGE_M100_100),
    "base_back_wheel": Motor(9, "sts3250", MotorNormMode.RANGE_M100_100),
    "base_right_wheel": Motor(10, "sts3250", MotorNormMode.RANGE_M100_100),
    "lift_axis": Motor(11, "sts3095", MotorNormMode.DEGREES),
}
```

右总线（7 个电机）：

```python
PORT = "/dev/am_arm_follower_right"
MOTORS = {
    "arm_right_shoulder_pan": Motor(1, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_right_shoulder_lift": Motor(2, "sts3095", MotorNormMode.RANGE_M100_100),
    "arm_right_elbow_flex": Motor(3, "sts3095", MotorNormMode.RANGE_M100_100),
    "arm_right_wrist_flex": Motor(4, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_right_wrist_yaw": Motor(5, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_right_wrist_roll": Motor(6, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_right_gripper": Motor(7, "sts3250", MotorNormMode.RANGE_0_100),
}
```

同时把存活探测的**两处**（probe 与波特率重试循环里那两次 `read_one`）都从 `"shoulder_pan"` 改成 `"arm_left_shoulder_pan"`（右总线那份对应改成 `"arm_right_shoulder_pan"`）：电机名改了，探测键必须跟着改，否则第一行就 `KeyError`。

跑法照 [§4.2](#sec-4-2) 第一到第三步（Mac 上建脚本 → `scp` 或 stdin 投递 → 用 Host 同环境解释器跑），产物名分开：

投递沿用 [§4.2](#sec-4-2) 第二步的两条路径之一（有「向 Pi 写文件」授权就 `scp`，没有就走 stdin）：

```bash
run 4.2-all18-scp scp "$S4DIR/stdout/4.2-all18-left-$STAMP.py" "$PI:/tmp/4.2-all18-left-$STAMP.py"
run 4.2-all18b-scp scp "$S4DIR/stdout/4.2-all18-right-$STAMP.py" "$PI:/tmp/4.2-all18-right-$STAMP.py"
```

```bash
run 4.2-all18-left ssh "$PI" "~/miniconda3/envs/lerobot_alohamini/bin/python /tmp/4.2-all18-left-$STAMP.py"
run 4.2-all18-right ssh "$PI" "~/miniconda3/envs/lerobot_alohamini/bin/python /tmp/4.2-all18-right-$STAMP.py"
```

判据：

| 判据 | 值 |
| --- | --- |
| 两份日志都以 `EXIT_CODE: 0` 结束 | 必需 |
| 每条总线每个电机都读到 `Model_Number` | 左 11/11、右 7/7；对照 `MODEL_NUMBER_TABLE`（`sts3215=777`、`sts3250=2825`、`sts3095=2569`） |
| 18 个电机的 `Present_Voltage` 全部回读 | 记 min / mean / max 三个数 |
| `Status` 全 0 | 非 0 就把该电机与非 0 值单独记下 |

**满电电压基准的判据**：记录 18 个电机 `Present_Voltage` 的 min / mean / max；**任一 < 10.6 V → 记「需充电后再取基准」**，本窗口不开始任何运动类步骤。10.6 V 是 **2026-10-25 台账的历史观测**（`docs/field-results.md`「扭矩处于开启态」一行记到温度 22–27 ℃、电压 10.6–10.8 V），**不是厂商规格，也不是本机型下限**；充没充电、仓内有没有充电步骤可写，仓库里都没有，所以这一条只给判据不给流程。

现场确认项（到场先填，填之前不要改脚本硬编码）：

| 项 | 怎么确认 | 填什么 |
| --- | --- | --- |
| 左 / 右总线别名的实际指向（复核） | `run 3.7b2-serial-links ssh "$PI" 'ls -l /dev/am_arm_follower_left /dev/am_arm_follower_right'` | 判据：两个别名都存在且分别指向 `ttyACM0` / `ttyACM1`，与 `configs/site.yaml` 的 `left_port` / `right_port` 一致。2026-11-06 实测 `left → ttyACM0`、`right → ttyACM1`（`stdout/3.7b2-serial-links-20261106T062506Z.log`）；换机、重插、换 USB 口后需复核 |
| 左总线上 ID8–11 是否在线 | 本节左总线那份日志里 `base_*` / `lift_axis` 的 `Model_Number` 是否回读到 | 读到 = 与源码一致；读不到 = 记现象原文，暂按「这台的底盘 / 升降接在别处」追查，不改脚本去猜端口 |
| vendor Host 启动输出里的端口 | Host 启动日志原文（[§6.1](2-host.md#host-start) 的启动命令骨架） | 若出现别的串口名，把原文抄进记录 |
| 底盘三轴与升降是否带电 | 本节读到的 `Torque_Enable` | 三个轮子与 `lift_axis` 各自的 `Torque_Enable` 值 |

产物：`stdout/4.2-all18-left-$STAMP.{cmd,log,exit}`、`stdout/4.2-all18-right-$STAMP.{cmd,log,exit}` 与两份脚本副本 `stdout/4.2-all18-*-$STAMP.py`。

<a id="sec-4-4"></a>
### 4.4 库真机 dry-run（只读观测，零下发）

前置：Host 常驻（[§3.4](1-prepare.md#sec-3) 的 5555 / 5556 在听）；本轮授权含「只读观测」；vendor 快照已在 `/tmp`（取法见 [§5.3](../reference/env-rebuild.md#sec-5-3) 的两行 `git clone` + `rev-parse HEAD`，应等于 Pi 运行时的 HEAD `6aee1f81`）。

这条命令验的是本库自己的 `connect(dry_run=True)` 真 client 路径：用厂商 `AlohaMiniClient` 注入本库，读观测、对升降轴下一条命令、断开——**`send_action` 一次都不该发生**。

```bash
run 4.4-robot-dry-run env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src:/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python examples/robot_dry_run.py --host "$PI_HOST"
```

`PYTHONPATH` 两段都必需：第一段是 vendor 快照（主环境与隔离环境都没有 `latest_request_timing` 那版 vendor 代码），第二段是本库 `src/`（本库没装进 vendor 环境，见 `examples/robot_dry_run.py:17-20` 的自述）。在仓库根下跑，`src` 写绝对路径以免依赖 cwd。

在 Pi 本机上跑时省掉 `--host`（脚本默认 `127.0.0.1`）；本手册的 `run` 在 Mac 上跑，所以要显式给 `"$PI_HOST"`。

判据（三件，缺一不算通过）：

| 判据 | 看什么 |
| --- | --- |
| exit 0 | `stdout/4.4-robot-dry-run-$STAMP.exit` 为 `0` |
| 零下发 | stdout 原文 `send_action 调用次数: 0 次发送（dry_run=True，期望 0）`（`examples/robot_dry_run.py:431`） |
| 读到了真观测 | stdout 有 `host=... ：读一帧观测，键 N 个` 与 metadata 行（`schema_version=2`、motor 条目 18） |

**这份脚本没有报告 JSON**：它只往 stdout 打印，`dry_run` 不落成字段，也没有观测帧数计数。「`dry_run` 为真」的现场可取证形态就是上面那句原文 + exit 0，不要把「没有 JSON」记成失败。

相机一句：脚本自身两次取帧都走 `include_cameras=False`（先 `prime_observation_request_window(include_cameras=False)`，再 `get_observation(include_cameras=False)`），但库内 `LiftHandle.height()`（`src/alohamini_cartesian/control/api.py:494`）、`move_by` 内部的 `height()`、以及 `_prime_state_machine()` 的首帧（`control/api.py:845`）共 3 次调用**无参** `get_observation()`，落回厂商默认 `include_cameras=True`——真机分支仍会多发 3 次相机帧请求。这是已知形态（`examples/robot_dry_run.py:51-59` 写明），不是本次新增的缺陷；证据在 Host 侧收到的请求，脚本输出里没有帧数。

预期结果：升降轴那条命令在 priming 阶段的 adapter 门禁被拒（`MotionNotEnabled`），打印拒绝原因后继续走完。**命令被拒属预期，不是失败**（`examples/robot_dry_run.py:44-49`）；失败指的是 exit 非 0，或 `send_action` 计数不为 0。

另一种预期形态：脚本先调 `session.lift.height()`，若反馈失联会先抛 `FeedbackLost`（`examples/robot_dry_run.py:344-346` 自己写明这一点），**这不等于 adapter 门禁拒绝**，按 `FeedbackLost` 记（查 Host 是否在跑、观测是否在涨），不要记成「门禁按预期拒绝」。

产物：`stdout/4.4-robot-dry-run-$STAMP.{cmd,log,exit}`。

