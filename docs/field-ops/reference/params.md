# 两个脚本的完整参数表（原 附录 A）

本文件管什么：`capture_robot_fingerprint.py` 与 `measure_latency.py` 每个参数是否必填、默认值是多少、什么值会被参数解析拒掉——判「这个值合不合法」以这里为单一来源（`measure` 的 `--help` 不打印默认值，默认值来自源码常量）。

<a id="param-capture"></a>
## 附录 A：两个脚本的完整参数表（原 附录 A）

### capture（6 个参数加 `-h`）

| 参数 | 必填 | 默认值 | 约束 | 影响什么 |
| --- | --- | --- | --- | --- |
| `--host` | 是 | 无 | 无 | 连哪台 Pi |
| `--port` | 否 | `5556` | 无 | 宿主观测端口 |
| `--robot-model` | 否 | `alohamini2pro` | 无 | 传给客户端的模型名；代码 Gate C 拿它与宿主 metadata 的 `robot_model` 比对，取值按第 4.1 节 JSON 里的值填 |
| `--samples` | 否 | `5` | 不小于 1（否则参数解析失败 exit 2） | 要采到的观测请求数 |
| `--timeout-s` | 否 | `30.0` | 必须有限且大于 0（否则参数解析失败 exit 2）；预算合法但到期仍不够样本落 exit 4 | 采样本的总时间预算，也是 `capture` 唯一的停机条件 |
| `--out` | 否 | `artifacts/alohamini-io/fingerprint-<UTC>.json`，解析参数时算时间戳 | 父目录自动创建 | JSON 落盘位置 |

<a id="param-measure"></a>
### measure（18 个参数加 `-h`）

| 参数 | 必填 | 默认值 | 约束 | 影响什么 |
| --- | --- | --- | --- | --- |
| `--host` | 是 | 无 | 无 | 连哪台 Pi |
| `--port` | 否 | `5556` | 无 | 宿主观测端口 |
| `--robot-model` | 否 | `alohamini2pro` | 无 | 代码 Gate C 的比对对象 |
| `--mode` | 否 | `rtt` | `rtt` 或 `tracking` | 跑哪套测量；`tracking` 会真动机器人 |
| `--rate-hz` | 否 | `30.0` | 必须有限且大于 0（否则参数解析失败 exit 2）；小于 200 打警告 | 观测采样率，与 `--control-hz` 独立 |
| `--control-hz` | 否 | `50.0` | 必须有限且不小于 1（否则 exit 2） | 命令最小间隔的倒数 |
| `--samples` | 否 | `500` | 不小于 1（否则 exit 2），仅 rtt 用 | 要采到的观测帧数 |
| `--timeout-s` | 否 | `30.0` | 必须有限且大于 0（否则参数解析失败 exit 2）；预算合法但在运行中到期不是 `2`：rtt 落 exit 7（样本不足）、tracking 落 exit 1（提前停止） | 整次运行的时间预算 |
| `--csv` | 否 | `artifacts/alohamini-io/<mode>-<UTC>.csv` | 无 | CSV 落盘位置 |
| `--request-window` | 否 | `1` | 不小于 1（否则 exit 2） | 在飞观测请求数；1 是 RTT 形态，3 是部署形态 |
| `--host-loop-hz` | 否 | `50.0` | 必须有限且大于 0（否则 exit 2） | 宿主主循环频率，决定相位项与排队项 |
| `--enable-motion` | 否 | 开关，默认关 | 无 | 代码 Gate A 的第一个条件 |
| `--joint` | 否 | 无 | tracking 模式实质必填 | 要步进的关节；代码 Gate A 的第二个条件 |
| `--delta-deg` | 否 | `1.0` | 必须有限且**非零**（`0` 落 exit 2）；幅度被钳到 `±1.0`、符号保留（`measure_latency.py:504`、`:2118-2126`） | 每步目标增量（度）；负值即朝反方向 |
| `--steps` | 否 | `20` | 不小于 1（否则 exit 2） | 步数；`\|delta\| × steps` 受 20.0° 上限约束（`measure_latency.py:2414-2418`） |
| `--settle-timeout-s` | 否 | `1.0` | 必须有限且大于 0（否则 exit 2） | 每步的稳定判定预算 |
| `--settle-tolerance-deg` | 否 | `0.15` | 必须有限且不小于 0（`0` 合法）；且必须小于 clamp 后有效 delta 的**绝对值**；两者违规均 exit 2（`measure_latency.py:1812-1817`） | 稳定判定的容差（度） |
| `--anchor` | 否 | `present` | 只能取 `present` / `command`（其它值参数解析失败 exit 2）；只在 `--mode tracking` 生效，`--mode rtt` 接受该参数并忽略 | 每步目标的锚点：`present` 锚实测位置，lead 只有一步；`command` 锚上一步发出的 target（首步回落实测），lead 跨步累积 |

`measure` 的 `--help` 用的是 `RawDescriptionHelpFormatter`，不打印默认值。上表默认值来自源码常量。

[§7.1](../runbook/4-experiments.md#exp-e3) 的「参数与理由」表只讲建议值与理由，默认值与约束以本文件为单一来源 → `../runbook/4-experiments.md#exp-e3`。
