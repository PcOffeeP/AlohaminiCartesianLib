# 隔离环境与 PYTHONPATH 重建（原 §5.3）

本文件管什么：主环境没装 `lerobot` 时要用的隔离环境怎么从零重建、`PYTHONPATH` 两段各指到哪、哪些步实测过哪些没实测。现场跑 `measure_latency.py` 解释器不对时，从这里照抄。

<a id="sec-5-3"></a>
### 5.3 跨机 RTT（PC → Pi 真实网段）

背景一句：2026-10-25 时的两份 rtt CSV 均为 Pi 本机回环（`host=127.0.0.1`），量不到 PC 到 Pi 这一段；2026-11-05 已补跨机实测（结果见 §9 与 `docs/field-results.md`）。跨机这一次必须换解释器：用仓库内的隔离环境 `.venvs/alohamini-lerobot/bin/python`，因为主环境没有 `lerobot`，且按 `docs/ADR.md` D8 主环境要停在 numpy 2.3.x。

前置条件：第 0 节的网络访问授权；隔离环境已建（环境事实见本节末尾）。

命令原文（在仓库根下执行；`$PI_HOST`、`$S4DIR`、`$STAMP` 用第 2 节变量块的定义）：

```bash
# 零运动 RTT 形态：请求 1000 Hz、窗口 1（与 Pi 本机那两次同口径，才可比）
/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py \
  --host "$PI_HOST" --mode rtt --rate-hz 1000 --request-window 1 --samples 500 \
  --csv "$S4DIR/rtt/rtt-net-cross-$STAMP.csv"
```

这条命令与 5.2 的唯一差别是解释器：把 `uv run --no-sync python` 换成 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python`，因为主环境没有 `lerobot`。参数口径（`--mode rtt`、`--rate-hz 1000`、`--request-window 1`、`--samples 500`）必须与 Pi 本机那两次一致，否则两组数字不可比。

`--csv` 必须改名成 `rtt-net-cross-$STAMP.csv`：与 5.2 同名会在同一次会话里互相覆盖。

要留痕就把整条命令套进 `run`，步骤号 `5.3-rtt-net-cross`：

```bash
run 5.3-rtt-net-cross /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode rtt --rate-hz 1000 --request-window 1 --samples 500 --csv "$S4DIR/rtt/rtt-net-cross-$STAMP.csv"
```

判据：`exit 0` 且 `rtt/rtt-net-cross-$STAMP.csv` 有数据行。把该 CSV 里 `net_rtt_estimate_s` 的 p50 与 Pi 回环的 `6.176 ms` 并列写进现场记录（`docs/background.md` 记 `net_rtt_estimate_ms` p50 6.176；原始 CSV 是 `rtt/rtt-loopback-merged-20261025.csv`），两者之差就是 PC 到 Pi 这一段网段的开销。返回 `9` 或 `3` 的处置与 5.2 相同。

#### 解释器与 `PYTHONPATH`（2026-11-05 实测）

两条硬事实：

- 主环境不装 `lerobot`，直接 `uv run --no-sync python benchmarks/alohamini_io/measure_latency.py …` 会以 exit `3`、`No module named 'lerobot'` 失败。
- 隔离环境装的是构建出的 vendor wheel，不含 `latest_request_timing`；不把 `PYTHONPATH` 指向 fork 快照时，rtt 会以 exit `9` 结束（实测见 `stdout/5.3-rtt-net-cross-*`；重跑 `5.3-rtt-net-cross-2` 把 `PYTHONPATH` 指向 `6aee1f81` 快照后 exit 0）。

所以本节与 5.1、5.2、7.1 的测量命令都写成 `env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src .venvs/alohamini-lerobot/bin/python …`。快照取法：把只读 fork 克隆到 `/tmp`，不改 fork 本身。

```bash
git clone --branch p1a-merged --quiet /Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini /tmp/lerobot-vendor-6aee1f81
git -C /tmp/lerobot-vendor-6aee1f81 rev-parse HEAD
```

第二条应等于 Pi 运行时的 HEAD `6aee1f81`（本次实测输出 `6aee1f81d503be98972d605ed32189221a674607`）。

环境事实：

| 项 | 值 |
| --- | --- |
| 环境路径 | `/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot`（仓库内 `.venvs/`，已被 `.gitignore` 忽略，不进 `git status`） |
| 解释器 | Python 3.13.13 |
| `lerobot` 版本 | `0.6.1` |
| numpy | `2.2.6`（主环境 2.3.5，两边 site-packages 完全隔离） |
| 整目录体积 | `du -sh` = 881M |

已知限制：

1. 冻结快照：装的是构建出的 wheel，厂商仓库后续改动不会反映进来，要更新得重装。
2. 只能在 PC 侧（macOS arm64）跑。Pi 是 Linux aarch64，需要另一套 wheel，这个环境整体不可搬移。
3. 这套环境只验到 import 与 `--help`，**未做任何 `connect()`**。5.3 是它的第一次真连，输出形态以首次现场为准。
4. 不要把 `lerobot` 装进本仓库的 `.venv`：它的核心依赖钉 `numpy>=2.0.0,<2.3.0`，会把主环境的 numpy 从 2.3.5 降级，冲撞 `docs/ADR.md` D8。

产物：`rtt/rtt-net-cross-$STAMP.csv`、`stdout/5.3-rtt-net-cross-*`。

#### 重建命令（2026-11-08 实测）

一句话：`.venvs/alohamini-lerobot` 丢了、或换一台 PC 时，照下面这段重建；重建后本节的「环境事实」表三项版本（Python 3.13.13 / `lerobot` `0.6.1` / numpy `2.2.6`）与整目录体积 `881M` 都对得上，`pip freeze` 计数与 2026-11-05 那套相同（44 行），当天 `diff` 无输出。

先定性质：这个环境是标准 `venv`，不是 `uv venv`。依据是该环境 `pyvenv.cfg` 里两行关键内容——`command = /opt/miniconda3/bin/python3.13 -m venv …`、`include-system-site-packages = false`。重建用同一条 `venv` 命令、同一个 conda 解释器；`uv venv` 建出来是另一副骨架（`pyvenv.cfg` 内容与 pip 版本都不同），不在本节验证范围。

命令原文（在仓库根下执行；前五行变量按本机实际路径改）：

```bash
REPO=/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib
FORK=/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini
PY313=/opt/miniconda3/bin/python3.13
VENV="$REPO/.venvs/alohamini-lerobot"
TMP=/tmp/lerobot-rebuild

# 1 建环境（2.4 s / 12 M）
"$PY313" -m venv "$VENV"
"$VENV/bin/python" -V                     # 判据：Python 3.13.13
"$VENV/bin/python" -m pip --version       # 判据：pip 26.0.1

# 2 取 vendor 主分支快照（2.0 s / 357 M）；git archive 只读，不写 fork
git -C "$FORK" rev-parse refs/heads/main  # 判据：ec65318f4cc3d830242f6e987d43f46e6b61f0a5
mkdir -p "$TMP/src" "$TMP/wheel"
git -C "$FORK" archive --format=tar refs/heads/main | tar -x -C "$TMP/src"

# 3 造 wheel（4.3 s / 1.7 M）
"$VENV/bin/python" -m pip wheel "$TMP/src" --no-deps -w "$TMP/wheel"
ls -l "$TMP/wheel/lerobot-0.6.1-py3-none-any.whl"   # 判据：1804013 字节

# 4 装 wheel 与全部依赖（45.3 s / 875 M；本次 pip 缓存命中）
"$VENV/bin/python" -m pip install "$TMP/wheel/lerobot-0.6.1-py3-none-any.whl"

# 5 把 6 个随解析时间漂移的包钉回 2026-11-05 的值（8 s → 881 M）
"$VENV/bin/python" -m pip install "anyio==4.15.1" "click==8.5.0" "filelock==4.2.4" \
  "hf-xet==1.7.0" "mypy_extensions==1.1.0" "pyzmq==27.2.0"

# 6 去掉指向 /tmp 的一次性元数据（2026-11-05 那套没有这个文件）
rm -f "$VENV/lib/python3.13/site-packages/lerobot-0.6.1.dist-info/direct_url.json"

# 7 验收
"$VENV/bin/python" -c "import sys, numpy, lerobot; from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient; print(sys.version.split()[0], lerobot.__version__, numpy.__version__, AlohaMiniClient.__module__)"
"$VENV/bin/python" "$REPO/benchmarks/alohamini_io/measure_latency.py" --help > /dev/null   # 判据：exit 0
"$VENV/bin/python" -m pip freeze | wc -l   # 判据：44
du -sh "$VENV"                             # 判据：881M

# 8 清掉 /tmp 里的构建快照（回收 358 M；测量要的 vendor 快照另按上一小节取）
rm -rf "$TMP"
```

第 7 步第一条的期望原文（本次实测，一行）：

```text
3.13.13 0.6.1 2.2.6 lerobot.robots.alohamini.alohamini_client
```

逐步判据与实测值（2026-11-08，试装置于 `/tmp/lerobot-rebuild-test-20261108/venv`）：

| 步 | 做什么 | 判据 | 本次实测 |
| --- | --- | --- | --- |
| 1 | 建 venv | `python -V` = `Python 3.13.13`、`pip --version` = `pip 26.0.1` | 2.4 s，12 M |
| 2 | `git archive` 取 fork `main` 快照 | `rev-parse refs/heads/main` = `ec65318f…f0a5` | 2.0 s，快照 357 M |
| 3 | 造 wheel | `lerobot-0.6.1-py3-none-any.whl` 存在、1804013 字节 | 4.3 s，wheel 1.7 M |
| 4 | 装 wheel + 全部依赖 | freeze 里有 `numpy==2.2.6`、`torch==2.11.0`、`torchvision==0.26.0` | 45.3 s，875 M |
| 5 | 钉回 6 个漂移包 | `pip freeze \| wc -l` = 44 | 8 s，881 M |
| 6 | 删 `direct_url.json` | 该文件不存在 | 文件已不存在；耗时未单测 |
| 7 | 验收 | 上面那行期望原文 + `--help` exit `0` + `du -sh` = 881M | 四项全通过 |
| 8 | 清 `/tmp` 构建快照 | `$TMP` 不存在 | 5 s（`du` + `rm` 两命令合计），回收 358 M |

按上表相加，第 1–5 步 62 s（pip 缓存命中；其中第 4 步 45.3 s 全是装依赖）。`git archive` 只读：重建前后 `git -C "$FORK" status --porcelain \| wc -l` 都是 6，即 fork 上那 6 个既有未提交改动，未被本流程触及。

第 5 步为什么存在：pip 对 `>=x,<y` 区间取当前最新，2026-11-08 重新解析时有 5 个包比 2026-11-05 那套新（`anyio` 4.15.1→4.16.0、`click` 8.5.0→8.5.1、`filelock` 4.2.4→4.2.9、`hf-xet` 1.7.0→1.8.0、`mypy_extensions` 1.1.0→1.2.0），另有 `pyzmq` 27.2.0 不在 `lerobot` 的基础安装依赖内（挂在 `pyzmq-dep` extra 上：`Requires-Dist: pyzmq<28.0.0,>=26.2.1; extra == "pyzmq-dep"`，基础安装不会拉；本次全量安装后 freeze 里确实没有它），2026-11-05 那套是额外装的。这 6 条钉死后 `pip freeze` 与现环境逐行相同（44 行对 44 行，`diff` 无输出），`du -sh` 也从 875M 回到 881M。

第 6 步为什么存在：pip 26.0.1 装本地 wheel 会写 `site-packages/lerobot-0.6.1.dist-info/direct_url.json`（内容指向 `file:///private/tmp/.../lerobot-0.6.1-py3-none-any.whl`），2026-11-05 那套没有这个文件。它是可追溯元数据，不影响 import；删掉只为让两套一致。pip 26.0.1 没有跳过它的开关（`pip install --help | grep -i direct-url` 无输出，实测）。

与 `PYTHONPATH` 快照的关系（两套东西，别合并）：

| 项 | 取自 | 来源提交 | 源码含 `latest_request_timing` |
| --- | --- | --- | --- |
| 装进环境的 wheel | fork 的 `main` | `ec65318f` | 否 |
| 测量命令的 `PYTHONPATH` | fork 的 `p1a-merged` | `6aee1f81` | 是 |

实测对照：同一条 `python -c` 只换 `PYTHONPATH` 跑——不设时 `alohamini_client.py` 来自 `site-packages`，源码里搜不到 `latest_request_timing`；设成 `/tmp/lerobot-vendor-6aee1f81/src` 时来自该快照，搜得到。所以重建只解决主环境那条 `No module named 'lerobot'`（exit `3`），不解决 rtt 的 exit `9`：`PYTHONPATH` 快照仍按上一小节的 `git clone --branch p1a-merged --quiet …` 取，本小节不改那段；第 2 步的 `git archive` 只为造 wheel，不拿来做 `PYTHONPATH`。

本库没装进这个环境（事实句）：重建后 `site-packages` 里没有 `alohamini_cartesian` 的 `dist-info` 与 `.pth`，在仓库根下 `import alohamini_cartesian` 报 `ModuleNotFoundError: No module named 'alohamini_cartesian'`（实测）。替代做法是把 `<repo>/src` 加进 `PYTHONPATH`，与 vendor 快照用冒号并列（写法见 §4.4 那条 `run`）；**把 cwd 放在仓库根不能替代它**，cwd 只解决 `benchmarks/…`、`examples/…` 这类脚本路径。5.1、5.2、5.3、6.2、7.1 的测量命令只 import `lerobot` 与 numpy，不需要本库，所以那些命令不写第二段 `PYTHONPATH`。

实测范围（哪些步没实测，别当成已验证）：

1. 上面 8 步全部在 `/tmp/lerobot-rebuild-test-20261108/venv` 上跑通，解释器与现环境同源（`/opt/miniconda3/bin/python3.13`）。**`.venvs/alohamini-lerobot` 本身没有重建**：唯一没实测的差别就是 `VENV` 指向仓库内路径这一处。试装目录（删前 `du -sh` 1.2 G，含 venv 881 M + 快照 357 M）已删；`/tmp` 下只剩既有的 `/tmp/lerobot-vendor-6aee1f81`（789 M，上一小节的快照，保留）与 `/tmp/alohamini-clean-install`（888 M，不是本流程建的，未动）。
2. 冷缓存耗时未实测：本次 pip 缓存（209 M）命中，`torch-2.11.0-cp313-cp313-macosx_11_0_arm64.whl (80.6 MB)` 走 `Using cached`；无缓存时要下载，耗时取决于网速。
3. wheel 逐字节可复现未验证：本次 sha256 `74ffa6fe1864259b6c35e3d8b84eb5a53852b50fba30987c7b59a81717913765`，没跑第二次比对。
4. `connect()` 仍未验，与上面「已知限制 3」同口径。

**vendor 行号基准**（本文与本项目其它文档统一口径）：vendor（fork）源码行号一律按**对应提交**计，不按本地工作区——`ec65318f` = fork `refs/heads/main`；`6aee1f81` = `p1a-merged` = Pi 运行时 HEAD。
本地 fork 工作区有 6 个未提交改动，`git -C <fork> diff --numstat` 的**净增行数**是 `docs/alohamini/commands.md` +94、`examples/alohamini/teleoperate_bi.py` +13、`src/lerobot/robots/alohamini/config_alohamini.py` +12（同一条命令里另改了 `examples/alohamini/calibrate_bi.py`、`examples/alohamini/record_bi.py`、`tests/robots/test_alohamini_ros_compat.py`）。这三个数是增删相抵后的净增，**不是**引用行处的一律偏移：偏移随位置变（例：`docs/alohamini/commands.md` `:210` 在工作区是 `:263`，偏移 +53；`examples/alohamini/teleoperate_bi.py` `:90` 在工作区是 `:103`，偏移 +13）。所以一律用 `git show <ref>:<path>` 取行，不要在工作区 grep。
本仓库内的行号引用由 `tests/test_doc_line_references.py` 守（判「文件能解析 / 行号在范围内 / 目标行非空」）；vendor 侧引用不在该守卫内。

