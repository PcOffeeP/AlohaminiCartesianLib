# AlohaMini Cartesian 项目交接与新人上手指南（Windows + WSL2 同学版）

> **本文写给接手本项目的两位同学。** 你们已经具备树莓派 5（Raspberry Pi 5）的 SSH 登录权限，本文帮你们在 **Windows 笔记本**上从零搭好环境、搞清 GitHub PR 协作规则、看懂当前项目进度，并按安全规范双人配合推进剩下的真机验收。

---

## 1. 10 分钟项目全貌与当前进度

### 1.1 一句话目标

**AlohaMini Cartesian**（`AlohaminiCartesianLib`，当前版本 `v0.7.0`，`main` 基线提交 `37e7416`）是一个运行在 PC 端的独立 Python 库，把「机械臂末端要走到空间中哪个坐标（**笛卡尔控制**，即直接给 `x, y, z` 目标，而不是手算 6 个电机角度）」翻译成 6 个关节的电机指令，再通过局域网发给树莓派 5 上常驻的厂商主程序（**Host 进程**，`alohamini_host`）。

- **要替代什么**：原先 AlohaMini 2 Pro 机械臂需要人手握遥操作主臂（leader arm）牵着走；本库改成 PC 侧调用 `connect` / `move_to` / `move_by` 直接控制。
- **架构边界**：本库核心只依赖 `numpy`，**不修改树莓派上的厂商仓库 `lerobot_alohamini`**；逆运动学求解由 **IK（Inverse Kinematics，逆运动学求解器，基于 `placo` QP）** 在 PC 端离线计算，通信复用厂商 `AlohaMiniClient`（经 ZMQ 5555/5556 端口连 Host）。

### 1.2 当前做到哪了（已完成 vs 仍缺实机证据）

本项目按 13 条产品验收基线 **V01–V13**（详见 [acceptance-baseline.md](acceptance-baseline.md)）管理进度，架构取舍记在 **ADR D1–D26**（Architecture Decision Records，详见 [ADR.md](ADR.md)）。软件离线逻辑与工具链已经收口，**当前主要工作是双人补齐剩余真机验证**。

| 验收维度 / 条目 | 当前状态 | 关键数字与结论（已核对） |
| --- | --- | --- |
| **软件离线测试集** | **已通过** | 主环境实测 `1576 passed, 2 skipped`（主环境缺 `lerobot` 时跳过 2 个真实总线差分文件，非失败） |
| **V01**（纯 PC 脱离主臂控制） | **已通过** | 按 [ADR.md](ADR.md) D20 分级判据（达成率 `≥50%`，肩类步末残余 `≤2°`、其余轴 `≤2 tick`；**tick** 是舵机编码器最小步进刻度，`4096 tick = 360°`，`1 tick ≈ 0.0879°`）：2026-11-08 `--anchor command` 两轮双侧过门槛，达成率 `77.36%` / `79.12%` / `82.64%` |
| **V06**（底层过流保护） | **实机侧豁免** | 按 D26 裁决：不改厂商固件且无外接电流表时实机结构性不可达，不阻塞 `v0.8.0` 发版；软件侧 hold 退让单测保留 |
| **V09**（dry-run 严格零发送） | **软件侧关闭** | 语义决定无实机项，软件侧全覆盖（`send_action` 调用 0 次） |
| **V10**（Gate 运行时门禁拒绝） | **已收口** | 按 D26 裁决：已取得 2026-10-25 v1 宿主 Gate F 拒绝与 2026-11-06 错机型 Gate B 拒绝（均退出码 `6`、零命令）；其余不可达分支由离线单测锁定 |
| **V11**（v1 帧兼容） | **软件侧锁定** | Pi 升级后只广播 `schema_version = 2`（v2 元数据），v1 解析与换算回落由离线单测锁定；帧平滑回退按 D15 不做 |
| **V12**（单位口径一致性） | **旧代际已过，新代际待重取** | 2026-10-27 旧代际 `2905048c28a2` 核对 18/18 电机差异 0、exit `0`；2026-11-10 重标定换代际后需重采指纹复核 |
| **V13**（IK 实时性能预算） | **目标 PC 已通过** | 2026-11-23 Apple M4 实测 `solve p99 = 0.0297 ms`（预算 `≤7.0 ms`）、`step p99 = 1.3706 ms`（预算 `≤15.0 ms`）、`within_budget=true`；你们在 WSL2 笔记本上建议重跑一次确认本机算力 |
| **V02–V05、V07、V08** | **软件已过，实机未取** | 对应双臂隔离、底盘/升降互锁、停止恢复、断流保护、启动零位移 hold、位置残差门禁，是你们后续的主要实机目标 |

### 1.3 接手前必须知道的一次硬件变动（2026-11-10 重标定）

2026-11-10 树莓派上的现役标定文件 `AlohaMiniRobot.json` 被完整重标，**标定代际身份 `calib_id`**（18 台电机 × 5 个核心标定字段算出的 sha256 前 12 位）从旧代际 `2905048c28a2` 切换为新代际 **`4a80c0fd4421`**，带来一好一险两件事：

- **好消息（左夹爪已修好）**：原先左夹爪 `arm_left_gripper` 行程异常偏窄（只有 `51 tick` / `4.5°`），重标后已恢复为 `2018..3483`（**`1465 tick` / `128.8°`**），与右夹爪（`1479 tick` / `130.0°`）一致。
- **需核查的风险（右肩行程缩水）**：右肩俯仰轴 `arm_right_shoulder_lift` 标定范围从 `2319 tick`（`203.8°`）缩水到 **`1143 tick`（`100.5°`）**（`905..2048`），左肩水平旋转 `arm_left_shoulder_pan` 也从 `2700` 缩到 `2286 tick`。目前本库的包络检查脚本 `tools/calib.py fetch` 会把右肩判为 `IMPLAUSIBLE`（退出码 `1`）。**现场第一步必须手工推臂核查**：这 `100.5°` 是机械真实极限，还是重标定时手没扫满？
- **供电变更备注**：前任已将树莓派与机械臂改为分开供电，但目前仅有空载状态检查，尚未在带载运动下验证，因此**不放宽任何实验判据或残余门槛**。

### 1.4 接下来要做的 6 件事（Phase A → D）

从现在到 `v0.8.0` 可交付版本，所有未完事项按执行顺序拆成 6 步（与 [TODO.md](../TODO.md) 同步）：

| 阶段 | 序号 | 要做什么 | 对应验收目标 / 阻塞解除 |
| --- | --- | --- | --- |
| **Phase A**（单臂闭环与标定） | **1** | 实机跑 `tools/calib.py fetch` 核对现役标定，并手工推臂核查右肩 `1143 tick`（S1） | 明确是否重标或记入接受名单，为 V12 与 E7 扫清限位隐患 |
| **Phase A** | **2** | 在新代际（`4a80c0fd4421`）重采 v2 指纹，跑 `tools/verify_unit_consistency.py` | 18/18 电机差异 0、exit `0` → **关闭新代际 V12** |
| **Phase A** | **3** | 执行 E8 标定（`tools/calibrate_kinematics.py`），产出左右臂各自的 `signs` / `zeros` JSON | **解除 E7 `move_to` 真机运行阻塞**（缺文件会报 exit `15`） |
| **Phase A** | **4** | 执行 E7 真机闭环（`tools/run_cartesian_move.py`）与 `+Z` 9 cm 物理量测 | 跑通笛卡尔控制闭环，实测检验 URDF 杆长（误差目标 `≤1 mm`） |
| **Phase B**（双臂独立验证） | **5** | 右臂受控试动 + 双臂与左右夹爪独立性验证 | 动一侧时对侧观测与动作键零串值 → **关闭 V02** |
| **Phase C**（整机协同验收） | **6** | 底盘/升降/机械臂分阶段互锁、停止恢复、反馈超时与残差门禁真机验证 | **关闭 V03 / V04 / V05 / V07 / V08**，推进 Phase D 发版 |

---

## 2. Windows + WSL2（Ubuntu）从零搭环境指南

### 2.1 为什么必须用 WSL2（Ubuntu），不要在原生 Windows 里装

1. **`placo==0.10.1` 没有 Windows 预编译包**：PyPI 上 `placo` 整个 `0.10.x` 系列（`0.10.0` / `0.10.1` / `0.10.2`）**零 Windows wheel**，仅提供 macOS 与 `manylinux_2_28` Linux（x86_64 / aarch64）wheel。在 Windows 原生 Python 里安装会触发 C++ / Pinocchio 源码编译并大概率失败；在 **WSL2 Ubuntu（22.04 / 24.04，glibc ≥ 2.28）** 中可直接命中 Linux 预编译 wheel，几秒装完。
2. **不需要折腾 USB 串口透传**：机械臂的 USB 串口全都插在树莓派 5 上，PC 端开发与控制**全程走局域网（SSH 22 端口 + ZMQ 5555/5556 端口）**，完全不需要用 `usbipd` 把 USB 设备透传进 WSL2。

### 2.2 准备 WSL2 Ubuntu 与拉取两个仓库

在 Windows PowerShell（管理员）执行一次安装（若已装好 Ubuntu 22.04/24.04 可跳过）：

```powershell
wsl --install -d Ubuntu-24.04
```

进入 WSL2 Ubuntu 终端后，**务必把代码放在 Linux 家目录（如 `~/workspace/`），不要放在 `/mnt/c/...`**（跨盘符 I/O 很慢，且文件权限与虚拟环境软链接容易异常）：

```bash
# 1. 安装基础工具与 uv（高速 Python 包管理器）
sudo apt update && sudo apt install -y git curl jq openssh-client python3-venv
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"

# 2. 拉取本库与厂商运行时 fork 的 main 分支
mkdir -p ~/workspace && cd ~/workspace
git clone https://github.com/PcOffeeP/AlohaminiCartesianLib.git alohamini-cartesian-lib
git clone https://github.com/PcOffeeP/lerobot_alohamini.git lerobot_alohamini

# 3. 核对厂商 fork 的提交哈希（应为 6aee1f81，与树莓派运行版本一致）
git -C ~/workspace/lerobot_alohamini rev-parse --short HEAD
```

> **选错 `lerobot` 来源会踩两个大坑**：
> - **严禁直接 `pip install lerobot`（PyPI 版）**：PyPI 版不含 `alohamini` 机器人模块，且会把 `numpy` 静默降级到 `<2.3`。
> - **严禁使用旧提交 `ec65318f`**：旧提交缺少 `latest_request_timing`，会导致 RTT 延迟测量直接以退出码 `9` 失败。必须使用 `PcOffeeP/lerobot_alohamini` 的 `main` 分支（`6aee1f81`）。

### 2.3 搭建两套独立虚拟环境（隔离依赖冲突）

为什么不能只建一个环境？因为本库日常开发和 GitHub CI 保持轻量、零 `lerobot` 导入（只有 `numpy` + `placo` + `PyYAML` + `pytest`/`ruff`），而真机联调必须引入厂商 `lerobot 0.6.1`（会拉入 `torch`、`pyzmq` 等约 1 GB 依赖，且与 `placo` 的 `numpy` 版本区间存在元数据冲突）。把两套环境放在仓库的 `.venv/` 和 `.venvs/` 下，两者均已被 `.gitignore` 忽略，互不污染。

#### 环境 ①：开发与离线测试环境（`.venv`，日常写代码、跑单测、对齐 CI 用）

```bash
cd ~/workspace/alohamini-cartesian-lib

# 用 Python 3.13 创建隔离虚拟环境（与 GitHub CI 版本一致；3.12 也可）
uv venv --python 3.13 .venv

# 安装本库（editable）+ yaml / kinematics extras + 开发工具
uv pip install --python .venv/bin/python -e ".[yaml,kinematics]" "pytest>=8.0" "ruff>=0.6"
```

> **防踩坑提醒（非常重要）**：本仓库刻意不维护 `uv.lock`。以后用 `uv run` 执行命令时**务必带上 `--no-sync`**（例如 `uv run --no-sync python -m pytest ...`）；如果不加 `--no-sync`，`uv` 会自行解析依赖并在仓库根目录生成一个未跟踪的 `uv.lock` 文件。**严禁提交 `uv.lock`**。

#### 环境 ②：真机联调合一环境（`.venvs/s4-move-to`，连树莓派采指纹、标定、跑 `move_to` 用）

```bash
cd ~/workspace/alohamini-cartesian-lib
mkdir -p .venvs

# 1. 创建独立真机联调环境（推荐 Python 3.13，与环境 ① 同版本）
uv venv --python 3.13 .venvs/s4-move-to

# 2. 从刚拉取的厂商 fork（6aee1f81）以 editable 方式安装 lerobot，并补齐 pyzmq
uv pip install --python .venvs/s4-move-to/bin/python -e ~/workspace/lerobot_alohamini "pyzmq>=26.2.1,<28.0.0"

# 3. 安装 placo、pytest、PyYAML，并以 --no-deps 可编辑安装本库
uv pip install --python .venvs/s4-move-to/bin/python "placo==0.10.1" "pytest>=8.0" "PyYAML>=6.0"
uv pip install --python .venvs/s4-move-to/bin/python --no-deps -e .

# 4. 核对导入路径：确认 lerobot 来自本地 fork（6aee1f81）且本库来自 src/
.venvs/s4-move-to/bin/python -c "
import numpy, lerobot, placo, alohamini_cartesian
import lerobot.robots.alohamini.alohamini_client as alohamini_client
assert hasattr(alohamini_client, 'AlohaMiniClient')
print('OK:', 'numpy=', numpy.__version__, 'lerobot=', lerobot.__version__)
print('client:', alohamini_client.__file__)
print('lib:', alohamini_cartesian.__file__)
"
```

> **已知依赖版本例外（正常现象，无需恐慌）**：在 `.venvs/s4-move-to` 里安装 `placo==0.10.1` 会把 `numpy` 升到 `2.3.5`，略微超出 `lerobot 0.6.1` 元数据声明的 `numpy<2.3.0`（运行 `pip check` 会提示这一条告警）。该组合已在离线差分测试（61/61）和 `--dry-run` 中跑通，但离线通过不能替代真机证明——**首次连真机时务必先跑只读的 `--limits-only` 验证真实 `connect()` 与观测解析无异常**。

### 2.4 环境搭建验收命令（4 条全过才算搭好）

在 `~/workspace/alohamini-cartesian-lib` 目录下依次执行：

```bash
# 1. 代码风格与格式检查（应全部通过、无报错）
uv run --no-sync ruff check .
uv run --no-sync ruff format --check .

# 2. 离线全量测试（应显示 1576 passed, 2 skipped；不要再额外加 -q）
uv run --no-sync python -m pytest tests -o addopts=""

# 3. 离线 IK 性能基准（V13 预算检查：确认 JSON 中 "within_budget": true）
uv run --no-sync python benchmarks/alohamini_ik/benchmark_arm_ik.py --json

# 4. 笛卡尔路径离线干跑（零网络、零发送，确认 exit 0 且生成 JSON）
uv run --no-sync python tools/run_cartesian_move.py --dry-run --points 7 --out /tmp/move-dryrun.json
```

如果想顺便验证环境 ②（`.venvs/s4-move-to`）里的真实 `lerobot` 差分对照测试，再跑一条：

```bash
.venvs/s4-move-to/bin/python -m pytest tests/test_kinematics_lerobot_parity.py tests/test_control_api_lerobot_parity.py -o addopts=""
```

---

## 3. 面向同学的 GitHub PR 协作流程

本仓库此前主要由 AI Agent 协作维护，所以根目录的 `AGENTS.md` 里写了不少针对 Agent 的纪律（比如必须走 `git worktree`、`work/*` 分支名、以及 macOS 清理脚本）。**两位同学接手后，只需要遵守 GitHub 实际生效的规则即可，不用照搬 Agent 专属束缚。**

### 3.1 哪些是 GitHub 真正强制的，哪些不是

以下规则已按仓库真实 Ruleset（`protect-main`）与 `.github/workflows/` 核实：

| 类别 | 具体规则 | 说明 |
| --- | --- | --- |
| **GitHub 强制** | 禁止直接 `git push` 到 `main`、禁止强推（`--force`）与删 `main` | 所有改动必须建新分支并提 Pull Request 合并 |
| **GitHub 强制** | PR 合并前必须通过全部 **3 项 CI 检查** | ① `ruff`（`ruff check .` + `ruff format --check .`）<br>② `pytest`（Python 3.13 下跑 `pytest tests -o addopts=""`）<br>③ `pr-template-check`（检查 PR 正文格式） |
| **GitHub 强制** | PR 正文必须满足 [PR 模板](../.github/PULL_REQUEST_TEMPLATE.md) 的 3 个必填小节 | 必须原样保留 3 个二级标题：`## 一句话总结`、`## 提交步骤清单`、`## 准入检查实测结果`；去掉 `<!-- ... -->` 注释后每节均不能为空，且**不能残留默认占位符 `<commit hash / 简述>`**（第 4 节 `## 重点审查与风险提示` 推荐填写，不硬拦截） |
| **GitHub 自动行为** | 合并后自动删除远端分支 | Ruleset 无强制审批人数限制（`required_approving_review_count = 0`），允许 `merge` / `squash` / `rebase`，且已开启合并后自动清理远端分支；**推荐两人互相看一眼再以 Merge Commit 合并** |
| **非强制（可忽略）** | `AGENTS.md` 里的 Agent 规则、`work/*` 前缀、`git worktree`、`tools/sync_and_clean.sh` | GitHub 不检查分支前缀，也不要求用 worktree；你们用普通的 `feat/...`、`fix/...`、`docs/...` 分支即可 |

> **改文档特别提醒**：`pytest` 会运行 `tests/test_doc_anchors.py` 和 `tests/test_doc_line_references.py`。写 Markdown 时，如果链接到其他仓库文档并带有 `#锚点`，该锚点必须对应目标文档里显式声明的 HTML 锚点 ID（如 `exp-e7`、`exit-codes`，**不能用中文标题自动生成的 slug**）；或者直接链接整份文件（不带 `#`）。

### 3.2 日常开发到提 PR 的 6 步标准命令

```bash
# Step 1: 同步最新 main 并切出自己的任务分支（名称清晰即可，如 feat/e8-calib-results）
git checkout main && git pull origin main
git checkout -b feat/<短任务名>

# Step 2: 修改代码或文档后，在本地跑齐与 CI 一致的检查（格式不齐时先跑 ruff format .）
uv run --no-sync ruff check .
uv run --no-sync ruff format --check .
uv run --no-sync python -m pytest tests -o addopts=""

# Step 3: 查看改动文件，显式暂存目标文件（禁止 git add -A，严禁提交 uv.lock 或几十 MB 的原始 CSV）
git status
git add <具体修改的文件路径>

# Step 4: 提交到本地分支
git commit -m "docs: 记录新代际 V12 单位口径核对结果"

# Step 5: 推送分支到 GitHub
git push -u origin feat/<短任务名>

# Step 6: 打开 GitHub 仓库网页点 "Compare & pull request"（目标分支 main）
#         按模板填完 3 个必填小节（替换掉 <commit hash / 简述>），贴入 Step 2 的实测输出，
#         等 ruff / pytest / pr-template-check 三项全绿后请搭档确认并合并。
```

---

## 4. 双人现场实操分工与上电前 5 条安全红线

### 4.1 两人天然分工与口令约定

现场操作机械臂**严禁单人一边盯终端一边盲跑运动命令**，推荐固定以下分工：

| 角色 | 站位与职责 | 关键动作 |
| --- | --- | --- |
| **A 同学（机器人安全员 / 实操手）** | 全程站在机器人旁，手随时能按到物理断电开关 | ① 断电前扶住机械臂放到低位<br>② 扭矩释放态下手工推臂（S1）与摆参考位形 C2b（E8）<br>③ 贴尺、立角尺、画起点标线并重复 3 次读数（E7）<br>④ 运动档全程目视防碰撞，异常立刻断电 |
| **B 同学（电脑操作员 / 记录员）** | 守在 Windows（WSL2）笔记本前操作终端 | ① SSH 连树莓派 5，检查磁盘与串口<br>② 按三步法启停 Host（只用 `kill -INT`）<br>③ 执行标定、采指纹与运动命令，盯紧退出码与 stderr<br>④ 按场次目录整理日志与实验记录 |

**发令口令**：每次准备跑运动命令（敲回车前），B 同学大声报清「准备动哪条臂、往哪走多少度/厘米」，A 同学确认「手在断电开关上、周围空场无遮挡、机械臂在低位或有承托」并回复「可以」，B 同学再回车。

### 4.2 上电前必须背熟的 5 条安全铁律

1. **停 Host 只能用 `kill -INT`，断电前必须先扶臂放低位**：
   Host 收到 `SIGINT`（`kill -INT <真实python PID>`）才会执行 `disable_torque_on_disconnect=True` 释放舵机扭矩；用 `SIGTERM` 或 `kill -9` 强杀不会释放扭矩，且失去看门狗保护。**注意**：停 Host 用的 PID 必须取 `ps` 查出的真实 `python` 进程号（不是 `setsid` 打印的 `$!`）。另外，无论怎么停 Host，**拉闸断电瞬间机械臂都会完全失力下坠**，A 同学必须先扶着机械臂放到低位再切电，防砸桌面。
2. **`placo` URDF 没有碰撞检测，防撞桌全靠人眼**：
   本库使用的 URDF 已剥离 collision mesh，IK 求解器不知道桌面、机身和升降柱在哪里。任何路径规划都只保证数学收敛，物理上会不会撞桌只能靠 A 同学目视确认。
3. **所有真实运动档必须有人手守断电开关**：
   包括 `--anchor command`、E8 里的 `1°` 单步 `sign` 判定、`tools/run_cartesian_move.py --enable-motion`。无人守在机器人旁时，只允许跑只读体检、`--dry-run` 或 `--limits-only`。
4. **`signs` / `zeros` 严禁手猜，严禁左右臂互抄**：
   `signs`（6 关节方向 `±1`）与 `zeros`（6 关节零点偏置，单位度）和舵机固件的 `drive_mode` / `homing_offset`（单位 tick）在不同层，绝不能混算。右臂代码虽复用左臂 URDF 几何模型，但电机安装零点各自独立，必须单独摆姿、单独做 `1°` 单步测 `sign` 并独立生成 JSON（缺文件时脚本会以退出码 `15` 拒绝运行）。
5. **不修改树莓派上的厂商代码**：
   树莓派 `/home/pi5/lerobot_alohamini` 保持只读运行，不要为了「让某个测试过去」去改树莓派代码或手动篡改标定 JSON。

### 4.3 按顺序推进的 4 步核心现场路线图（Phase A → B → C）

**开场准备（每次到现场先做，约 5 分钟）**：确认机器人和笔记本在同一局域网，获取树莓派当前 IP，在 WSL2 终端初始化本次会话变量、本地证据目录与 [1-prepare.md §2](field-ops/runbook/1-prepare.md#sec-2) 的 `run` 留痕包装函数（[2-host.md §6.1](field-ops/runbook/2-host.md#host-start) 的 Host 三步法等后续命令块都会直接调用 `run`），做完 `0.9-probe false` 自检后再检查树莓派磁盘余量与串口软链接：

```bash
export PI_HOST="<树莓派当前IP>"              # 例如 10.34.170.98，每次连之前先确认
export PI="pi5@$PI_HOST"
export STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
export S4DIR="$HOME/alohamini-evidence/s4-$(date -u +%Y%m%d)"
mkdir -p "$S4DIR"/{capture,rtt,tracking,configs,stdout}

# 1. 定义 run 留痕包装函数（与 docs/field-ops/runbook/1-prepare.md §2 同源）
#    每次调用落盘 $S4DIR/stdout/<步骤号>-<STAMP>.{cmd,log,exit}，并保留被调命令真实退出码
run() {
  local step="$1"; shift
  local base="$S4DIR/stdout/$step-$STAMP"
  printf '%s\n' "$*" > "$base.cmd"
  "$@" >"$base.log" 2>&1
  local code=$?
  echo "$code" > "$base.exit"
  cat "$base.log"
  echo "[exit $code] $base"
  return $code
}

# 2. 自检留痕链路（预期打印 [exit 1]，且 0.9-probe-$STAMP.exit 内容为 1；否则先修好再往下走）
run 0.9-probe false
cat "$S4DIR/stdout/0.9-probe-$STAMP.exit"

# 3. 测 SSH、查树莓派磁盘余量（上次仅剩 4.8G，务必先看）与串口设备
ssh "$PI" 'df -h / ~ && ls -l /dev/am_arm_follower_* /dev/ttyACM* 2>&1'
```

> 提示：如果后续要运行 `tools/calib.py fetch --host "$PI_HOST"`，该脚本底层带 `-o BatchMode=yes`（禁止交互输密码），请先用 `ssh-copy-id "$PI"` 配好免密登录。

接下来严格按 **Step 1 → Step 4** 推进 Phase A 收口（完整细节见各锚点手册）：

| 步骤 | 目标与操作要点 | 关键命令与通过判据 |
| --- | --- | --- |
| **Step 1**<br>抓标定 + S1 推臂核查右肩 `1143 tick` | ① 从 Pi 抓取现役标定（代际 `4a80c0fd4421`）并核对与 Pi 原件 `sha256` 一致（因右肩 `1143 tick`，此步预期返回退出码 `1` 提醒 `IMPLAUSIBLE`）。<br>② 确认 Host 已用 `kill -INT` 停止且扭矩已释放后，A 同学轻推 `arm_right_shoulder_lift` 与 `arm_left_shoulder_pan` 到两端物理极限，B 同学按 [3-health.md §4.2](field-ops/runbook/3-health.md#sec-4-2) 读寄存器 `Present_Position` 与 `Min/Max_Position_Limit`。 | `uv run --no-sync python tools/calib.py fetch --host "$PI_HOST" --out "$S4DIR/capture/calib-$STAMP.json"`<br>**判据**：若推臂实测超出标定 range → 说明重标未扫满，需先用厂商工具重扫并重新登记；若确实顶到机械限位 → 记入 `configs/calib-envelope-accepted.yaml` 或后续带 `--allow-implausible "实测理由"`。详见 [next-mission.md §3.3](field-ops/next-mission.md#mission-s1)。 |
| **Step 2**<br>起 Host + 新代际重采指纹复核 V12 | ① 确认当前 shell 已完成上方「开场准备」的 `run` 函数初始化与 `0.9-probe false` 自检，按 [2-host.md §6.1](field-ops/runbook/2-host.md#host-start) **三步法** 启动树莓派 Host（验证 `5555` 与 `5556` 端口 `LISTEN`，并记下 `ps` 中的真实 python PID）。<br>② 用合一环境 `.venvs/s4-move-to` 采一份新代际 v2 指纹。<br>③ 同会话内把新指纹与 Step 1 抓回的标定做单位口径比对。 | `.venvs/s4-move-to/bin/python benchmarks/alohamini_io/capture_robot_fingerprint.py --host "$PI_HOST" --lerobot-version 0.6.1 --out "$S4DIR/capture/fingerprint-$STAMP.json"`<br>`uv run --no-sync python tools/verify_unit_consistency.py "$S4DIR/capture/fingerprint-$STAMP.json" "$S4DIR/capture/calib-$STAMP.json"`<br>**判据**：`robot_metadata.schema_version == 2`（注意不是顶层 `schema_version: 1`），`verify_unit_consistency.py` 输出 **18/18 电机差异 0、退出码 `0`** → 新代际 V12 关闭。 |
| **Step 3**<br>E8 `signs` / `zeros` 标定 | ① **先在安全低位定 `signs`**（承接 Step 2 已运行的 Host，A 同学手守断电开关）：对左臂 6 个关节逐轴跑一次 `+1°` 单步（`measure_latency.py --mode tracking --enable-motion --joint arm_left_<joint>.pos --delta-deg 1.0 --steps 1`），看 CSV 中该轴实测角 `measured_deg` 相对基线的增/减定出 `sign`（增=`+1`，减=`-1`；**勿肉眼看 `wrist_roll`**，它 1° 末端原点位移 `0 mm`），写成扁平六键 JSON `10.8-signs-left-$STAMP.json`。<br>② **释放扭矩、摆正并核对参考位形 C2b**：用 `kill -INT <真实python PID>` 停 Host 释放扭矩（A 同学全程承托手臂），按 [4-experiments.md §10.8](field-ops/runbook/4-experiments.md#exp-e8) 逐关节目视与垂线/角尺判据摆成参考位形 **C2b**（URDF 运动学角 `(0, -72, +73, 0, 0, 0)°`：大臂竖直向上、小臂水平朝正前方、腕部顺直）；人手扶稳不动，重新按 [2-host.md §6.1](field-ops/runbook/2-host.md#host-start) 三步法启动 Host 并复核 C2b 姿态未偏移。<br>③ **在未移动的 C2b 姿态下立刻算 `zeros`**：**不得在此时再下发任何运动命令**，立刻跑 `tools/calibrate_kinematics.py` 连机只读采帧，按当前 C2b 实时观测算出扁平六键 `zeros` 并自动跑 FK 自检（`calibrate_kinematics.py` 按 `zero = q_ref - sign * motor_deg` 用实时位姿反推零点；若摆好 C2b 后又做过 `+1°` 单步或被碰歪，必须先 `kill -INT` 释放扭矩、重新摆正并核对 C2b 再重起 Host 采帧，否则位移会直接变成 `zeros` 的系统性零点偏差且同帧自检无法察觉；离线复算亦可用 `--from-observation-json` 传入移动前保存的 C2b 快照 JSON，含 `observation` 与 `metadata` 键）。右臂严格独立重跑 ①→③。 | `.venvs/s4-move-to/bin/python tools/calibrate_kinematics.py --site configs/site.yaml --side left --signs-json "$S4DIR/stdout/10.8-signs-left-$STAMP.json" --host "$PI_HOST" --zeros-out "$S4DIR/stdout/10.8-zeros-left-$STAMP.json"`<br>**判据**：退出码 `0`、`self_check` 的 `max_error_deg <= 3.0°`，且末端位置与 C2b 离线 FK `(-0.001, -0.316, +0.234) m` 在 `±0.05 m` 量级内吻合。详见 [4-experiments.md §10.8](field-ops/runbook/4-experiments.md#exp-e8)。 |
| **Step 4**<br>E7 `move_to` 真机闭环与 `+Z` 9 cm 实测 | 严格按 **①a → ①b → ② → ④ → ③** 顺序跑：<br>• **①a 只读限位窗口（`--limits-only`）**：零发送，同时检验 `numpy 2.3.5` 下真机连接与 6 轴有效限位窗口。<br>• **①b 离线 `--dry-run`**：已在 2.4 节跑过，可复跑对齐真机 seed。<br>• **② 单次 `+Z` 9 cm 真机运动（`--enable-motion --points 1`）**：自 ADR D24 起默认保持起始 FK 姿态并自动航点二分（每周期跳变 `≤2.0°`），不会再出现旧版腕部大幅翻转死锁。<br>• **④ 物理量测**：② 跑完静止 2 s 后立刻量测末端夹爪尖 `+Z` 实走距离。<br>• **③ 7 点路径（`--points 7`）**：② 与 ④ 正常后再跑连续多点路径。 | ①a：`.venvs/s4-move-to/bin/python tools/run_cartesian_move.py --site configs/site.yaml --host "$PI_HOST" --limits-only --signs-json "$S4DIR/stdout/10.8-signs-left-$STAMP.json" --zeros-json "$S4DIR/stdout/10.8-zeros-left-$STAMP.json" --out "$S4DIR/stdout/10.7-limits-$STAMP.json"`<br>②：把 `--limits-only` 换成 `--enable-motion --points 1 --timeout-s 10.0`，输出改 `10.7-single-$STAMP.json`。<br>**判据**：①a 与 ② 退出码 `0`；④ 满足 `\|实测值 − 90 mm\| ≤ 1 mm`（详见下节量具指南与 [4-experiments.md §10.7](field-ops/runbook/4-experiments.md#exp-e7)）。 |

**Phase A 收口后的后续阶段（Phase B → C）**：
- **Phase B（`TODO` #5 → V02）**：用右臂独立标定的 `signs`/`zeros` 复现 Step 4；动左/右一侧时记录双侧观测与动作键，确认对侧零串值；同时验证已修好的左夹爪（`1465 tick`）与右夹爪独立开合。
- **Phase C（`TODO` #6 → V03 / V04 / V05 / V07 / V08）**：依次取证机械臂与底盘/升降分阶段互锁（V03）、`stop`/`resume` 停止优先（V04）、反馈冻结/断流拦截（V05）、`connect` 不额外回位且首条手臂指令为零位移 hold（V07）、以及超残差硬门禁零下发（V08）。

### 4.4 量具实操说明：实验室只有一把直尺行不行？

[4-experiments.md §10.7](field-ops/runbook/4-experiments.md#exp-e7) 要求 ④ 步量测使用「钢卷尺或 30 cm 钢直尺（最小刻度 1 mm）+ 美纹纸 + 垂线或角尺」，是因为要做的是**毫米级几何验收（`≤ 1 mm`）**，用来检验厂商 URDF 杆长是不是准确的：

1. **为什么不能徒手悬空拿尺**：手拿尺子没有固定参照物，稍微晃一下起点就漂 2–3 mm，远超 1 mm 判据。所以必须在运动**前**把尺子固定住，并在尺旁做一条**起点的物理标线**（不能靠记忆记起点）。
2. **为什么必须校竖直（防歪斜余弦误差）**：量的是夹爪固定指尖（`Fixed_Jaw`，非活动指）沿基座系 `+Z` 方向的**纯竖直位移**。尺子哪怕歪了 15°，会把 90 mm 读成约 93 mm，横向漂移也会混进读数。
3. **为什么要静等 2 s 并读 3 次**：运动结束后等手臂完全稳住 2 s，由 A 同学同一视角重复读 3 次取中位数；若 3 次之间极差 `> 2 mm`，说明基准漂了，必须重固定重测。

**如果实验室手头只有一把直尺，怎么低成本凑齐达标工具？**

| 手册原要求 | 实验室低成本达标替代 | 达标条件 |
| --- | --- | --- |
| 钢卷尺 / 30 cm 钢直尺 | **1 mm 刻度的钢直尺或硬质塑料直尺**（不要用软布尺） | 最小刻度清晰到 1 mm |
| 垂线 / 角尺 | **一本厚精装书或规整硬纸盒的 90° 直角边**紧贴桌面，把直尺贴着直角立直 | 尺面竖直、与 `+Z` 方向平行 |
| 美纹纸 | **透明胶带 / 便签纸 / 创可贴**把尺子底座和直角靠山固定在桌面或支架上，用细笔在便签/胶带上画一条**起点的物理对齐刻线** | 启动前已有实物起点线，读数全程尺不挪动 |

> **诚实记录底线**：按上面替代法满足「1 mm 硬尺 + 直角立直 + 固定起点实物标线 + 等静止 2 s 读 3 次极差 ≤ 2 mm」，即可计为正式量测。如果现场**真的只徒手悬空拿直尺粗量了一下**，在记录里**只能写「粗测参考（量具不足）」**，**绝对不能**写成「正式通过 `≤ 1 mm` 签收」，也不得拿 JSON 里的 FK 预测值冒充实测数。

---

## 5. 常用命令速查、证据归档与高频排错

### 5.1 常用命令速查表

> 所有开发/离线命令在 WSL2 的 `~/workspace/alohamini-cartesian-lib` 根目录执行；真机联调命令统一用 `.venvs/s4-move-to/bin/python`。

| 用途 | 命令 |
| --- | --- |
| 代码规范 + 格式检查 | `uv run --no-sync ruff check . && uv run --no-sync ruff format --check .` |
| 自动格式化代码 | `uv run --no-sync ruff format .` |
| 离线全量测试 | `uv run --no-sync python -m pytest tests -o addopts=""` |
| 仅跑文档锚点与路径守卫 | `uv run --no-sync python -m pytest tests/test_doc_anchors.py tests/test_doc_line_references.py -o addopts=""` |
| 真实 `lerobot` 差分测试 | `.venvs/s4-move-to/bin/python -m pytest tests/test_kinematics_lerobot_parity.py tests/test_control_api_lerobot_parity.py -o addopts=""` |
| 离线 IK 性能基准（V13） | `uv run --no-sync python benchmarks/alohamini_ik/benchmark_arm_ik.py --json` |
| 抓现役标定并跑包络检查 | `uv run --no-sync python tools/calib.py fetch --host "$PI_HOST" --out "$S4DIR/capture/calib-$STAMP.json"` |
| 检查本地标定代际健康状态 | `uv run --no-sync python tools/calib.py health --calib-file "$S4DIR/capture/calib-$STAMP.json"` |
| 登记新标定代际（确认无误后） | `uv run --no-sync python tools/calib.py register --from-file "$S4DIR/capture/calib-$STAMP.json" --write` |
| 核对 V12 单位口径一致性 | `uv run --no-sync python tools/verify_unit_consistency.py "$S4DIR/capture/fingerprint-$STAMP.json" "$S4DIR/capture/calib-$STAMP.json"` |
| 查 Pi 5 磁盘余量与大目录 | `ssh "$PI" 'df -h / ~ && du -sh ~/* ~/.cache/* 2>/dev/null \| sort -hr \| head -n 20'` |
| 查 Pi 5 Host 进程与端口 | `ssh "$PI" 'ps -eo pid,args \| grep alohamini_host \| grep -v grep; ss -ltnp \| grep -E ":5555\|:5556"'` |
| **安全停止 Pi 5 Host（释放扭矩）** | `ssh "$PI" 'kill -INT <真实python PID>'`（停完用上一条复核端口无 `LISTEN`，扶臂放低位再断电） |

启动 Host 前，请先在当前 WSL2 shell 完成第 4.3 节「开场准备」（与 [1-prepare.md §2](field-ops/runbook/1-prepare.md#sec-2) 同源的会话变量、`run` 留痕包装函数与 `run 0.9-probe false` 自检，确认写入退出码 `1`），再照抄 [2-host.md §6.1](field-ops/runbook/2-host.md#host-start) 的三步法命令块（既避免单条前台 SSH 挂住 10 分钟不返回，也确保 `run 6.1a-host-pid` 正常落盘真实 python PID 与 `5555`/`5556` 端口核验，不报 `run: command not found`）。

### 5.2 证据归档位置约定

- **原始大文件留在本地，不要提交进 Git**：每次现场窗口建一个以 UTC 日期命名的目录（如 `~/alohamini-evidence/s4-YYYYMMDD/`），保持 5 个标准子目录（`capture/`、`rtt/`、`tracking/`、`configs/`、`stdout/`，列义详见 [artifacts.md §11](field-ops/reference/artifacts.md#sec-11)）。
- **结论与数字通过 PR 入库**：每完成一轮现场实验，把关键数字、命令与退出码整理进 [field-results.md](field-results.md)（按时间倒序新增日期小节），并同步更新 [acceptance-baseline.md](acceptance-baseline.md) 的对应实机证据列与 [TODO.md](../TODO.md)。
- **历史草稿说明**：前任开发机本地曾有一份 `2026-11-10-recalibration.md` 逐字段草稿笔记，未随代码入库；但该次重标定的关键结论、4 个大幅变动关节明细以及 18 电机新代际身份 `4a80c0fd4421` 均已完整记录在本仓库的 [field-results.md](field-results.md)、[next-mission.md](field-ops/next-mission.md#mission-limit-risk) 与 `configs/calib-registry.yaml` 中，且你们首次连 Pi 执行 `tools/calib.py fetch` 即可获取树莓派现役标定原件。

### 5.3 高频排错表（含退出码 `2` / `6` / `9` / `10–15`）

| 现象 / 退出码 | 常见原因 | 排查与处置 |
| --- | --- | --- |
| **Pi 5 磁盘余量告急** | 2026-11-10 侦察时根分区 29 G 已用 23 G（**仅剩 4.8 G / 83%**）；本次交接前未能远程连通 Pi 5 实时复测 | **首次 SSH 必查** `df -h / ~`。优先把采集产物 `scp` 拉回笔记本，不在 Pi 上囤大日志。Pi 上曾留有 6.3 G 的旧目录 `lerobot_alohamini.legacy-ec65318f-20261025`，确认不再回滚后可按需清理，**切勿删错现役 `/home/pi5/lerobot_alohamini`** |
| **SSH 连不上 / Pi 5 IP 变化** | 实验室网段 `pi5.local`（mDNS）常无法解析，且 DHCP 重启后会更换历史 IP（曾用过 `10.34.170.98`、`10.34.159.106`） | ① 确认树莓派已上电且与笔记本连同一局域网；② 在路由器/校园网列表或给树莓派接显示器运行 `hostname -I` 查新 IP；③ 更新 `export PI_HOST=<新IP>`；④ 若 `calib.py fetch` 报 exit `4`，先手动 `ssh "$PI"` 确认免密登录与主机指纹正常 |
| `No module named 'lerobot'` 或 `capture`/`measure` 报 **exit `3`** | 错用了开发环境的 `uv run --no-sync python`（`.venv` 刻意不装 `lerobot`），或 Host 未启动 / IP 不通 | 改用 `.venvs/s4-move-to/bin/python`，并检查树莓派 Host 是否监听 `5555` / `5556` |
| **exit `2`**（各脚本通用） | 命令行参数/配置错误（如 `--delta-deg 0`、`run_cartesian_move.py` 忘传 `--enable-motion`、`calib.py` 互斥参数同传或全缺） | 读 stderr 补齐/修正参数；不涉硬件，可放心重跑 |
| **exit `6`**（`capture` / `measure`） | **一码两义**：在 `capture` 表示宿主 metadata 非 `schema_version == 2` 或 `motors` 为空；在 `measure` 表示命中运行时门禁 **Gate A–F**（发命令前的 6 道检查）并零下发拒绝 | 读 stderr 确认命中的是哪道 Gate（详见 [gates.md §10](field-ops/reference/gates.md#exit-codes) 与 [Gate A–F 表](field-ops/reference/gates.md#gate-a)），核对 `--joint`、`--robot-model` 与 Host 版本 |
| **exit `8`**（`measure`） | 结束时的收尾 hold 命令被拒绝，停机指令可能没送到 | **按「机械臂可能仍在动」处置：先断电，再排查**（见 [2-host.md §8.1](field-ops/runbook/2-host.md#sec-8-1)） |
| **exit `9`**（`measure --mode rtt`） | 导入了不含 `latest_request_timing` 的旧版 `lerobot`（例如错用了 `ec65318f` 或旧 wheel） | 检查 `alohamini_client.__file__`（`.venvs/s4-move-to/bin/python -c "import lerobot.robots.alohamini.alohamini_client as m; print(m.__file__)"`）与 `git -C ~/workspace/lerobot_alohamini rev-parse --short HEAD` 是否为 `6aee1f81` |
| **exit `10`**（`run_cartesian_move.py`） | `OutOfRange`：IK 求解或直线路径预检被拒（超关节限位或超位置残差 `5 mm`） | 查输出 JSON 的 `moves`，对比 ①a `--limits-only` 限位窗口，缩小位移或调整起始姿态 |
| **exit `11`**（`run_cartesian_move.py`） | `MotionTimeout`：超过 `--timeout-s` 仍未在容差内稳住 | 查 stderr 与 JSON 是否含 `FOLLOW_TIMEOUT`、有效限位窗口是否过窄，或适度增大 `--timeout-s` |
| **exit `12`**（`run_cartesian_move.py`） | `FeedbackLost`：超过 `feedback_timeout_s`（默认 1.0 s）无新观测样本 | 检查局域网丢包（`ping "$PI_HOST"`）与树莓派 Host 进程是否存活 |
| **exit `13`**（`run_cartesian_move.py`） | `MotionBlocked`：状态机阻断或 adapter 连续 3 个周期报错 | 先检查 `signs` / `zeros` 文件与 Host v2 metadata 是否齐全 |
| **exit `14`**（`run_cartesian_move.py`） | 未启用运动、缺少 `lerobot` / `placo` 运行时或连接失败 | 确认使用了合一环境 `.venvs/s4-move-to/bin/python` 且 Host 已就绪 |
| **exit `15`**（`run_cartesian_move.py` / `calibrate_kinematics.py`） | 缺少 `--signs-json` / `--zeros-json`，或文件不是扁平六键 JSON（比如多包了一层 `"signs": {...}`） | 先按 Step 3（E8）产出并用 `jq '.signs'` / `jq '.zeros'` 拆出扁平六键 JSON，严禁手猜 |
| **`calib.py` 报 exit `1` / `3` / `4`** | `fetch` 的 `1` = 有 `IMPLAUSIBLE`（非阻断提醒，如右肩 `1143 tick`）；`health` 的 `1` = 新代际未登记、`3` = 可疑、`4` = 损坏或会话代际漂移；`fetch` 的 `4` = SSH 失败/超时或文件非法（不写快照） | 查 [gates.md §10](field-ops/reference/gates.md#exit-codes) 逐脚本对照表；`calib.py fetch` 报 `4` 时先确认免密 SSH 可直通 |
| **PR 检查 `pr-template-check` 爆红** | 漏掉 3 个必填 H2 标题之一、小节为空（只剩注释），或残留 `<commit hash / 简述>` | 直接在 GitHub PR 网页点 Edit 补齐后保存，该 workflow 监听 `edited` 事件会自动重跑 |
| **不小心生成了 `uv.lock`** | 跑 `uv run` 时忘加 `--no-sync` | 执行 `rm uv.lock` 删掉，确保 `git status` 干净，不要提交 |

### 5.4 核心文档导航表

需要深入了解代码设计或现场逐条命令时，按这张表直接跳转：

| 你现在想做什么 | 直接看这份文档 |
| --- | --- |
| 15 分钟看懂代码分层架构、模块职责与术语消歧 | [ARCHITECTURE.md](ARCHITECTURE.md) |
| 查看下一轮现场任务书与 S1 / T1 / T3 详细前后置 | [field-ops/next-mission.md](field-ops/next-mission.md) |
| 到现场从开场到收尾的 8 步标准流程总入口 | [field-ops/README.md](field-ops/README.md) |
| Host 三步法启动与 `kill -INT` 扭矩纪律 | [field-ops/runbook/2-host.md](field-ops/runbook/2-host.md#host-start) / [§8.1](field-ops/runbook/2-host.md#sec-8-1) |
| E7（`move_to` + 卷尺）与 E8（`signs`/`zeros` 标定）完整实验卡 | [E7 (§10.7)](field-ops/runbook/4-experiments.md#exp-e7) / [E8 (§10.8)](field-ops/runbook/4-experiments.md#exp-e8) |
| 全部脚本退出码与 Gate A–F 完整定义 | [field-ops/reference/gates.md](field-ops/reference/gates.md#exit-codes) |
| 现场故障处置与 CSV / sidecar 列定义 | [troubleshooting.md](field-ops/reference/troubleshooting.md#sec-12) / [artifacts.md](field-ops/reference/artifacts.md#sec-11) |
| 查看 13 条验收基线（V01–V13）的当前填实进度 | [acceptance-baseline.md](acceptance-baseline.md) |
| 查历史现场实测数字台账（按时间倒序） | [field-results.md](field-results.md) |
| 查 Python API、配置覆盖与 `deg_per_unit` 换算细节 | [library-guide.md](library-guide.md) |
| 查某个设计为什么这么定（ADR D1–D26） | [ADR.md](ADR.md) |
| 查看当前未完成待办清单 | [TODO.md](../TODO.md) |
