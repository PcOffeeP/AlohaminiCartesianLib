# 跨阶段工程债归档（2026-11 瘦身存档）

本文件承接 2026-11-23 `TODO.md` 瘦身（ADR D25）时迁出的**已关闭事项**、**离线已收口仅余极端形态未在真机复现的尾项**，以及**低优先级边角细节**，保留完整历史背景便于日后检索；**现行阻塞与下一轮现场待办统一见仓库根 `TODO.md`**。

## 1 已关闭或由 ADR D24/D25 直接解决的事项

| 原事项 | 关闭原因 / 去向 |
| --- | --- |
| 左夹爪 51 tick 行程异常（旧代际 `2905048c28a2`） | 2026-11-10 重标定后现役 `2018..3483`（1465 tick / 128.8°），原前提失效；查证记录见 `docs/investigations/gripper-range-51-tick.md`。 |
| `docs/field-ops/next-mission.md` T0 卡口径与 D22 对齐 | 已完成文档对齐并关闭。 |
| 上游 URDF（`liyiteng/AM-ARM`）Apache-2.0 署名合规 | 已落地仓库根 `LICENSE` 与 `NOTICE` 并对齐 `pyproject.toml`（Phase 0 / D25）。 |
| 决定 `adapter/lerobot_version.py` 运行时调用点 | 按 D25（Q3-A）移除该孤立模块；运行时依赖边界由 `pyproject.toml` 与 `capability/detect.py` 能力探测承担。 |
| 三层限位 `calibration/limits.py` 未接入控制路径 | 按 D25（Q2-A）移除未接入模块与空模板 `configs/limits-observed.yaml`；待未来采集到真实硬件极限后再设计。 |
| `ArmHandle.move_to` 步长门禁导致从静止起步超 2° 拒发 | 按 D24 改为链式航点二分步进（`max|Δq| <= max_joint_step_deg`）+ 默认锁定初始姿态 + 暴露跟随容差参数。 |
| 历史文档行号锁维护摩擦 | 按 D25（Q1）改写 `tests/test_doc_line_references.py` 为文件路径存在性守卫。 |
| 手册 §4.1c 与收尾指纹同名覆盖风险 | 已在 `docs/field-ops/runbook/3-health.md` §4.1c 写明命名口径（过程指纹 vs `postpower` 指纹）。 |
| `Minimum_Startup_Force` 是否被 `configure()` 覆盖 | 已由 vendor 源码核清：`AlohaMini.configure()` / `configure_motors()` 无该寄存器写入点，写进 EPROM 后跨重启保持；仅余下次 Host 重启后复读确认。 |
| 腕部 1.375 tick 稳态残余与 deg→units 换算差额 | 已定位无换算 bug：系 `--anchor present` 下跟随滞后吃掉约 12% 步长 + 整数 tick 量化死区所致；需真发 1.0° 步进使用 `--anchor command`。 |
| 首次起 Host 本地 ssh 挂 600 s | 已由 `docs/field-ops/runbook/2-host.md` §6.1 三步启动法稳定规避；根因不再追查。 |

## 2 离线已收口、仅余低概率边界或真机未复现的观察项

1. **`benchmarks/alohamini_io` 边缘输入与退出码细粒度**：
   - 计时面值形态校验、非有限/非正 `--rate-hz` / `--timeout-s` / `--host-loop-hz` / `--settle-*` / `--delta-deg`、以及中途冻结均已离线修补并锁定测试；仅余极端坏对象（如自定义 `__float__` 抛 `RuntimeError`）落退出码 `3`、直接调用者传 `host_loop_hz=inf`、以及 `achieved_rate_hz` 在极小样本下的 `N/(N-1)` 偏置（500 行时约 +0.2%）。
   - `measure_latency.py` 的 sidecar 写盘阶段二次 Ctrl-C 分支无专项单测；tracking 横幅括号仍打印 `delta` 而非 `0.1 * delta`（被既有测试锁住，暂不改文案）。
   - `rtt` 逐帧能力探测隐含假定 `observation_sequence` 与 `latest_request_timing` 原子更新；若真机偶发误拒再按 ADR D12 复议。
2. **标定 CLI 的次要细节**：
   - `CalibrationRegistry.save()` 使用 `yaml.safe_dump` 会丢弃 YAML 文件头注释（记录本身无损，已在文件头注明）。
   - `tools/calib_health.py` 的 `_identity_field_diffs` 无法捕获「同一电机同时改名且改 `id`」，靠登记时人工核对兜底；其默认 stdout 新增的包络行尚无专项锁定用例。
   - `tools/calib_fetch.py` 在 `--record` 与 `--out` 同给时的 stdout 行序、以及 CLI 层 `calib_id` 过期用例（模块层已有 `test_a_stale_accepted_entry_with_a_different_calib_id_still_warns`）属低优先级补测项。
   - D23 删除的「`range` 中心偏离 2047」判据与肩族中心偏移测试：缺乏肩/腕物理依据前不恢复。
3. **测试与兼容边界的已知说明**：
   - `tests/test_control_api_lerobot_parity.py` 的 `_make_command_client()` 依赖 1 s wall-clock 窗口，`_StubZmq` 未桩 `zmq.Again`；两个 parity 文件仅在安装 vendor `lerobot` 的环境下运行，主环境跳过不代表差分对照通过。
   - V09 进程外零发送证据目前由 `examples/minimal_dry_run.py` 覆盖（臂 + disconnect）；夹爪/升降/底盘零发送由进程内单测与 `examples/robot_dry_run.py` 覆盖。
   - V11 仅靠离线用例覆盖 v1 schema 帧（Pi 已升级至 v2，无法再现 v1 宿主）；帧平滑回退已由 ADR D15 定为不做。
   - 4 路相机采集返回 `Frame is None` 尚未排查，不阻塞当前状态控制链路（内部读帧已默认 `include_cameras=False`）。
   - `src/` 未内置编码器型号→分辨率映射表，当前全部由 v2 指纹提供 18/18 电机 `resolution`。
   - `rise_90_time_s` 探针在残余 `> 0.1 × delta` 时为空（已确认原因）；2026-11-06 曾观测到 `arm_right_wrist_yaw` 单点温度 29℃、三轴相对 10-25 达成率波动及 `Torque_Enable=1` 外部写入源未定位，留作下次现场顺手核对项。
