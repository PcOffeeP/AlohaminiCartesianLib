# 标定 fixture 的来源（provenance）

这两份 JSON 是**真机标定原件的逐字节副本**，不是合成数据、不是手写的测试样本。改任何
一个字节都会换掉文件 sha256，`tests/test_calibration_identity.py` 的 sha256 用例随即变红
（该用例把下面两个 sha256 硬编码在测试代码里）。

| 文件 | 字节数 | sha256 | 来源（仓库外，只读） | 对应代际 |
| --- | --- | --- | --- | --- |
| `AlohaMiniRobot.20261022.mac-snapshot.json` | 2880 B | `6d089d3879b538a806dd5008dba66cde1bdb5a819aa7c4987becd0bf6b6e4c56` | `/Users/pcoffeep/pi-workspace/Cartesianlib-workbench/calibration/archive/AlohaMiniRobot.20261022.mac-snapshot.json` | `calib_id 2905048c28a2` |
| `AlohaMiniRobot.20261110.pi-live.json` | 2882 B | `d6594b999eb8533932adf20b6b5be413768c521f9f082109454cd323e183dfcf` | `/Users/pcoffeep/pi-workspace/Cartesianlib-workbench/calibration/AlohaMiniRobot.json` | `calib_id 4a80c0fd4421` |

补充说明：

- 第一份是归档时刻的 mac 侧快照，归档时刻按归档文件名记到日粒度：`20261022`（无更早的精确 UTC 时点证据，不得当精确时刻使用）。
- 第二份是 2026-11-10T11:58:15Z 重标定后的**现役**标定副本。
- `calib_id` 由 `alohamini_cartesian.calibration.identity.calib_id()` 实算得出（只取
  `id` / `drive_mode` / `homing_offset` / `range_min` / `range_max` 五个字段的规范化 sha256
  前 12 位），两份副本与登记表 `configs/calib-registry.yaml` 里的 `calib_id` 一致。
- 两份副本与上述原件逐字节相同（`cmp` 与 sha256 均已核对过）；原件侧的同一组 `file_sha256`
  与 `byte_size` 也记在登记表 `configs/calib-registry.yaml` 的对应记录里。

维护纪律：**不要为了「修好测试」去改这两份副本**。它们存在的意义就是「真机原件长这样」；
测试断言应当跟着原件走，而不是反过来。要加新代际的 fixture，从原件再拷一份并同时更新本表
与测试里的 sha256 常量。
