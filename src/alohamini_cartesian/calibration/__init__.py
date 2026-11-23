"""标定代际的身份、包络判定与登记表。

三个子模块各管一件事，互不依赖控制路径：

* :mod:`~alohamini_cartesian.calibration.identity`：``calib_id`` 身份、结构校验、
  两代变化分类；
* :mod:`~alohamini_cartesian.calibration.envelope`：经验区间阈值与四态包络判定；
* :mod:`~alohamini_cartesian.calibration.registry`：代际登记表（``configs/calib-registry.yaml``）
  的读写，按 ``calib_id`` 幂等。
"""
