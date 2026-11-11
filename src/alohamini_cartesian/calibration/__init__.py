"""标定代际的身份、登记表与三层限位判定。

三个子模块各管一件事，互不依赖控制路径：

* :mod:`~alohamini_cartesian.calibration.identity`：``calib_id`` 身份、结构校验、
  合理性告警、两代变化分类；
* :mod:`~alohamini_cartesian.calibration.registry`：代际登记表（``configs/calib-registry.yaml``）
  的读写，按 ``calib_id`` 幂等；
* :mod:`~alohamini_cartesian.calibration.limits`：URDF / 实测 / 标定三层限位的判定与实测层
  解析。**未接入控制路径**，理由见该模块 docstring。
"""
