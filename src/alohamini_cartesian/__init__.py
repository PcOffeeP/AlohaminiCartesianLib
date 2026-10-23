"""AlohaMini 笛卡尔遥操作的独立库。

本库把笛卡尔遥操作的产品逻辑与机器人运行时解耦：核心只依赖 numpy（控制层与运动学的
数值数学需要），只通过 :mod:`alohamini_cartesian.adapter.protocol` 定义的协议接触机器人。
可选依赖按需引入：运行时 ``lerobot``（runtime extra）、YAML 配置（``yaml`` extra）、
运动学求解器 ``placo``（仅在 ``control.arm_ik`` 内惰性使用，未声明为依赖）。
"""

__version__ = "0.3.0"
