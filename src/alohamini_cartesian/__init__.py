"""AlohaMini 笛卡尔遥操作的独立库。

本库把笛卡尔遥操作的产品逻辑与机器人运行时解耦：核心只依赖 numpy（控制层与运动学的
数值数学需要），只通过 :mod:`alohamini_cartesian.adapter.protocol` 定义的协议接触机器人。
可选依赖按需引入：运行时 ``lerobot``（runtime extra）、YAML 配置（``yaml`` extra）、
运动学求解器 ``placo``（``kinematics`` extra，``placo>=0.10.1,<0.11``；``control/arm_ik.py``
在函数内惰性 import，核心依赖仍只有 numpy）。
"""

__version__ = "0.7.0"
