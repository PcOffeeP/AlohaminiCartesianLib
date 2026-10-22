"""运动学层：关节单位适配（``joint_adapter``）与单臂模型规格（``model_specs``）。

本包只依赖 numpy 与标准库；URDF 资产位于 ``kinematics/assets/am_arm200/``。
数值求解（placo）由 ``control.arm_ik`` 惰性引入，不在本包导入路径上。
"""
