"""本库对机器人运行时的**唯一**接触面。

适配层之外的任何模块都禁止 import ``lerobot`` 的深层模块（如
``lerobot.motors``、``lerobot.robots.alohamini``）；需要机器人状态或动作时，
只依赖这里定义的 :class:`RobotTransport` 与 :class:`RobotSpec` 协议。这样
产品逻辑可以在不引入 lerobot 依赖的环境里被导入、测试与静态检查。
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class RobotTransport(Protocol):
    """机器人运行时的最小传输面：读观测、下发动作、预热观测请求窗口。"""

    def get_observation(self) -> Mapping[str, Any]:
        """返回一帧观测，键为关节名（如 ``"left_waist.pos"``）与相机名。"""
        ...

    def send_action(self, action: Mapping[str, float]) -> Mapping[str, Any]:
        """下发一帧动作为归一化单位，返回该帧生效后的观测。"""
        ...

    def prime_observation_request_window(self, *, include_cameras: bool) -> None:
        """预热观测请求窗口，避免首帧因相机请求而超时。"""
        ...


@runtime_checkable
class RobotSpec(Protocol):
    """机器人静态描述：型号与左右臂串口。"""

    robot_model: str
    left_port: str
    right_port: str
