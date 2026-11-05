"""真机 dry-run 示例：把厂商 ``AlohaMiniClient`` 注入本库 ``connect(dry_run=True)``。

这是「真 client 注入」的参考实现：用真实的厂商客户端打开本库会话，**零发送、只读观测**。

1. 这是什么。``examples/minimal_dry_run.py`` 用内存里的假 transport 演示调用形状；这份脚本
   换成真的 ``AlohaMiniClient``（厂商 ``lerobot_alohamini`` 的客户端），经
   ``connect(..., client_factory=...)`` 注入本库。脚本做三件事：读一次观测、对升降轴下一条
   dry-run 命令、断开。命令在 priming 阶段的 adapter 门禁就被拒（第 3 节），本会话
   ``send_action`` 一次都不会被调用；结尾把包在真 client 外面的计数代理断言出来（不是硬编码
   的 0）。
2. 真机怎么跑。厂商 client 只存在于装了 ``lerobot`` 的环境里，本库主环境（只有 numpy）没有
   它。两条命令：第一条把 vendor 快照与本库的 ``src`` 一起放进 ``PYTHONPATH``（本库没装进
   vendor 环境，所以要带 ``:src``），第二条用 vendor 环境的解释器跑本脚本；``--host`` 就是
   机器人 Pi 的 IP/主机名::

       export PYTHONPATH=<vendor fork>/src:src
       .venvs/alohamini-lerobot/bin/python examples/robot_dry_run.py --host <Pi 的 IP>

   在 Pi 本机上跑时省掉 ``--host``（默认 ``127.0.0.1``）。解释器路径与 ``--host`` 都按本机
   环境替换。主环境（无 ``lerobot``）下脚本走跳过分支：打印跳过文案、退出码 ``0``，不连设备、
   不联网。
3. 零发送的保证来自哪条库内路径。本脚本**不注入** ``JointKinematicAdapter``（也不建
   ``placo`` IK），所以命令根本进不了状态机；实际生效的是这三处：

   - ``src/alohamini_cartesian/control/api.py:860-862``：**priming 阶段的 adapter 门禁**。
     ``session._prime_state_machine()`` 在 ``LiftHandle.move_to`` 的 ``state.handle()`` 之前
     调用（``api.py:513``）；会话没有 adapter、且观测也没带显式的 ``settled``/``valid`` 时，
     它抛 ``MotionNotEnabled("motion requires a kinematic adapter; ...")``。命令在读第一帧
     反馈之后就被拒绝，没有任何动作进状态机。厂商 ``AlohaMiniClient`` 既不在观测字典里给
     ``_control_status``，也没有 ``latest_control_status`` 旁路属性（已按源码核对），
     ``_has_explicit_settled_valid`` 恒为 False，所以真机上同样落在这条分支；
   - ``src/alohamini_cartesian/control/api.py:674-675``：``connect(dry_run=True)`` 连初始保持
     都不发（``if not dry_run: self._send_initial_hold()``）；
   - ``src/alohamini_cartesian/control/api.py:913-914``：``disconnect()`` 的底座零速同样只在
     非 dry-run 下发。

   另外两处常被当成「零发送门禁」的代码在本脚本里**不可达 / 不适用**，不要按它们解释：
   ``api.py:519-520``（升降轴到位后的下发门禁）位于 ``state.handle()`` 之后，无 adapter 时
   前面已经抛错，走不到；``controller.py:169-172``（臂 ``CartesianController._send()`` 的
   dry-run 早退）属于臂控制器，而控制器只在 ``adapter is not None and ik_factory is not
   None`` 时才构造（构造条件见 ``api.py:653-672``），本脚本不建控制器。

   为什么命令的是升降轴而不是臂：臂的 ``move_by``/``move_to`` 需要 ``adapter`` 与
   ``ik_factory``（``placo``，可选 extra ``kinematics``），库内唯一的 adapter
   ``JointKinematicAdapter`` 需要每侧六关节完整的 ``signs``/``zeros``，现场没有标定值；
   伪造这些值与 ``AGENTS.md`` 第 3 节的证据纪律冲突，因此不在真机路径上指挥臂。升降轴同样要
   过 adapter 门禁，只是不需要 IK，所以在真机上的预期结果是**命令被拒**，而不是真的走到下发
   门禁。
"""

from __future__ import annotations

import argparse
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Any

from alohamini_cartesian.control import (
    AlohaMiniControl,
    FeedbackLost,
    MotionBlocked,
    MotionNotEnabled,
    MotionTimeout,
    connect,
)

# 默认值取自本机现场配置 ``configs/site.yaml``：Host 侧 robot_model 与观测请求窗口深度
# 必须与这里一致，否则状态键与观测时序都对不上。
_DEFAULT_ROBOT_MODEL = "alohamini2pro"
_DEFAULT_OBSERVATION_REQUEST_WINDOW = 3

# 默认跑在 Pi 本机（Host 常驻在那里）；从 Mac 连过去时用 ``--host <Pi 的 IP>`` 覆盖。
_DEFAULT_HOST = "127.0.0.1"

# 升降轴这次 dry-run 请求的位移（毫米）。dry-run 下不会真的动，读数前后应相等。
_LIFT_STEP_MM = 1.0


def build_client_factory(
    *,
    robot_model: str = _DEFAULT_ROBOT_MODEL,
    observation_request_window: int = _DEFAULT_OBSERVATION_REQUEST_WINDOW,
) -> Callable[..., Any]:
    """造一个注入给 ``connect(client_factory=...)`` 的真 client 工厂。

    返回的工厂签名是 ``(host, **kwargs) -> client``，与 ``connect`` 的调用形式一致。厂商
    ``lerobot`` 的 import 全部落在最内层工厂体内：主环境（没有 ``lerobot``）照样能 import
    本模块并拿到这个构造器，真调用时才抛 ``ImportError``。

    Args:
        robot_model: 厂商 ``AlohaMiniClientConfig.robot_model``，必须与 Host 侧一致。
        observation_request_window: 厂商 ``AlohaMiniClientConfig.observation_request_window``，
            观测请求窗口深度，必须 ``>= 1``。

    Returns:
        工厂 ``(host, **kwargs) -> AlohaMiniClient``；``**kwargs`` 原样透传给厂商 config。
    """

    def factory(host: str, **kwargs: Any) -> Any:
        """按 host 构造一个真 ``AlohaMiniClient``。

        Args:
            host: 机器人 Pi 的 IP/主机名，即厂商 config 的 ``remote_ip``。
            **kwargs: 透传给 ``AlohaMiniClientConfig`` 的额外字段。

        Returns:
            已构造、尚未 ``connect()`` 的厂商 client。
        """
        # 惰性导入：模块顶层不得出现 lerobot——主环境要能加载本脚本并打印跳过文案。
        from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient
        from lerobot.robots.alohamini.config_alohamini import AlohaMiniClientConfig

        config = AlohaMiniClientConfig(
            remote_ip=host,
            robot_model=robot_model,
            observation_request_window=observation_request_window,
            **kwargs,
        )
        return AlohaMiniClient(config)

    return factory


class _CountingTransport:
    """计数代理：转调真 client 的传输面，并真的数 ``send_action`` 被调用了几次。

    真机上这一层就是厂商 ``AlohaMiniClient``；包一层只为了拿到「下发了几次」这个读数，
    不改变任何行为。其余属性（``latest_robot_metadata``、``observation_sequence``、
    ``latest_control_status`` 这些旁路字段）经 ``__getattr__`` 原样转发。
    """

    def __init__(self, client: Any) -> None:
        """包住一个真实 client 并清零下发计数。

        Args:
            client: 厂商 ``AlohaMiniClient`` 实例（任何满足 ``RobotTransport`` 的对象）。
        """
        self._client = client
        self.send_action_calls = 0

    def connect(self) -> None:
        """转调真 client 的 ``connect()``。"""
        self._client.connect()

    def disconnect(self) -> None:
        """转调真 client 的 ``disconnect()``。"""
        self._client.disconnect()

    def get_observation(self, **kwargs: Any) -> Mapping[str, Any]:
        """转调真 client 的 ``get_observation()``，不做任何改造。

        Args:
            **kwargs: 原样透传（厂商签名的 ``include_cameras`` 在这里可选）。

        Returns:
            真 client 返回的一帧观测。
        """
        return self._client.get_observation(**kwargs)

    def send_action(self, action: Mapping[str, float]) -> Mapping[str, Any]:
        """先记一次下发，再转调真 client 的 ``send_action()``。

        Args:
            action: 归一化单位的动作映射。

        Returns:
            真 client 返回的动作生效后观测。
        """
        self.send_action_calls += 1
        return self._client.send_action(action)

    def prime_observation_request_window(self, *, include_cameras: bool) -> None:
        """转调真 client 的 ``prime_observation_request_window()``。

        Args:
            include_cameras: 是否连相机一起预热观测请求窗口。
        """
        self._client.prime_observation_request_window(include_cameras=include_cameras)

    def __getattr__(self, name: str) -> Any:
        """把未被本代理定义的属性转发给真 client。

        Args:
            name: 属性名。

        Returns:
            真 client 上同名属性的取值。
        """
        return getattr(self._client, name)


class _CountingFactory:
    """包住真工厂：造出的每个 client 都套上计数代理，并记下最近一个供计数。

    每次调用真工厂前不会预导入任何东西；``lerobot`` 的 import 仍只发生在真工厂体内，所以
    主环境里构造本对象也不会触发 ``ImportError``。
    """

    def __init__(self, factory: Callable[..., Any]) -> None:
        """记录被包裹的工厂（签名 ``(host, **kwargs) -> client``）。

        Args:
            factory: 由 :func:`build_client_factory` 造出的惰性工厂。
        """
        self._factory = factory
        self.last_transport: _CountingTransport | None = None

    def __call__(self, host: str, **kwargs: Any) -> _CountingTransport:
        """造一个计数代理，并把它记成 ``last_transport``。

        Args:
            host: 机器人 Pi 的 IP/主机名，原样透传给真工厂。
            **kwargs: 原样透传给真工厂。

        Returns:
            包住真 client 的计数代理。
        """
        self.last_transport = _CountingTransport(self._factory(host, **kwargs))
        return self.last_transport


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析命令行参数。

    Args:
        argv: 参数列表；``None`` 时取 ``sys.argv[1:]``。

    Returns:
        含 ``host``、``robot_model``、``observation_request_window`` 的命名空间。
    """
    parser = argparse.ArgumentParser(
        description="真机 dry-run：把厂商 AlohaMiniClient 注入本库 connect(dry_run=True)，零发送。"
    )
    parser.add_argument(
        "--host",
        default=_DEFAULT_HOST,
        help="机器人 Pi 的 IP/主机名，即厂商 config 的 remote_ip（默认 127.0.0.1，即 Pi 本机）",
    )
    parser.add_argument(
        "--robot-model",
        default=_DEFAULT_ROBOT_MODEL,
        help=f"必须与 Host 侧一致（默认 {_DEFAULT_ROBOT_MODEL}，同 configs/site.yaml）",
    )
    parser.add_argument(
        "--observation-request-window",
        type=int,
        default=_DEFAULT_OBSERVATION_REQUEST_WINDOW,
        help=(
            "观测请求窗口深度，必须 >= 1"
            f"（默认 {_DEFAULT_OBSERVATION_REQUEST_WINDOW}，同 configs/site.yaml）"
        ),
    )
    return parser.parse_args(argv)


def _is_lerobot_import_error(exc: ImportError) -> bool:
    """判断导入异常是否确属 ``lerobot`` 缺件（而不是被顺带包住的真故障）。

    ``ModuleNotFoundError``（``ImportError`` 子类）带 ``.name``；纯 ``ImportError`` 的
    ``.name`` 可能为 ``None``，此时按 ``str(exc)`` 里是否出现 ``lerobot`` 判断。两者都不命中
    就返回 ``False``，由调用方原样 ``raise``——不能把「厂商包缺失」之外的任何导入错误都当成
    「需要 vendor 环境」而退 0。

    Args:
        exc: 在 ``connect(...)`` 调用链里捕到的导入异常。

    Returns:
        确实是 ``lerobot``（或其子模块）缺件时为 ``True``。
    """
    name = getattr(exc, "name", None)
    if name is not None:
        return name == "lerobot" or name.startswith("lerobot.")

    return "lerobot" in str(exc)


def _print_vendor_skip(args: argparse.Namespace, exc: ImportError) -> None:
    """打印主环境（无 ``lerobot``）的跳过说明与真机跑法。

    只写 stdout、不写 Traceback：主环境没有 ``lerobot`` 是预期情况，不是故障。

    Args:
        args: 解析好的命令行参数（跑法提示里带上本次的 ``--host``）。
        exc: 触发跳过的导入异常，打印它的类型与消息。
    """
    print("跳过：这份脚本需要 vendor 环境（装了 lerobot 的 Python），本环境没有它。")
    print(f"导入失败原因：{type(exc).__name__}: {exc}")
    print("主环境按设计不连设备、不联网，退出码 0。")
    print("真机跑法（两行：第 1 行把 vendor 与本库的 src 一起放进 PYTHONPATH，第 2 行跑本脚本）：")
    print("  export PYTHONPATH=<vendor fork>/src:src")
    print(f"  .venvs/alohamini-lerobot/bin/python examples/robot_dry_run.py --host {args.host}")


def _print_observation(
    host: str, observation: Mapping[str, Any], transport: _CountingTransport
) -> None:
    """打印一帧真观测的读数：键数量、样本序号、metadata 来源与条目数。

    真 client 把 ``_robot_metadata`` 放在 ``latest_robot_metadata`` 旁路属性上，观测字典里
    通常没有它；两处都取不到时如实打印「缺失」，不补默认值、也不拿别的帧顶替。

    Args:
        host: 机器人 Pi 的 IP/主机名，只用于打印。
        observation: 真 client 返回的一帧观测。
        transport: 包住真 client 的计数代理（旁路属性经它转发）。
    """
    keys = sorted(str(key) for key in observation)
    print(f"host={host}：读一帧观测，键 {len(keys)} 个，前 8 个 {keys[:8]}")
    print(f"样本序号 observation_sequence={getattr(transport, 'observation_sequence', None)}")

    metadata = observation.get("_robot_metadata") or getattr(
        transport, "latest_robot_metadata", None
    )
    if metadata is None:
        print("metadata：缺失（观测字典与 latest_robot_metadata 旁路都没有）")
        return
    motors = metadata.get("motors")
    motor_count = len(motors) if isinstance(motors, Mapping) else None
    print(f"metadata：schema_version={metadata.get('schema_version')}，motor 条目 {motor_count}")


def _run_dry_run_lift_command(session: AlohaMiniControl) -> None:
    """对升降轴下一条 dry-run 命令：读高度、``move_by``、再读高度。

    本会话没有注入 ``JointKinematicAdapter``：``LiftHandle.move_by`` 走到
    ``session._prime_state_machine()`` 就会抛 ``MotionNotEnabled``
    （``src/alohamini_cartesian/control/api.py:860-862``），命令进不了状态机，也到不了下发
    门禁。命令被控制层拒绝时只打印原因、不打印 Traceback：拒绝本身也是「零发送」的一种结果，
    不该让读数消失。

    Args:
        session: ``connect(dry_run=True)`` 打开的会话。
    """
    print(
        f"命令：lift.move_by(delta_mm={_LIFT_STEP_MM})"
        "（dry_run=True，无 adapter，预期在 priming 阶段被拒）"
    )
    try:
        height_before = session.lift.height()
        session.lift.move_by(delta_mm=_LIFT_STEP_MM)
        height_after = session.lift.height()
    except (FeedbackLost, MotionBlocked, MotionNotEnabled, MotionTimeout) as exc:
        print(
            f"升降轴命令被控制层拒绝：{type(exc).__name__}: {exc}"
            "（命令在 priming 阶段被拒、未进入状态机，零发送不变）"
        )
        return

    print(
        f"升降高度读数：before={height_before:.2f} mm，after={height_after:.2f} mm"
        "（dry_run 下不动，两者应相等）"
    )


def main(argv: Sequence[str] | None = None) -> int:
    """真机 dry-run 主流程：注入厂商 client、读一次观测、下一条零发送命令、断开。

    命令在 priming 阶段的 adapter 门禁（``src/alohamini_cartesian/control/api.py:860-862``）
    就被拒，不进入状态机，所以本会话不会下发任何动作。

    Args:
        argv: 命令行参数；``None`` 时取 ``sys.argv[1:]``。

    Returns:
        进程退出码：主环境跳过分支与真机路径都是 ``0``。
    """
    args = _parse_args(argv)
    counting = _CountingFactory(
        build_client_factory(
            robot_model=args.robot_model,
            observation_request_window=args.observation_request_window,
        )
    )

    # 厂商 import 与 client 构造都发生在 connect 调用注入工厂的那一刻；主环境在这里抛
    # ImportError。只有它确属 lerobot 缺件时才走跳过分支（不打印 Traceback、退 0），
    # 其余导入错误原样往上抛。若异常发生在 client.connect() 之后，厂商 _connect() 自己会关
    # 两条 zmq socket 并销毁 context 再抛出（alohamini_client.py:160-167），本脚本不兜。
    try:
        session = connect(
            args.host,
            dry_run=True,
            client_factory=counting,
            clock=time.monotonic,
            sleeper=time.sleep,
        )
    except ImportError as exc:
        if not _is_lerobot_import_error(exc):
            raise
        _print_vendor_skip(args, exc)
        return 0

    transport = counting.last_transport
    if transport is None:
        raise RuntimeError("connect() 没有调用注入的工厂：拿不到计数代理，无法给出下发读数")

    try:
        # 只读的一次观测：先用 include_cameras=False 预热观测请求窗口，再按同样口径取帧。
        # 厂商 docstring（lerobot/robots/alohamini/alohamini_client.py:279-285）指出：开场前
        # 窗口里可能残留一整窗相机请求，直接返回它们会造成「无意的图像突发」；远程链路上也
        # 不该拖相机帧（本脚本只看状态键）。
        transport.prime_observation_request_window(include_cameras=False)
        observation = transport.get_observation(include_cameras=False)
        _print_observation(args.host, observation, transport)
        _run_dry_run_lift_command(session)
    finally:
        session.disconnect()

    assert transport.send_action_calls == 0, (
        f"dry_run=True 下不应下发动作，实际调用 {transport.send_action_calls} 次"
    )
    print(f"send_action 调用次数: {transport.send_action_calls} 次发送（dry_run=True，期望 0）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
