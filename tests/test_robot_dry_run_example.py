"""``examples/robot_dry_run.py`` 的验收契约（真机 client 注入示例）。

示例已存在，本文件锁的是它当前必须成立的行为，不再有「预期全红」的红阶段语义。

这个示例补的缺口是「真 client 注入」：今天所有实机运动证据都来自
``benchmarks/alohamini_io/measure_latency.py``，它不 import 本库任何模块；
``src/alohamini_cartesian/control/api.py`` 至今零实机证据。示例的职责是给出这条
参考路径——用厂商 ``AlohaMiniClient`` 走本库的 ``connect(dry_run=True)``，零发送、
只读观测。

契约要点（按主题分组）：

- A：主环境（无 ``lerobot``）下子进程跑示例，退出码 0、stdout 出现跳过文案
  ``需要 vendor 环境`` 且该文案给出跑法（含 ``PYTHONPATH``），stderr 不含 ``Traceback``。
  主环境必须优雅跳过，不是崩。子进程带一份剥掉 ``PYTHONPATH`` / ``PYTHONHOME`` /
  ``VIRTUAL_ENV`` 的环境：否则调用方环境里指向 vendor ``src`` 的 ``PYTHONPATH`` 会让
  示例走真机分支去 ``client.connect()``，测试就不再离线。
- B：AST 级守卫。模块**import 期作用域**不得 import ``lerobot``——递归下探 ``try:`` /
  ``if TYPE_CHECKING:`` / ``with`` / ``match`` 的分支体与类体，只在函数定义处停手；
  函数体内的厂商 import 必须存在（惰性导入，构造器本身在无 lerobot 的主环境里也能
  import 与调用）；``connect(...)`` 调用必须带 ``dry_run=True``，且不得出现
  ``dry_run=False`` 的 ``connect``。
- C：真机分支的**离线**覆盖。往 ``sys.modules`` 塞一个最小假 ``lerobot`` 包，在同一个
  进程里跑 ``main(["--host", ...])`` 的真机分支（不连设备、不联网、不 sleep 等网络），
  断言：退出码 0；读数行 ``send_action 调用次数: 0 次发送``（正则取数 == 0）；观测读数行
  出现且键数对得上（确实读过观测）；假 client 自己的 ``send_action`` 计数为 0（独立于
  被测代码计数代理的第二重证据）；命令步的结局被如实打印为「被拒 / MotionNotEnabled」。
  夹具用 ``monkeypatch.setitem`` 注入并在 teardown 恢复 ``sys.modules``，不污染
  ``tests/test_capability.py`` 与两个 parity 文件的跳过判定。
- D：防「硬编码 0 次发送」假绿。这里**不用**源码常量守卫（旧守卫要求 ``print`` 的参数里
  含字面量 ``0 次发送``，而旧文案里的「期望 0 次发送」能让硬编码冒充真读数：计数为 3 时
  stdout 也含「0 次发送」）。现行口径是三条：
  1. 运行时读数——真机分支里 stdout 的读数行取数必须等于真计数值（见 C）；
  2. 反例自证——让计数代理真的被 ``send_action`` 一次，同一条读数行必须变成
     ``1 次发送``（断言具体数字 1）；再加一条端到端反例：``main()`` 在计数为 1 时以
     ``断言失败`` 收场（``实际调用 1 次``），证明这个数字来自真计数；
  3. AST 级守卫——结尾那条 ``print`` 的 f-string 插值里必须有 ``send_action_calls``
     属性访问（``FormattedValue`` 里能找到），防止有人把插值换成硬编码字面量。
- E：协议守卫。计数代理 ``isinstance(proxy, RobotTransport)`` 为真，并同时钉住三个关键
  方法的**实际行为**（``send_action`` 自增、``get_observation`` 返回 Mapping、
  ``prime_observation_request_window`` 转发到 client）。``runtime_checkable`` 协议只查
  方法是否存在，所以另加一条反例：只有 ``__getattr__`` 的转发壳**不**通过协议检查。

全程离线：不连设备、不联网，不依赖 ``lerobot`` / ``placo`` 是否安装。外部动作只有拉起
一个子进程跑示例脚本、从路径加载示例模块（后者靠示例自己的 ``if __name__ == "__main__":``
收口保证不会执行 ``main()``）、以及在同一进程里用假 ``lerobot`` 跑真机分支。
"""

from __future__ import annotations

import ast
import contextlib
import dataclasses
import importlib.util
import io
import os
import re
import subprocess
import sys
from collections.abc import Mapping
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest

from alohamini_cartesian.adapter.protocol import RobotTransport

REPO_ROOT = Path(__file__).resolve().parents[1]
EXAMPLE_PATH = REPO_ROOT / "examples" / "robot_dry_run.py"

SUBPROCESS_TIMEOUT_S = 60

# 主环境（无 lerobot）跳过分支的两个契约字面量：一句说明「为什么跳过」，一句给出跑法。
SKIP_NOTICE_LITERAL = "需要 vendor 环境"
HOWTO_RUN_LITERAL = "PYTHONPATH"

# 真机路径读数行的形态；契约 D 用正则取数，不做子串匹配。
ZERO_SEND_LITERAL = "0 次发送"
READOUT_RE = re.compile(r"send_action 调用次数: (\d+) 次发送")
OBSERVATION_SEQUENCE_RE = re.compile(r"样本序号 observation_sequence=(\d+)")
OBSERVATION_KEY_COUNT_RE = re.compile(r"读一帧观测，键 (\d+) 个")

# 计数器在 AST 里对应的属性名：``<x>.send_action_calls``。
SEND_ACTION_COUNTER_ATTR = "send_action_calls"

# 子进程只在这个白名单之外裁环境：这三个变量能让示例误以为自己在 vendor 环境里。
STRIPPED_ENV_VARS = ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV")

# 真机分支用到的 host 名。它是假的：没有一个字符会被送到网络上（假 lerobot 全程内存）。
STUB_HOST = "stub-host"

# 假 lerobot 的假 client 返回的观测帧：含升降轴高度与若干臂关节键。
STUB_OBSERVATION: dict[str, Any] = {
    "lift_axis.height_mm": 142.5,
    "arm_left_shoulder_pan.pos": 1.0,
    "arm_left_shoulder_lift.pos": -2.0,
    "arm_right_shoulder_pan.pos": 3.5,
    "arm_right_shoulder_lift.pos": -4.5,
}


# ---------------------------------------------------------------------------
# 通用 helper：AST、子进程输出、失败信息
# ---------------------------------------------------------------------------


def _is_lerobot_module(name: str) -> bool:
    """判断模块名是否为 ``lerobot`` 或其子模块。

    Args:
        name: ``import`` 语句里的模块名（``ast.ImportFrom.module`` 或 ``alias.name``）。

    Returns:
        是 ``lerobot`` 体系时为 ``True``。
    """
    return name == "lerobot" or name.startswith("lerobot.")


def _imported_lerobot_names(node: ast.Import | ast.ImportFrom) -> list[str]:
    """给出单个 import 节点里 import 到的 lerobot 模块名。

    Args:
        node: ``ast.Import`` 或 ``ast.ImportFrom`` 节点。

    Returns:
        该语句 import 到的 lerobot 模块名列表（非 lerobot 的为空）。
    """
    if isinstance(node, ast.Import):
        return [alias.name for alias in node.names if _is_lerobot_module(alias.name)]
    if node.level != 0 or node.module is None:
        return []

    return [node.module] if _is_lerobot_module(node.module) else []


def _module_scope_import_nodes(tree: ast.Module) -> list[ast.Import | ast.ImportFrom]:
    """收集**模块 import 期作用域**里的 import 语句节点。

    口径是「模块被 import 时会不会执行到这条语句」，不是「它是不是 ``tree.body`` 的
    直接子节点」：

    - **下探**：``try:`` / ``try/except*``（body、每个 handler、orelse、finalbody）、
      ``if``（body、orelse，含 ``if TYPE_CHECKING:``）、``with``、``match`` 各分支体、
      ``while`` / ``for`` 的循环体，以及**类体**——类体也在模块 import 时执行，
      ``class C: import lerobot`` 一样会把 lerobot 拉进主环境。
    - **停手**：``def`` / ``async def`` 的函数体。函数体是惰性执行域，
      ``def factory(...): from lerobot... import AlohaMiniClient`` 这类写法合法，正是
      示例注入真 client 的形态。

    实现走「遇到 import 就收下、遇到函数定义就整棵剪掉、其余节点继续下探」，所以上面
    列出的分支形态不必逐个枚举也不会漏。

    Args:
        tree: 待检查的 AST 模块根节点。

    Returns:
        import 期作用域上的 ``ast.Import`` / ``ast.ImportFrom`` 节点列表。
    """
    found: list[ast.Import | ast.ImportFrom] = []
    stack: list[ast.AST] = list(tree.body)
    while stack:
        node = stack.pop()
        if isinstance(node, ast.FunctionDef | ast.AsyncFunctionDef):
            continue
        if isinstance(node, ast.Import | ast.ImportFrom):
            found.append(node)
            continue
        stack.extend(ast.iter_child_nodes(node))

    return found


def _module_scope_lerobot_imports(tree: ast.Module) -> list[str]:
    """收集模块 import 期作用域里 import 到的 lerobot 模块名。

    作用域口径见 :func:`_module_scope_import_nodes`（下探 ``try:`` / ``if TYPE_CHECKING:``
    / ``with`` / ``match`` / 类体，在函数定义处停手）。返回值去重且排序，便于断言。

    Args:
        tree: 待检查的 AST 模块根节点。

    Returns:
        import 期作用域上 import 到的 lerobot 模块名列表；没有违规时为空列表。
    """
    names: set[str] = set()
    for node in _module_scope_import_nodes(tree):
        names.update(_imported_lerobot_names(node))

    return sorted(names)


def _function_scope_lerobot_imports(tree: ast.Module) -> list[str]:
    """收集**函数体内**出现的 lerobot 导入模块名（证明厂商 client 是惰性导入的）。

    用 ``ast.walk`` 遍历整棵树，但只收落在 ``FunctionDef`` / ``AsyncFunctionDef`` 子孙里的
    导入节点；模块顶层的导入不算。返回值去重且排序。

    Args:
        tree: 待检查的 AST 模块根节点。

    Returns:
        函数体内 import 到的 lerobot 模块名列表；没有时为空列表。
    """
    names: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef | ast.AsyncFunctionDef):
            continue
        for inner in ast.walk(node):
            if isinstance(inner, ast.Import | ast.ImportFrom):
                names.update(_imported_lerobot_names(inner))

    return sorted(names)


def _call_name(call: ast.Call) -> str | None:
    """给出一个调用节点的被调函数名（``name(...)`` 与 ``obj.name(...)`` 两种形态）。

    Args:
        call: 待检查的调用节点。

    Returns:
        被调函数名；不是简单名/属性形态（例如 ``f()()``）时返回 ``None``。
    """
    func = call.func
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr

    return None


def _keyword_value(call: ast.Call, name: str) -> ast.expr | None:
    """取出调用节点里某个关键字参数的取值表达式。

    Args:
        call: 待检查的调用节点。
        name: 关键字参数名。

    Returns:
        对应的取值表达式；该关键字参数缺席时返回 ``None``。``**kwargs`` 展开（``arg``
        为 ``None``）不参与匹配。
    """
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value

    return None


def _calls_named(tree: ast.AST, name: str) -> list[ast.Call]:
    """收集 AST 里所有被调函数名为 ``name`` 的调用节点。

    Args:
        tree: 待扫描的 AST 节点（通常是模块根节点）。
        name: 被调函数名，例如 ``"connect"``。

    Returns:
        命中的调用节点列表（按遍历顺序）。
    """
    return [
        node for node in ast.walk(tree) if isinstance(node, ast.Call) and _call_name(node) == name
    ]


def _keyword_flags(calls: list[ast.Call], name: str) -> list[object]:
    """取出若干调用节点里某个关键字参数的字面量取值，便于做「有/没有 True」断言。

    Args:
        calls: 待检查的调用节点列表。
        name: 关键字参数名。

    Returns:
        每个调用上该关键字参数的常量取值；缺席或不是常量的位置记为空字符串
        ``"<非字面量>"`` / ``"<缺>"``。
    """
    flags: list[object] = []
    for call in calls:
        value = _keyword_value(call, name)
        if value is None:
            flags.append("<缺>")
        elif isinstance(value, ast.Constant):
            flags.append(value.value)
        else:
            flags.append("<非字面量>")

    return flags


def _is_counter_ref(node: ast.AST) -> bool:
    """判断节点是否为 ``<x>.send_action_calls`` 这样的属性访问。

    Args:
        node: 待判定的 AST 节点。

    Returns:
        是计数器属性访问时为 ``True``。
    """
    return isinstance(node, ast.Attribute) and node.attr == SEND_ACTION_COUNTER_ATTR


def _argument_interpolates_counter(argument: ast.expr) -> bool:
    """判断一个 ``print`` 参数里是否插值了 ``send_action_calls``。

    Args:
        argument: ``print`` 的一个位置参数表达式。

    Returns:
        参数某个 f-string 插值（``ast.FormattedValue``）里出现该属性访问时为 ``True``。
    """
    return any(
        isinstance(node, ast.FormattedValue)
        and any(_is_counter_ref(inner) for inner in ast.walk(node))
        for node in ast.walk(argument)
    )


def _readout_print_calls(tree: ast.Module) -> list[ast.Call]:
    """收集所有插值了 ``send_action_calls`` 的 ``print`` 调用节点。

    Args:
        tree: 待扫描的 AST 模块根节点。

    Returns:
        命中该形态的 ``print`` 调用节点列表。
    """
    return [
        call
        for call in _calls_named(tree, "print")
        if any(_argument_interpolates_counter(argument) for argument in call.args)
    ]


def _readout_print_statement(tree: ast.Module) -> ast.Expr:
    """取出「下发次数读数行」那条 ``print`` 语句节点（用于真值自证里单独执行它）。

    Args:
        tree: 待扫描的 AST 模块根节点。

    Returns:
        包住读数 ``print`` 调用的 ``ast.Expr`` 语句节点。

    Raises:
        AssertionError: 找不到插值 ``send_action_calls`` 的 ``print`` 语句时。
    """
    call_ids = {id(call) for call in _readout_print_calls(tree)}
    for node in ast.walk(tree):
        if isinstance(node, ast.Expr) and id(node.value) in call_ids:
            return node

    raise AssertionError(
        "示例里找不到插值 send_action_calls 的 print 语句（读数行必须由真计数插值产生）: "
        f"{EXAMPLE_PATH}"
    )


def _is_send_action_zero_assert(node: ast.AST) -> bool:
    """判断节点是否为 ``assert <x>.send_action_calls == 0`` 形态的真断言。

    只认这一种形态：``test`` 是 ``ast.Compare``，``left`` 是 ``send_action_calls`` 属性
    访问，``ops`` 含 ``ast.Eq``，比较对象里有整数常量 ``0``（``False`` 这类 bool 常量不算）。
    ``hits = t.send_action_calls; assert hits == 0``、``assert not t.send_action_calls``
    都不被接受——留下它们是给「读到变量就叫断言」开口子。

    Args:
        node: 待判定的 AST 节点。

    Returns:
        命中该断言形态时为 ``True``。
    """
    if not isinstance(node, ast.Assert):
        return False

    test = node.test
    if not isinstance(test, ast.Compare):
        return False
    if not _is_counter_ref(test.left):
        return False
    if not any(isinstance(op, ast.Eq) for op in test.ops):
        return False

    return any(
        isinstance(comparator, ast.Constant)
        and not isinstance(comparator.value, bool)
        and isinstance(comparator.value, int)
        and comparator.value == 0
        for comparator in test.comparators
    )


def _is_main_guard(node: ast.stmt) -> bool:
    """判断模块顶层语句是否为 ``if __name__ == "__main__":`` 收口。

    Args:
        node: 模块顶层的一条语句。

    Returns:
        是 ``__main__`` 收口时为 ``True``。
    """
    if not isinstance(node, ast.If):
        return False

    test = node.test
    if not isinstance(test, ast.Compare) or len(test.comparators) != 1:
        return False
    if not (isinstance(test.left, ast.Name) and test.left.id == "__name__"):
        return False
    comparator = test.comparators[0]

    return isinstance(comparator, ast.Constant) and comparator.value == "__main__"


def _describe(proc: subprocess.CompletedProcess[str]) -> str:
    """把一次子进程运行拼成便于定位的失败信息。

    Args:
        proc: 子进程运行结果。

    Returns:
        含命令行、退出码与两路输出的多行文本。
    """
    return (
        f"cmd={sys.executable} {EXAMPLE_PATH}\n"
        f"returncode={proc.returncode}\n"
        f"--- stdout ---\n{proc.stdout}\n"
        f"--- stderr ---\n{proc.stderr}"
    )


def _describe_run(run: _RealBranchRun) -> str:
    """把一次进程内真机分支运行拼成便于定位的失败信息。

    Args:
        run: 真机分支运行记录。

    Returns:
        含退出码与 stdout 的多行文本。
    """
    return f'main(["--host", {STUB_HOST!r}]) 返回 {run.returncode}\n--- stdout ---\n{run.stdout}'


def _isolated_env() -> dict[str, str]:
    """返回剥掉 ``PYTHONPATH`` / ``PYTHONHOME`` / ``VIRTUAL_ENV`` 的环境变量副本。

    调用方环境里若 ``PYTHONPATH`` 指向 vendor 的 ``src``（装了 ``lerobot`` 的那份），
    「主环境没有 lerobot」这个前提就不成立，示例会走真机分支去 ``client.connect()``
    连设备，测试就不再离线。其余变量原样保留（解释器启动与 locale 依赖它们）。

    Returns:
        可用于 ``subprocess.run(env=...)`` 的环境变量映射。
    """
    return {name: value for name, value in os.environ.items() if name not in STRIPPED_ENV_VARS}


def _build_client_factory(module: ModuleType) -> object:
    """取出示例模块的 ``build_client_factory``；缺失时给出明确的失败信息。

    Args:
        module: 已加载的示例模块。

    Returns:
        示例模块里的 ``build_client_factory`` 对象（未调用）。
    """
    factory_builder = getattr(module, "build_client_factory", None)
    assert factory_builder is not None, (
        f"示例必须提供名为 build_client_factory 的工厂构造器（惰性 import 的那个），"
        f"在 {EXAMPLE_PATH} 里找不到该属性"
    )

    return factory_builder


def _load_example_module() -> ModuleType:
    """从文件路径加载示例脚本为模块（不执行 ``main()``）。

    不往 ``sys.path`` 里塞 ``examples/``（避免污染）。示例已用
    ``if __name__ == "__main__":`` 收口，因此加载本身不会执行 ``main()``，也不会碰设备。

    Returns:
        已执行模块级代码、但未调用 ``main()`` 的示例模块对象。
    """
    assert EXAMPLE_PATH.is_file(), f"示例脚本尚未创建: {EXAMPLE_PATH}"

    spec = importlib.util.spec_from_file_location("_example_robot_dry_run", EXAMPLE_PATH)
    assert spec is not None, f"无法为示例脚本构建模块 spec: {EXAMPLE_PATH}"
    loader = spec.loader
    assert loader is not None, f"示例脚本的模块 spec 没有 loader: {EXAMPLE_PATH}"

    module = importlib.util.module_from_spec(spec)
    # 加载即执行模块级代码，会往 examples/ 下写 __pycache__；临时关掉字节码落盘。
    previous_dont_write_bytecode = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous_dont_write_bytecode

    return module


def _make_counting_transport(module: ModuleType) -> tuple[Any, _StubAlohaMiniClient]:
    """按示例自己的工厂链造一对「计数代理 + 假 client」。

    走的是被测代码的真实路径：``build_client_factory()(host)`` 造 client，
    ``_CountingTransport(client)`` 包一层，不另写替身。

    Args:
        module: 已加载的示例模块（假 ``lerobot`` 已就位）。

    Returns:
        ``(proxy, client)``：示例的计数代理与它包住的假厂商 client。
    """
    factory = _build_client_factory(module)()
    client = factory(STUB_HOST)

    # 直接取示例里的私有类：这是被测代码本体，不能在测试里另写一份替身。
    return module._CountingTransport(client), client


# ---------------------------------------------------------------------------
# 假 lerobot 包与假厂商 client（离线真机分支用）
# ---------------------------------------------------------------------------


@dataclasses.dataclass
class _StubClientConfig:
    """假厂商 config：收下示例会传的三个字段，不做任何校验。

    Attributes:
        remote_ip: 机器人 IP/主机名，示例从 ``--host`` 传进来。
        robot_model: 型号。
        observation_request_window: 观测请求窗口深度。
    """

    remote_ip: str
    robot_model: str
    observation_request_window: int


class _StubAlohaMiniClient:
    """假厂商 client：内存版传输面，逐方法记调用次数，不连设备、不联网。

    与真 ``AlohaMiniClient`` 的关键差异是**有意**的、也是契约 C 强制的：这里不提供
    ``_control_status`` 观测字段与 ``latest_control_status`` 旁路属性，所以会话拿不到
    显式 ``settled``/``valid`` 声明，命令在 priming 阶段的 adapter 门禁就被拒——真机上
    同样是这个结局（详见示例模块 docstring 第 3 节）。
    """

    def __init__(self, config: _StubClientConfig) -> None:
        """记录 config 并清零所有计数。

        Args:
            config: 假厂商 config。
        """
        self.config = config
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.send_action_calls = 0
        self.get_observation_calls = 0
        self.prime_calls = 0
        self.sent_actions: list[dict[str, float]] = []
        self.prime_include_cameras: list[bool] = []
        self.get_observation_kwargs: list[dict[str, Any]] = []
        self.observation_sequence = 0

    def connect(self) -> None:
        """假连接：只累加计数。"""
        self.connect_calls += 1

    def disconnect(self) -> None:
        """假断开：只累加计数。"""
        self.disconnect_calls += 1

    def get_observation(self, **kwargs: Any) -> dict[str, Any]:
        """返回一帧常量观测，并把样本序号与读数计数各推进一。

        Args:
            **kwargs: 原样记下（示例会传 ``include_cameras=False``）。

        Returns:
            :data:`STUB_OBSERVATION` 的浅拷贝。
        """
        self.get_observation_calls += 1
        self.observation_sequence += 1
        self.get_observation_kwargs.append(dict(kwargs))

        return dict(STUB_OBSERVATION)

    def send_action(self, action: Mapping[str, float]) -> dict[str, Any]:
        """记一次下发（独立于被测代码计数代理的第二重计数器），并把动作原样回显。

        Args:
            action: 归一化单位的动作映射。

        Returns:
            与入参内容相同的动作映射。
        """
        self.send_action_calls += 1
        self.sent_actions.append(dict(action))

        return dict(action)

    def prime_observation_request_window(self, *, include_cameras: bool) -> None:
        """记一次预热请求与其参数。

        Args:
            include_cameras: 是否连相机一起预热。
        """
        self.prime_calls += 1
        self.prime_include_cameras.append(include_cameras)


def _install_fake_lerobot(monkeypatch: pytest.MonkeyPatch) -> list[_StubAlohaMiniClient]:
    """往 ``sys.modules`` 塞一个最小假 ``lerobot`` 包，返回被造出来的 client 实例列表。

    注入用 ``monkeypatch.setitem``：原条目（例如真装了 lerobot 的环境）在 teardown 自动
    恢复，没有原条目则被删除，因此不会污染 ``tests/test_capability.py`` 的导入边界守卫与
    两个 parity 文件的收集期跳过判定。

    Args:
        monkeypatch: pytest 的 monkeypatch 夹具，负责 teardown 恢复。

    Returns:
        被假 client 构造器记下的实例列表（每个实例自带 ``send_action`` 等计数器）。
    """
    instances: list[_StubAlohaMiniClient] = []

    class _RegisteredClient(_StubAlohaMiniClient):
        def __init__(self, config: _StubClientConfig) -> None:
            super().__init__(config)
            instances.append(self)

    leaf_names = (
        "lerobot.robots.alohamini.alohamini_client",
        "lerobot.robots.alohamini.config_alohamini",
    )
    module_names = ("lerobot", "lerobot.robots", "lerobot.robots.alohamini", *leaf_names)
    modules: dict[str, ModuleType] = {}
    for name in module_names:
        module = ModuleType(name)
        if name not in leaf_names:
            module.__path__ = []  # type: ignore[attr-defined]
        modules[name] = module

    modules[leaf_names[0]].AlohaMiniClient = _RegisteredClient
    modules[leaf_names[1]].AlohaMiniClientConfig = _StubClientConfig
    modules["lerobot"].robots = modules["lerobot.robots"]
    modules["lerobot.robots"].alohamini = modules["lerobot.robots.alohamini"]
    modules["lerobot.robots.alohamini"].alohamini_client = modules[leaf_names[0]]
    modules["lerobot.robots.alohamini"].config_alohamini = modules[leaf_names[1]]

    for name in module_names:
        monkeypatch.setitem(sys.modules, name, modules[name])

    return instances


@dataclasses.dataclass(frozen=True)
class _RealBranchRun:
    """一次进程内真机分支运行的结果。

    Attributes:
        returncode: ``main()`` 的返回值。
        stdout: 这段时间里 ``main()`` 写到 stdout 的全部文本。
        clients: 假 client 构造器记下的实例（真机上就是 ``AlohaMiniClient``）。
    """

    returncode: int
    stdout: str
    clients: list[_StubAlohaMiniClient]


# ---------------------------------------------------------------------------
# fixture：子进程输出、源码文本、AST、已加载模块、真机分支运行
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def example_run() -> subprocess.CompletedProcess[str]:
    """在仓库根下用当前解释器跑一次示例脚本，A 组断言复用同一份输出。

    主环境没有 ``lerobot``，因此走的是示例的跳过分支。子进程带 :func:`_isolated_env`
    剥过环境变量，保证「主环境」这个前提不被调用方 ``PYTHONPATH`` 改写。

    Returns:
        子进程运行结果。
    """
    assert EXAMPLE_PATH.is_file(), f"示例脚本尚未创建: {EXAMPLE_PATH}"

    return subprocess.run(
        [sys.executable, str(EXAMPLE_PATH)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=SUBPROCESS_TIMEOUT_S,
        check=False,
        env=_isolated_env(),
    )


@pytest.fixture(scope="module")
def example_source() -> str:
    """读取示例源码文本；文件缺失时给出明确信息。

    Returns:
        示例源码全文。
    """
    assert EXAMPLE_PATH.is_file(), f"示例脚本尚未创建: {EXAMPLE_PATH}"

    return EXAMPLE_PATH.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def example_tree(example_source: str) -> ast.Module:
    """把示例源码解析成 AST，供 B/D 组静态守卫复用。

    Args:
        example_source: 示例源码全文。

    Returns:
        解析后的 AST 模块根节点。
    """
    return ast.parse(example_source, filename=str(EXAMPLE_PATH))


@pytest.fixture
def example_module() -> ModuleType:
    """从文件路径加载一份新的示例模块，供行为级守卫使用。

    function 作用域：有的用例会 ``monkeypatch`` 模块级函数，独立实例避免相互串味。

    Returns:
        已执行模块级代码、但未调用 ``main()`` 的示例模块对象。
    """
    return _load_example_module()


@pytest.fixture
def fake_lerobot(monkeypatch: pytest.MonkeyPatch) -> list[_StubAlohaMiniClient]:
    """把假 ``lerobot`` 包注入 ``sys.modules``（teardown 由 monkeypatch 恢复）。

    Args:
        monkeypatch: pytest 的 monkeypatch 夹具。

    Returns:
        假 client 的实例列表（空列表表示构造器还没被调用过）。
    """
    return _install_fake_lerobot(monkeypatch)


@pytest.fixture
def real_branch_run(fake_lerobot: list[_StubAlohaMiniClient]) -> _RealBranchRun:
    """在同一进程里用假 ``lerobot`` 跑一遍 ``main()`` 的真机分支。

    全程内存：假 client 不碰 socket、不开线程，命令在 priming 阶段就被 adapter 门禁拒绝，
    所以不会 sleep 等网络，也不会真的下发。

    Args:
        fake_lerobot: 假 client 实例列表（真机分支的断言要用它们的计数器）。

    Returns:
        退出码、stdout 与假 client 实例列表。
    """
    module = _load_example_module()
    stream = io.StringIO()
    with contextlib.redirect_stdout(stream):
        returncode = module.main(["--host", STUB_HOST])

    return _RealBranchRun(returncode=returncode, stdout=stream.getvalue(), clients=fake_lerobot)


# ---------------------------------------------------------------------------
# 契约 A：主环境（无 lerobot）下的子进程行为
# ---------------------------------------------------------------------------


def test_example_script_exists() -> None:
    """契约 A：示例脚本文件存在。"""
    assert EXAMPLE_PATH.is_file(), f"示例脚本尚未创建: {EXAMPLE_PATH}"


def test_example_script_exits_zero_without_lerobot(
    example_run: subprocess.CompletedProcess[str],
) -> None:
    """契约 A：主环境（无 lerobot）下退出码为 0——优雅跳过，不是崩。"""
    assert example_run.returncode == 0, f"示例脚本退出码非 0\n{_describe(example_run)}"


def test_example_script_prints_skip_notice_naming_vendor_environment(
    example_run: subprocess.CompletedProcess[str],
) -> None:
    """契约 A：stdout 含字面量 ``需要 vendor 环境``，说明跳过是因为主环境没有 lerobot。"""
    assert SKIP_NOTICE_LITERAL in example_run.stdout, (
        f"stdout 缺少跳过文案「{SKIP_NOTICE_LITERAL}」\n{_describe(example_run)}"
    )


def test_example_script_skip_notice_says_how_to_run(
    example_run: subprocess.CompletedProcess[str],
) -> None:
    """契约 A：跳过文案给出跑法（含 ``PYTHONPATH``），不是只说一句「跳过」。"""
    assert HOWTO_RUN_LITERAL in example_run.stdout, (
        f"stdout 缺少跑法提示「{HOWTO_RUN_LITERAL}」：跳过分支要说明怎么在 vendor 环境里跑\n"
        f"{_describe(example_run)}"
    )


def test_example_script_stderr_has_no_traceback(
    example_run: subprocess.CompletedProcess[str],
) -> None:
    """契约 A：主环境跳过时 stderr 不含 ``Traceback``（没有未捕获异常漏到终端）。"""
    assert "Traceback" not in example_run.stderr, (
        f"主环境下的跳过分支不应抛未捕获异常\n{_describe(example_run)}"
    )


def test_example_subprocess_runs_with_isolated_environment() -> None:
    """契约 A：子进程的环境确实剥掉了能让它误走真机分支的三个变量。

    这条守卫钉的是「离线」前提本身：夹具若把 ``env=_isolated_env()`` 去掉（回退成继承
    父进程环境），本用例会在 ``PYTHONPATH`` 指向 vendor ``src`` 的机器上先红。
    """
    env = _isolated_env()

    leaked = [name for name in STRIPPED_ENV_VARS if name in env]

    assert leaked == [], f"隔离后的子进程环境里仍然留着这些变量: {leaked}"


# ---------------------------------------------------------------------------
# 契约 B：AST 级守卫（import 期作用域、惰性导入、dry_run=True）
# ---------------------------------------------------------------------------


def test_example_module_scope_does_not_import_lerobot(example_tree: ast.Module) -> None:
    """契约 B：模块 import 期作用域不 import ``lerobot``（函数体内的惰性 import 合法）。

    ``import 期作用域``包含 ``try:`` / ``if TYPE_CHECKING:`` / ``with`` / ``match`` 的分支体
    与类体——这些都随模块 import 一起执行；只在函数定义处停手。守卫本身的自测（含
    ``try:``、``TYPE_CHECKING``、handler 体、类体、函数体放行）在
    ``tests/test_examples_and_configs.py`` 的参数化用例里。
    """
    offenders = _module_scope_lerobot_imports(example_tree)

    assert offenders == [], (
        f"示例的模块 import 期作用域不得 import lerobot（本库在无 lerobot 的主环境里也要能"
        f"加载本脚本），实际导入: {offenders}。请把厂商 import 移到最内层工厂函数体内"
    )


def test_example_imports_vendor_client_inside_a_function(example_tree: ast.Module) -> None:
    """契约 B：至少一处 ``from lerobot...`` 落在函数体内（惰性导入，构造器可被主环境加载）。"""
    imports = _function_scope_lerobot_imports(example_tree)

    assert imports, (
        "示例必须在函数体内 import 厂商 client（例如 build_client_factory 返回的最内层工厂里"
        "`from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient`），"
        f"当前函数体内没有任何 lerobot 导入：{EXAMPLE_PATH}"
    )


def test_example_calls_connect_with_dry_run_true(example_tree: ast.Module) -> None:
    """契约 B：存在 ``connect(...)`` 调用且其关键字 ``dry_run`` 为常量 ``True``。"""
    connect_calls = _calls_named(example_tree, "connect")
    assert connect_calls, f"示例里找不到名为 connect 的调用: {EXAMPLE_PATH}"

    dry_run_true = [
        call
        for call in connect_calls
        if (value := _keyword_value(call, "dry_run")) is not None
        and isinstance(value, ast.Constant)
        and value.value is True
    ]

    assert dry_run_true, (
        "示例里的 connect(...) 必须显式带关键字 dry_run=True（真机形态零发送的前提）；"
        f"当前 connect 调用的 dry_run 取值: {_keyword_flags(connect_calls, 'dry_run')}"
    )


def test_example_has_no_connect_call_with_dry_run_false(example_tree: ast.Module) -> None:
    """契约 B：不存在 ``dry_run=False`` 的 ``connect`` 调用（示例不得具备真实运动分支）。"""
    offenders = [
        ast.unparse(call)
        for call in _calls_named(example_tree, "connect")
        if (value := _keyword_value(call, "dry_run")) is not None
        and isinstance(value, ast.Constant)
        and value.value is False
    ]

    assert offenders == [], (
        f"示例不得包含 dry_run=False 的 connect 调用（零发送是这份脚本的全部价值）: {offenders}"
    )


def test_example_has_main_guard(example_tree: ast.Module) -> None:
    """契约 B：脚本用 ``if __name__ == "__main__": main()`` 收口，加载模块不会执行 main。"""
    guards = [node for node in example_tree.body if _is_main_guard(node)]

    assert guards, (
        f'示例必须用 `if __name__ == "__main__":` 收口（否则按路径加载模块时会直接跑主流程），'
        f"未在 {EXAMPLE_PATH} 的模块顶层找到该分支"
    )

    main_calls = [call for guard in guards for call in _calls_named(guard, "main")]
    assert main_calls, (
        '`if __name__ == "__main__":` 分支里必须调用 main()（main 是脚本入口，'
        f"返回 None 或 int 都可）: {EXAMPLE_PATH}"
    )


# ---------------------------------------------------------------------------
# 契约 C：真机分支的离线覆盖（假 lerobot 注入 sys.modules）
# ---------------------------------------------------------------------------


def test_example_real_branch_exits_zero(real_branch_run: _RealBranchRun) -> None:
    """契约 C：装了（假）lerobot 时真机分支跑通并以退 0 收尾。"""
    assert real_branch_run.returncode == 0, (
        f"真机分支的 main() 返回值非 0\n{_describe_run(real_branch_run)}"
    )


def test_example_real_branch_readout_line_reports_zero_sends(
    real_branch_run: _RealBranchRun,
) -> None:
    """契约 C/D：读数行正则取数等于 0——完整读数行由真计数插值产生，不是源码常量。"""
    match = READOUT_RE.search(real_branch_run.stdout)
    assert match is not None, (
        f"stdout 里找不到读数行「send_action 调用次数: N 次发送」\n{_describe_run(real_branch_run)}"
    )

    assert int(match.group(1)) == 0, (
        f"读数行的下发次数是 {match.group(1)}，dry_run=True 下必须为 0\n"
        f"{_describe_run(real_branch_run)}"
    )


def test_example_real_branch_reads_a_real_observation(real_branch_run: _RealBranchRun) -> None:
    """契约 C：观测读数行出现，键数对得上——会话真的读过一帧观测，不是空跑。"""
    sequence_match = OBSERVATION_SEQUENCE_RE.search(real_branch_run.stdout)
    assert sequence_match is not None, (
        f"stdout 里找不到观测读数行「样本序号 observation_sequence=N」\n"
        f"{_describe_run(real_branch_run)}"
    )
    assert int(sequence_match.group(1)) >= 1, (
        f"观测样本序号是 {sequence_match.group(1)}，说明没有真的取到帧\n"
        f"{_describe_run(real_branch_run)}"
    )

    keys_match = OBSERVATION_KEY_COUNT_RE.search(real_branch_run.stdout)
    assert keys_match is not None, (
        f"stdout 里找不到观测键数读数行「读一帧观测，键 N 个」\n{_describe_run(real_branch_run)}"
    )
    assert int(keys_match.group(1)) == len(STUB_OBSERVATION), (
        f"读数行报的键数是 {keys_match.group(1)}，假 client 给的帧有 "
        f"{len(STUB_OBSERVATION)} 个键——说明打印的不是这一帧\n{_describe_run(real_branch_run)}"
    )

    clients = real_branch_run.clients
    assert clients, (
        f"假 client 一次都没被构造出来，真机分支根本没走\n{_describe_run(real_branch_run)}"
    )
    assert clients[0].get_observation_calls >= 1, (
        f"假 client 的 get_observation 调用次数是 {clients[0].get_observation_calls}，"
        f"要求 >= 1\n{_describe_run(real_branch_run)}"
    )


def test_example_real_branch_prints_lift_command_rejected(real_branch_run: _RealBranchRun) -> None:
    """契约 C：命令步结局被如实打印为「被拒 / MotionNotEnabled」，不是静默吞掉。"""
    stdout = real_branch_run.stdout

    assert "被拒" in stdout, (
        f"stdout 里没有「被拒」：无 adapter 时命令必须在 priming 阶段被拒并说清楚\n"
        f"{_describe_run(real_branch_run)}"
    )
    assert "MotionNotEnabled" in stdout, (
        f"stdout 里没有「MotionNotEnabled」：拒绝原因必须点名控制层异常类型\n"
        f"{_describe_run(real_branch_run)}"
    )


def test_example_real_branch_client_received_no_send_action(
    real_branch_run: _RealBranchRun,
) -> None:
    """契约 C：假 client 自己的 ``send_action`` 计数为 0——独立于被测代码计数代理的证据。"""
    clients = real_branch_run.clients
    assert clients, (
        f"假 client 一次都没被构造出来，真机分支根本没走\n{_describe_run(real_branch_run)}"
    )

    offenders = [
        {"send_action_calls": client.send_action_calls, "sent_actions": client.sent_actions}
        for client in clients
        if client.send_action_calls != 0
    ]

    assert offenders == [], (
        f"接在计数代理后面的假 client 收到了下发（第二重独立证据要求 0 次）: {offenders}\n"
        f"{_describe_run(real_branch_run)}"
    )


# ---------------------------------------------------------------------------
# 契约 D：防「硬编码 0 次发送」假绿（运行时读数 + 反例自证 + AST 插值守卫）
# ---------------------------------------------------------------------------


def test_example_readout_print_interpolates_send_action_counter(example_tree: ast.Module) -> None:
    """契约 D：读数行的 f-string 插值里必须有 ``send_action_calls``（防硬编码字面量）。

    只看 ``print`` 调用的参数表达式树：``FormattedValue`` 里能找到
    ``<x>.send_action_calls`` 属性访问才算数。把插值换成常量 ``0``（或整条改成普通字符串）
    这条用例就红——这正是旧源码常量守卫做不到的。
    """
    calls = _readout_print_calls(example_tree)

    assert calls, (
        f"示例结尾那条 print 必须插值 send_action_calls（形如 "
        f'`print(f"send_action 调用次数: {{transport.send_action_calls}} 次发送（...）")`），'
        f"不能把数字写成常量。当前 print 调用数: {len(_calls_named(example_tree, 'print'))}，"
        f"其中插值了该属性的: 0"
    )


def test_example_readout_reports_one_when_proxy_really_sends_once(
    example_module: ModuleType,
    fake_lerobot: list[_StubAlohaMiniClient],
    example_tree: ast.Module,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """契约 D 反例自证：计数代理真被下发一次后，同一条读数行必须变成 ``1 次发送``。

    为什么单独执行那条 ``print`` 语句：``main()`` 里的 ``assert ... == 0`` 是硬门禁，
    计数不为 0 时它在打印之前就抛错（下一条用例正是钉这个行为），所以「计数为 1 的读数行」
    只能由被钉住的那条源码语句本身跑出来。这里取的是示例源码的原文语句（``ast.unparse``
    后执行），参数是**被测代码自己的**计数代理实例，计数由一次真实 ``send_action`` 调用
    产生——不是在这里拼一个假字符串，因此能证明数字来自真计数。
    """
    proxy, client = _make_counting_transport(example_module)

    proxy.send_action({"lift_axis.height_mm": 1.0})

    assert proxy.send_action_calls == 1, (
        f"计数代理在下发一次后应记 1，实际 {proxy.send_action_calls}（代理没在真计数）"
    )
    assert client.send_action_calls == 1, (
        f"计数代理应把这次下发转发给 client，client 计数为 {client.send_action_calls}"
    )

    readout_source = ast.unparse(_readout_print_statement(example_tree))
    exec(compile(readout_source, str(EXAMPLE_PATH), "exec"), {"transport": proxy})  # noqa: S102

    stdout = capsys.readouterr().out
    match = READOUT_RE.search(stdout)
    assert match is not None, f"执行读数行没有产出「send_action 调用次数: N 次发送」: {stdout!r}"
    assert int(match.group(1)) == 1, (
        f"计数为 1 时读数行报的是 {match.group(1)} 次发送：数字不是来自真计数"
        f"（源码语句: {readout_source}）"
    )


def test_example_main_fails_closed_when_the_proxy_sends_once(
    example_module: ModuleType,
    fake_lerobot: list[_StubAlohaMiniClient],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """契约 D 反例自证：计数为 1 时 ``main()`` 以断言失败收场，且消息里报的是具体数字 1。

    把命令步换成「通过计数代理真的下发一次」，跑完整 ``main()``：读数前的
    ``assert transport.send_action_calls == 0`` 必须拦下来，消息里报 ``实际调用 1 次``。
    这条用例钉两件事：计数确实来自真下发；零发送是硬门禁，不是仅靠打印。
    """

    def _send_one_lift_action(session: Any) -> None:
        """绕过命令门禁，直接经会话的计数代理下发一条升降轴动作。

        Args:
            session: ``connect(dry_run=True)`` 打开的会话（``client`` 就是计数代理）。
        """
        session.client.send_action({"lift_axis.height_mm": session.lift.height()})

    monkeypatch.setattr(example_module, "_run_dry_run_lift_command", _send_one_lift_action)

    with pytest.raises(AssertionError, match="实际调用 1 次"):
        example_module.main(["--host", STUB_HOST])

    assert fake_lerobot, "假 client 一次都没被构造出来，这条反例没有真的跑到下发路径"


# ---------------------------------------------------------------------------
# 契约 E：协议守卫（结构 + 三个方法的实际行为）
# ---------------------------------------------------------------------------


class _BareForwarder:
    """只有 ``__getattr__`` 的转发壳：用来证明协议检查不是恒真。"""

    def __getattr__(self, name: str) -> Any:
        """把任意属性名转发给一个空对象（必然 ``AttributeError``）。

        Args:
            name: 属性名。

        Returns:
            永不返回（``object`` 上没有该属性）。

        Raises:
            AttributeError: 恒抛。
        """
        return getattr(object(), name)


def test_example_counting_transport_satisfies_robot_transport(
    example_module: ModuleType, fake_lerobot: list[_StubAlohaMiniClient]
) -> None:
    """契约 E：计数代理 ``isinstance(..., RobotTransport)`` 为真，且该检查不是恒真。

    ``runtime_checkable`` 协议只检查方法**是否存在**（不看行为）。这里一并钉住两点：
    纯 ``__getattr__`` 转发壳**不**通过检查（CPython 的实例检查对协议方法用
    ``getattr_static``，``__getattr__`` 蒙不过去），所以下面的正例不是空检查。
    """
    proxy, _ = _make_counting_transport(example_module)

    assert isinstance(proxy, RobotTransport), (
        f"示例的计数代理必须满足 RobotTransport 协议，实际类型 {type(proxy)}"
    )
    assert not isinstance(_BareForwarder(), RobotTransport), (
        "只有 __getattr__ 的转发壳不该通过协议检查——它通过了说明这条守卫退化成恒真"
    )


def test_example_counting_transport_forwards_and_counts_for_real(
    example_module: ModuleType, fake_lerobot: list[_StubAlohaMiniClient]
) -> None:
    """契约 E：三个关键方法的**行为**都要对——协议通过不能替代行为验证。"""
    proxy, client = _make_counting_transport(example_module)

    # send_action：先自增，再转发给 client。
    proxy.send_action({"lift_axis.height_mm": 7.5})
    assert proxy.send_action_calls == 1, (
        f"send_action 调用一次后计数应为 1，实际 {proxy.send_action_calls}"
    )
    assert client.sent_actions == [{"lift_axis.height_mm": 7.5}], (
        f"send_action 必须把动作原样转发给 client，client 实收 {client.sent_actions}"
    )

    # get_observation：返回 Mapping。
    observation = proxy.get_observation(include_cameras=False)
    assert isinstance(observation, Mapping), (
        f"get_observation 必须返回 Mapping，实际类型 {type(observation)}"
    )

    # prime_observation_request_window：关键字参数要转发到 client。
    proxy.prime_observation_request_window(include_cameras=False)
    assert client.prime_calls == 1, (
        f"prime_observation_request_window 应转发到 client（client 计数应为 1，实际 "
        f"{client.prime_calls}）"
    )
    assert client.prime_include_cameras == [False], (
        f"include_cameras 关键字没有转发到 client，client 实收 {client.prime_include_cameras}"
    )


# ---------------------------------------------------------------------------
# 契约 C（静态侧）：轻行为守卫，不 require lerobot
# ---------------------------------------------------------------------------


def test_example_exposes_callable_build_client_factory(example_module: ModuleType) -> None:
    """构造器守卫：示例模块提供可调用的 ``build_client_factory``（无 lerobot 时也能 import 到）。"""
    factory_builder = _build_client_factory(example_module)

    assert callable(factory_builder), (
        f"build_client_factory 必须可调用（它返回注入给 connect 的工厂）: 实际类型 "
        f"{type(factory_builder)}"
    )


def test_example_build_client_factory_returns_callable_factory(example_module: ModuleType) -> None:
    """构造器守卫：无参调用 ``build_client_factory()`` 返回一个可调用对象。

    返回对象的形状约定为 ``(host, **kwargs) -> client``。

    只断言 ``callable(...)``，**不调用返回的工厂**：这里是主环境，没有 ``lerobot``，真调用会
    抛 ``ImportError``。构造器本身必须是惰性的，否则这条用例会以 ImportError 变红。
    """
    factory_builder = _build_client_factory(example_module)

    factory = factory_builder()

    assert callable(factory), (
        f"build_client_factory() 必须返回可调用的工厂（形如 (host, **kwargs) -> client），"
        f"实际返回 {type(factory)}"
    )
