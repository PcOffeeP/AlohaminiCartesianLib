"""``examples/robot_dry_run.py`` 的验收契约（TDD 红阶段）。

本文件在 ``coder`` 动手之前先落下契约：示例脚本还不存在，因此本文件预期全红（子进程
拉不到脚本、从路径加载模块找不到 ``build_client_factory``）。

这个示例要补的缺口是「真 client 注入」：今天所有实机运动证据都来自
``benchmarks/alohamini_io/measure_latency.py``，它不 import 本库任何模块；
``src/alohamini_cartesian/control/api.py`` 至今零实机证据。示例的职责是给出这条
参考路径——用厂商 ``AlohaMiniClient`` 走本库的 ``connect(dry_run=True)``，零发送、
只读观测，可在下个远程窗口直接上机。

契约要点（banner 按主题分组）：

- A：主环境（无 ``lerobot``）下子进程跑示例，退出码 0、stdout 出现跳过文案
  ``需要 vendor 环境`` 且该文案给出跑法（含 ``PYTHONPATH``），stderr 不含 ``Traceback``。
  主环境必须优雅跳过，不是崩。
- B：AST 级守卫。模块**顶层**（``tree.body``）不得 import ``lerobot``；厂商 import 必须
  出现在**函数体内**（惰性导入，构造器本身在无 lerobot 的主环境里也能 import 与调用）；
  ``connect(...)`` 调用必须带 ``dry_run=True``，且不得出现 ``dry_run=False`` 的 ``connect``。
- C：静态/轻行为守卫。从路径加载示例模块后 ``build_client_factory`` 存在、可调用，
  调用它返回的对象 ``callable(...)`` 为真。**不调用返回的工厂**——主环境没有 ``lerobot``，
  真调用会抛 ``ImportError``；只断言可调用，保持确定性。
- D：防「硬编码 0 次发送」假绿。``0 次发送`` 的断言不落在主环境 stdout 上（主环境走的是
  跳过分支，不会打印它），而是落在源码级：``print`` 调用参数里有该字面量，并且存在
  ``assert <x>.send_action_calls == 0`` 形态的真断言（与 ``examples/minimal_dry_run.py``
  一致的写法）。

全程离线：不连设备、不联网。外部动作只有拉起一个子进程跑示例脚本，以及从路径加载示例
模块（后者靠示例自己的 ``if __name__ == "__main__":`` 收口保证不会执行 ``main()``）。
"""

from __future__ import annotations

import ast
import importlib.util
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
EXAMPLE_PATH = REPO_ROOT / "examples" / "robot_dry_run.py"

SUBPROCESS_TIMEOUT_S = 60

# 主环境（无 lerobot）跳过分支的两个契约字面量：一句说明「为什么跳过」，一句给出跑法。
SKIP_NOTICE_LITERAL = "需要 vendor 环境"
HOWTO_RUN_LITERAL = "PYTHONPATH"

# 真机路径的打印契约句（只在装了 lerobot 的 vendor 环境里出现）。
ZERO_SEND_LITERAL = "0 次发送"

# 计数器在 AST 里对应的属性名：``<x>.send_action_calls``。
SEND_ACTION_COUNTER_ATTR = "send_action_calls"


# ---------------------------------------------------------------------------
# 通用 helper：AST 与子进程输出
# ---------------------------------------------------------------------------


def _is_lerobot_module(name: str) -> bool:
    """判断模块名是否为 ``lerobot`` 或其子模块。

    Args:
        name: ``import`` 语句里的模块名（``ast.ImportFrom.module`` 或 ``alias.name``）。

    Returns:
        是 ``lerobot`` 体系时为 ``True``。
    """
    return name == "lerobot" or name.startswith("lerobot.")


def _module_scope_lerobot_imports(tree: ast.Module) -> list[str]:
    """收集 AST 模块**顶层**语句里 import 到的 lerobot 模块名。

    只看 ``tree.body`` 的直接子节点，也就是模块作用域上的语句；函数体内的 import 不在
    统计范围（那是允许的惰性导入）。返回值去重且排序，便于断言。

    Args:
        tree: 待检查的 AST 模块根节点。

    Returns:
        顶层 import 到的 lerobot 模块名列表；没有违规时为空列表。
    """
    names: set[str] = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names if _is_lerobot_module(alias.name))
        elif (
            isinstance(node, ast.ImportFrom)
            and node.level == 0
            and node.module is not None
            and _is_lerobot_module(node.module)
        ):
            names.add(node.module)

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
            if isinstance(inner, ast.Import):
                names.update(alias.name for alias in inner.names if _is_lerobot_module(alias.name))
            elif (
                isinstance(inner, ast.ImportFrom)
                and inner.level == 0
                and inner.module is not None
                and _is_lerobot_module(inner.module)
            ):
                names.add(inner.module)

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


def _print_call_string_literals(tree: ast.Module) -> list[str]:
    """收集所有 ``print`` 调用参数里的字符串常量片段。

    普通字符串常量整段收下；f-string 只收其中的常量片段（``JoinedStr`` 的 ``Constant``
    部分），插值部分不看。注释与 docstring 不是 ``print`` 调用的参数，因此不参与匹配。

    Args:
        tree: 待扫描的 AST 模块根节点。

    Returns:
        命中的字符串片段列表。
    """
    literals: list[str] = []
    for call in _calls_named(tree, "print"):
        for argument in call.args:
            for node in ast.walk(argument):
                if isinstance(node, ast.Constant) and isinstance(node.value, str):
                    literals.append(node.value)

    return literals


def _keyword_flags(calls: list[ast.Call], name: str) -> list[object]:
    """取出若干调用节点里某个关键字参数的字面量取值，便于做「有/没有 True」断言。

    Args:
        calls: 待检查的调用节点列表。
        name: 关键字参数名。

    Returns:
        每个调用上该关键字参数的常量取值；缺席或不是常量的位置记为空字符串 ``"<非字面量>"``。
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
    if not (isinstance(test.left, ast.Attribute) and test.left.attr == SEND_ACTION_COUNTER_ATTR):
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


# ---------------------------------------------------------------------------
# fixture：子进程输出、源码文本、AST、已加载模块
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def example_run() -> subprocess.CompletedProcess[str]:
    """在仓库根下用当前解释器跑一次示例脚本，A 组断言复用同一份输出。

    主环境没有 ``lerobot``，因此走的是示例的跳过分支。脚本文件缺失时先给出明确信息，
    避免测试以 ``FileNotFoundError`` 这类脚手架错误变红。

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
    """把示例源码解析成 AST，供 B 组静态守卫复用。

    Args:
        example_source: 示例源码全文。

    Returns:
        解析后的 AST 模块根节点。
    """
    return ast.parse(example_source, filename=str(EXAMPLE_PATH))


@pytest.fixture(scope="module")
def example_module() -> ModuleType:
    """从文件路径加载示例脚本为模块，供 C 组轻行为守卫复用。

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


# ---------------------------------------------------------------------------
# 契约 B：AST 级守卫（顶层 import、惰性导入、dry_run=True）
# ---------------------------------------------------------------------------


def test_example_module_scope_does_not_import_lerobot(example_tree: ast.Module) -> None:
    """契约 B：模块顶层语句不 import ``lerobot``（函数体内的惰性 import 合法）。"""
    offenders = _module_scope_lerobot_imports(example_tree)

    assert offenders == [], (
        f"示例的模块顶层不得 import lerobot（本库在无 lerobot 的主环境里也要能加载本脚本），"
        f"实际顶层导入: {offenders}。请把厂商 import 移到最内层工厂函数体内"
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
# 契约 D：防「硬编码 0 次发送」假绿（源码级）
# ---------------------------------------------------------------------------


def test_example_print_contains_zero_send_literal(example_tree: ast.Module) -> None:
    """契约 D：某个 ``print`` 调用的参数里含字面量 ``0 次发送``（真机路径的打印契约句）。"""
    literals = _print_call_string_literals(example_tree)

    assert any(ZERO_SEND_LITERAL in literal for literal in literals), (
        f"示例必须在 print 的参数里打印字面量「{ZERO_SEND_LITERAL}」"
        f"（真机路径下 stdout 要出现它）；"
        f"当前 print 的字符串片段里没有该字面量——f-string 可用，但「{ZERO_SEND_LITERAL}」这段"
        f"必须是常量片段，不能拼成由变量插值出来的文本。已见片段: {literals}"
    )


def test_example_source_asserts_real_zero_send_action_counter(example_tree: ast.Module) -> None:
    """契约 D：源码里必须有 ``assert <x>.send_action_calls == 0`` 形态的真断言。"""
    asserts = [node for node in ast.walk(example_tree) if _is_send_action_zero_assert(node)]

    assert asserts, (
        "示例必须真的断言 send_action_calls == 0（AST 级检查），不能只打印字面量"
        f"「{ZERO_SEND_LITERAL}」。请在 {EXAMPLE_PATH} 里保留形如 "
        "`assert transport.send_action_calls == 0` 的语句"
        "（`hits = t.send_action_calls; assert hits == 0`、`assert not t.send_action_calls`、"
        "`assert 0 == t.send_action_calls` 都不被接受）"
    )


# ---------------------------------------------------------------------------
# 契约 C：轻行为守卫（不联网、不 require lerobot）
# ---------------------------------------------------------------------------


def test_example_exposes_callable_build_client_factory(example_module: ModuleType) -> None:
    """契约 C：示例模块提供可调用的 ``build_client_factory``（无 lerobot 时也能 import 到）。"""
    factory_builder = _build_client_factory(example_module)

    assert callable(factory_builder), (
        f"build_client_factory 必须可调用（它返回注入给 connect 的工厂）: 实际类型 "
        f"{type(factory_builder)}"
    )


def test_example_build_client_factory_returns_callable_factory(example_module: ModuleType) -> None:
    """契约 C：无参调用 ``build_client_factory()`` 返回一个可调用对象。

    返回对象的形状约定为 ``(host, **kwargs) -> client``。

    只断言 ``callable(...)``，**不调用返回的工厂**：主环境没有 ``lerobot``，真调用会抛
    ``ImportError``。构造器本身必须是惰性的，否则这条用例会以 ImportError 变红。
    """
    factory_builder = _build_client_factory(example_module)

    factory = factory_builder()

    assert callable(factory), (
        f"build_client_factory() 必须返回可调用的工厂（形如 (host, **kwargs) -> client），"
        f"实际返回 {type(factory)}"
    )
