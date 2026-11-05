"""``examples/minimal_dry_run.py`` 与 ``configs/*.yaml`` 的验收契约（TDD 红阶段）。

本文件在 ``coder`` 动手之前先落下契约：示例脚本与两份配置都还不存在，因此本文件预期
全红（子进程拉不到脚本、读文件抛 ``FileNotFoundError``）。

契约要点：

- A：示例脚本能离线跑通，退出码 0；stdout 里出现 ``0 次发送``（证明 ``send_action``
  一次都没被调用）、``steps`` 与 ``residual``。
- B：``configs/template.yaml`` 与 ``configs/site.yaml`` 的键集合等于
  ``overlay.SETTINGS_FIELDS``，取值等于冻结值，且 site 覆盖 template。
- C：``examples/`` 下任何 ``*.py`` 的**模块顶层**都不 import ``lerobot``（函数体内的惰性
  import 是合法的：``examples/robot_dry_run.py`` 就是这样把厂商 client 藏在最内层工厂里，
  以便无 lerobot 的主环境也能加载脚本并打印跳过文案）。本库自身只有
  ``src/alohamini_cartesian/adapter/protocol.py`` 是运行时接触面。
- D：契约 A 的「0 次发送」不能是靠硬编码打印出来的假绿。源码级用 **AST 级**守卫要求
  示例里存在一个 ``ast.Assert`` 节点，被测表达式形如 ``<x>.send_action_calls == 0``
  （注释、docstring、``if False:`` 分支、``or True`` 都不算数），且源码 AST 里
  ``send_action_calls`` 属性访问至少 4 处（初始化 / 自增 / 断言条件 / 断言消息 / 打印，
  字符串与注释不参与计数）；行为级要求把示例模块从路径加载后，
  ``_FakeTransport.send_action_calls`` 会随调用自增（0 → 1 → 2），而不是常量，并真的满足
  ``RobotTransport`` 协议；再加一条**对示例子进程真实输出取数**的读数断言：会话必须真的
  读过观测（``循环共读 N 帧观测``，N >= 1）且报告的下发次数确为 ``0``，否则「dry-run
  不下发」这条根本没被验证到。

全程离线：不连设备、不联网，唯一的外部动作是拉起一个子进程跑示例脚本。读真实配置时
一律传 ``check_devices=False``（本机没有这些 ``/dev`` 节点），并传 ``env={}`` 让断言不
受 ambient 环境变量干扰。
"""

from __future__ import annotations

import ast
import importlib.util
import operator
import re
import subprocess
import sys
from collections.abc import Mapping
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest
import yaml

from alohamini_cartesian.adapter.protocol import RobotTransport
from alohamini_cartesian.config.overlay import (
    INT_FIELDS,
    SETTINGS_FIELDS,
    ConfigError,
    load_settings,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
EXAMPLES_DIR = REPO_ROOT / "examples"
EXAMPLE_PATH = EXAMPLES_DIR / "minimal_dry_run.py"
CONFIGS_DIR = REPO_ROOT / "configs"
TEMPLATE_PATH = CONFIGS_DIR / "template.yaml"
SITE_PATH = CONFIGS_DIR / "site.yaml"

SUBPROCESS_TIMEOUT_S = 60

# 冻结值：这两份字典就是 coder 必须写进 YAML 的内容，逐字段断言。
TEMPLATE_VALUES: dict[str, Any] = {
    "robot_model": "alohamini1",
    "left_port": "/dev/ttyUSB0",
    "right_port": "/dev/ttyUSB1",
    "cameras": {},
    "observation_request_window": 1,
    "watchdog_timeout_ms": 500,
    "max_loop_freq_hz": 50,
}

SITE_VALUES: dict[str, Any] = {
    "robot_model": "alohamini2pro",
    "left_port": "/dev/am_arm_follower_left",
    "right_port": "/dev/am_arm_follower_right",
    "cameras": {},
    "observation_request_window": 3,
    "watchdog_timeout_ms": 1000,
    "max_loop_freq_hz": 50,
}


def _load_yaml(path: Path) -> dict[str, Any]:
    """用 ``yaml.safe_load`` 读一份 YAML，返回顶层映射；文件缺失时给出明确信息。"""
    assert path.is_file(), f"配置文件尚未创建: {path}"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))

    return data if data is not None else {}


@pytest.fixture(scope="module")
def example_run() -> subprocess.CompletedProcess[str]:
    """在仓库根下用当前解释器跑一次示例脚本，四条契约断言复用同一份输出。"""
    return subprocess.run(
        [sys.executable, str(EXAMPLE_PATH)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=SUBPROCESS_TIMEOUT_S,
        check=False,
    )


def _describe(proc: subprocess.CompletedProcess[str]) -> str:
    """把一次子进程运行拼成便于定位的失败信息。"""
    return (
        f"cmd={sys.executable} {EXAMPLE_PATH}\n"
        f"returncode={proc.returncode}\n"
        f"--- stdout ---\n{proc.stdout}\n"
        f"--- stderr ---\n{proc.stderr}"
    )


# ---------------------------------------------------------------------------
# 契约 A：examples/minimal_dry_run.py
# ---------------------------------------------------------------------------


def test_example_script_exists() -> None:
    """契约 A：示例脚本文件存在。"""
    assert EXAMPLE_PATH.is_file(), f"示例脚本尚未创建: {EXAMPLE_PATH}"


def test_example_script_exits_zero(example_run: subprocess.CompletedProcess[str]) -> None:
    """契约 A：示例脚本以退 0 结束（离线 dry_run，不碰设备与网络）。"""
    assert example_run.returncode == 0, f"示例脚本退出码非 0\n{_describe(example_run)}"


def test_example_script_reports_zero_send_action_calls(
    example_run: subprocess.CompletedProcess[str],
) -> None:
    """契约 A：stdout 含字面量 ``0 次发送``，证明 ``send_action`` 调用次数为 0。"""
    assert "0 次发送" in example_run.stdout, f"stdout 缺少「0 次发送」\n{_describe(example_run)}"


def test_example_script_prints_step_diagnostics(
    example_run: subprocess.CompletedProcess[str],
) -> None:
    """契约 A：stdout 出现 ``steps``（每步诊断与最终报告）。"""
    assert "steps" in example_run.stdout, f"stdout 缺少「steps」\n{_describe(example_run)}"


def test_example_script_prints_residual_report(
    example_run: subprocess.CompletedProcess[str],
) -> None:
    """契约 A：stdout 出现 ``residual``（最终报告里的残差）。"""
    assert "residual" in example_run.stdout, f"stdout 缺少「residual」\n{_describe(example_run)}"


# ---------------------------------------------------------------------------
# 契约 B：configs/template.yaml 与 configs/site.yaml
# ---------------------------------------------------------------------------


def test_template_yaml_field_names_equal_settings_fields() -> None:
    """契约 B：template.yaml 的键集合与 ``SETTINGS_FIELDS`` 完全一致（不多不少）。"""
    data = _load_yaml(TEMPLATE_PATH)

    assert set(data) == set(SETTINGS_FIELDS), (
        f"template.yaml 字段不一致: 缺 {sorted(set(SETTINGS_FIELDS) - set(data))}，"
        f"多 {sorted(set(data) - set(SETTINGS_FIELDS))}"
    )


def test_site_yaml_field_names_equal_settings_fields() -> None:
    """契约 B：site.yaml 的键集合与 ``SETTINGS_FIELDS`` 完全一致（不多不少）。"""
    data = _load_yaml(SITE_PATH)

    assert set(data) == set(SETTINGS_FIELDS), (
        f"site.yaml 字段不一致: 缺 {sorted(set(SETTINGS_FIELDS) - set(data))}，"
        f"多 {sorted(set(data) - set(SETTINGS_FIELDS))}"
    )


def test_template_yaml_values_are_frozen() -> None:
    """契约 B：template.yaml 逐字段取值等于冻结的默认模板值。"""
    data = _load_yaml(TEMPLATE_PATH)

    assert data == TEMPLATE_VALUES


def test_site_yaml_values_are_frozen() -> None:
    """契约 B：site.yaml 逐字段取值等于冻结的本机现场值。"""
    data = _load_yaml(SITE_PATH)

    assert data == SITE_VALUES


def test_template_yaml_loads_with_expected_settings() -> None:
    """契约 B：``load_settings(template=...)`` 读通并返回冻结的模板值。"""
    settings = load_settings(template=TEMPLATE_PATH, env={}, check_devices=False)

    assert settings.robot_model == TEMPLATE_VALUES["robot_model"]
    assert settings.left_port == TEMPLATE_VALUES["left_port"]
    assert settings.right_port == TEMPLATE_VALUES["right_port"]
    assert settings.cameras == TEMPLATE_VALUES["cameras"]
    assert settings.observation_request_window == TEMPLATE_VALUES["observation_request_window"]
    assert settings.watchdog_timeout_ms == TEMPLATE_VALUES["watchdog_timeout_ms"]
    assert settings.max_loop_freq_hz == TEMPLATE_VALUES["max_loop_freq_hz"]


def test_site_yaml_loads_with_expected_settings() -> None:
    """契约 B：``load_settings(site=...)`` 读通并返回冻结的现场值。"""
    settings = load_settings(site=SITE_PATH, env={}, check_devices=False)

    assert settings.robot_model == SITE_VALUES["robot_model"]
    assert settings.left_port == SITE_VALUES["left_port"]
    assert settings.right_port == SITE_VALUES["right_port"]
    assert settings.cameras == SITE_VALUES["cameras"]
    assert settings.observation_request_window == SITE_VALUES["observation_request_window"]
    assert settings.watchdog_timeout_ms == SITE_VALUES["watchdog_timeout_ms"]
    assert settings.max_loop_freq_hz == SITE_VALUES["max_loop_freq_hz"]


def test_site_overrides_template_when_both_are_loaded() -> None:
    """契约 B：同时给 template 与 site 时 site 覆盖 template。"""
    settings = load_settings(
        template=TEMPLATE_PATH,
        site=SITE_PATH,
        env={},
        check_devices=False,
    )

    assert settings.left_port == "/dev/am_arm_follower_left"
    assert settings.robot_model == "alohamini2pro"


def test_template_and_site_load_together_and_the_three_fields_follow_site_yaml() -> None:
    """契约 B：三个数值字段跟着 ``configs/site.yaml`` 原文走，两份 YAML 能一起加载。

    两份 YAML 分工不同，不是同一份东西：``template.yaml`` 是随库分发的通用默认（取自
    ``overlay.py`` 的 ``AlohaMiniSettings``），``site.yaml`` 是本机现场值，也就是
    ``docs/s4-field-smoke.md`` 第 4.3 节写的那个口径——现场数字以 ``configs/site.yaml``
    为唯一出口，文档不再另存一份。

    所以期望值**不在这里硬编码、也不借 ``SITE_VALUES``**（那是「取值被冻结」的另一条契约，
    与「文档指向哪份文件」无关）：直接从 ``configs/site.yaml`` 原文用 ``yaml.safe_load``
    读出来，再与 ``load_settings`` 的返回值逐字段比。现场改配置只改 YAML，这条用例跟着走。
    """
    site_data = _load_yaml(SITE_PATH)

    try:
        settings = load_settings(
            template=TEMPLATE_PATH,
            site=SITE_PATH,
            env={},
            check_devices=False,
        )
    except ConfigError as exc:
        pytest.fail(f"两份配置一起加载抛了 ConfigError: {exc}")

    mismatches = [
        f"{field}: configs/site.yaml 读到 {site_data[field]!r}，"
        f"load_settings 返回 {getattr(settings, field)!r}"
        for field in INT_FIELDS
        if getattr(settings, field) != site_data[field]
    ]

    assert not mismatches, "三个数值字段必须与 configs/site.yaml 原文一致：" + "；".join(mismatches)


def test_loaded_window_satisfies_hard_validation() -> None:
    """契约 B：两份配置的 ``observation_request_window`` 都满足 ``>= 1`` 的硬校验。"""
    template_settings = load_settings(template=TEMPLATE_PATH, env={}, check_devices=False)
    site_settings = load_settings(site=SITE_PATH, env={}, check_devices=False)

    assert template_settings.observation_request_window >= 1
    assert site_settings.observation_request_window >= 1


# ---------------------------------------------------------------------------
# 契约 C：examples/ 静态断言
# ---------------------------------------------------------------------------


def _parse_module(path: Path) -> ast.Module:
    """把一份 Python 源文件解析成 AST，供作用域级 import 检查使用。

    Args:
        path: 源文件路径。

    Returns:
        AST 模块根节点。
    """
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _module_scope_lerobot_imports(tree: ast.Module) -> list[str]:
    """收集 AST 模块**顶层**语句里 import 到的 lerobot 模块名。

    只看 ``tree.body`` 的直接子节点（模块作用域上的语句）；函数体内的 import 不在统计
    范围，那是用 ``connect(..., client_factory=...)`` 注入真 client 时的惰性导入写法，
    合法。返回值去重且排序。

    Args:
        tree: 待检查的 AST 模块根节点。

    Returns:
        顶层 import 到的 lerobot 模块名列表；没有违规时为空列表。
    """
    names: set[str] = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            names.update(
                alias.name
                for alias in node.names
                if alias.name == "lerobot" or alias.name.startswith("lerobot.")
            )
        elif (
            isinstance(node, ast.ImportFrom)
            and node.level == 0
            and node.module is not None
            and (node.module == "lerobot" or node.module.startswith("lerobot."))
        ):
            names.add(node.module)

    return sorted(names)


def test_examples_directory_contains_python_sources() -> None:
    """契约 C：``examples/`` 下至少有一个 ``*.py`` 源文件可供静态扫描。"""
    assert EXAMPLES_DIR.is_dir(), f"examples 目录尚未创建: {EXAMPLES_DIR}"
    assert sorted(EXAMPLES_DIR.glob("*.py")), f"examples 下没有任何 *.py: {EXAMPLES_DIR}"


def test_examples_do_not_import_lerobot_at_module_scope() -> None:
    """契约 C：``examples/`` 下所有 ``*.py`` 的模块顶层都不 import ``lerobot``。

    检查改用 **AST 级作用域**判定，不再是文本级子串扫描：

    - **顶层不得**：``tree.body`` 上的 ``import lerobot...`` / ``from lerobot...`` 一律违例——
      示例脚本在无 lerobot 的主环境里必须能被加载与运行（主环境跑的是跳过分支）。
    - **函数体内允许**：``def factory(...): from lerobot... import AlohaMiniClient`` 这种惰性
      导入合法，也正是 ``examples/robot_dry_run.py`` 注入真 client 的写法。

    所以本用例拦的是「把 lerobot 拉到模块作用域」这件事，不是「文件里出现 lerobot 字样」。
    下面的 ``_module_scope_lerobot_imports`` 另有自测用例，保证这条守卫没有退化成空检查。
    """
    sources = sorted(EXAMPLES_DIR.glob("*.py"))
    assert sources, f"examples 下没有任何 *.py: {EXAMPLES_DIR}"

    offenders = {
        path.name: imports
        for path in sources
        if (imports := _module_scope_lerobot_imports(_parse_module(path)))
    }

    assert not offenders, (
        "这些示例在模块顶层 import 了 lerobot（顶层不得、函数体内的惰性导入允许）: "
        f"{offenders}；请把厂商 import 移到最内层工厂函数体内"
    )


@pytest.mark.parametrize(
    "source",
    [
        "import lerobot\n",
        "import lerobot.robots.alohamini.alohamini_client\n",
        "from lerobot import isaac\n",
        "from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient\n",
    ],
)
def test_module_scope_lerobot_guard_catches_top_level_import(source: str) -> None:
    """契约 C 自测：顶层导入 lerobot 的源码会被守卫判出违规（守卫保有拦截能力）。

    Args:
        source: 一段含顶层 lerobot 导入的源码文本。
    """
    offenders = _module_scope_lerobot_imports(ast.parse(source))

    assert offenders, f"守卫漏掉了顶层 lerobot 导入（退化成空检查）: {source!r}"
    assert all(name == "lerobot" or name.startswith("lerobot.") for name in offenders), (
        f"守卫报出的模块名不是 lerobot 体系: {offenders}"
    )


def test_module_scope_lerobot_guard_allows_import_inside_function() -> None:
    """契约 C 自测：函数体内的 lerobot 导入不算违例（惰性导入正是被允许的写法）。"""
    source = (
        "def build_factory():\n"
        "    from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient\n"
        "    return AlohaMiniClient\n"
    )

    offenders = _module_scope_lerobot_imports(ast.parse(source))

    assert offenders == [], f"函数体内的惰性 lerobot 导入被误判为违例: {offenders}"


# ---------------------------------------------------------------------------
# 契约 D：防「硬编码 0 次发送」假绿
# ---------------------------------------------------------------------------

# 计数器在 AST 里对应的属性名：``<x>.send_action_calls``。
SEND_ACTION_COUNTER_ATTR = "send_action_calls"

# 该属性在示例源码的 AST 里至少出现这么多处（注释与字符串不参与计数）：
# ``__init__`` 初始化赋值、``send_action`` 里 ``+=`` 自增、断言条件、断言消息 f-string、
# 结尾 print。真实现实测 5 处，阈值取 4 留一格余量。
MIN_SEND_ACTION_COUNTER_AST_REFS = 4

# 行为级读数：从示例子进程的真实 stdout 取数（用正则取数字，不做子串匹配）。
OBSERVATION_FRAMES_RE = re.compile(r"循环共读 (\d+) 帧观测")
SEND_ACTION_COUNT_RE = re.compile(r"send_action 调用次数: (\d+) 次发送")
MIN_OBSERVATION_FRAMES = 1
EXPECTED_SEND_ACTION_CALLS = 0

# 用于静态判定常量比较（``if 1 == 0:`` 也算死分支）的比较运算表。
_COMPARE_OPERATORS: dict[type[ast.cmpop], Any] = {
    ast.Eq: operator.eq,
    ast.NotEq: operator.ne,
    ast.Lt: operator.lt,
    ast.LtE: operator.le,
    ast.Gt: operator.gt,
    ast.GtE: operator.ge,
}


def _static_truth(node: ast.AST) -> bool | None:
    """静态判定一个测试表达式是否恒真 / 恒假，判不出时返回 ``None``。

    只认 ``ast.Constant``、两侧都是常量的简单比较、以及对它们取 ``not``，足以覆盖
    ``if False:`` / ``if 1 == 0:`` / ``while False:`` 这类死分支。

    Args:
        node: 测试表达式节点。

    Returns:
        ``True`` 恒真、``False`` 恒假、``None`` 判不出（按「两支都可能走到」处理）。
    """
    if isinstance(node, ast.Constant):
        return bool(node.value)

    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.Not):
        inner = _static_truth(node.operand)

        return None if inner is None else not inner

    if (
        isinstance(node, ast.Compare)
        and len(node.ops) == 1
        and len(node.comparators) == 1
        and isinstance(node.left, ast.Constant)
        and isinstance(node.comparators[0], ast.Constant)
    ):
        func = _COMPARE_OPERATORS.get(type(node.ops[0]))
        if func is None:
            return None
        try:
            return bool(func(node.left.value, node.comparators[0].value))
        except TypeError:
            return None

    return None


def _reachable_children(node: ast.AST) -> list[ast.AST]:
    """给出一个节点的直接子节点，跳过常量判定下的死分支。

    Args:
        node: 当前节点。

    Returns:
        仍可能被执行的直接子节点（含测试表达式本身）。
    """
    if isinstance(node, ast.If):
        truth = _static_truth(node.test)
        if truth is True:
            return [node.test, *node.body]
        if truth is False:
            return [node.test, *node.orelse]

        return [node.test, *node.body, *node.orelse]

    if isinstance(node, ast.While):
        truth = _static_truth(node.test)
        if truth is False:
            return [node.test, *node.orelse]

        return [node.test, *node.body, *node.orelse]

    return list(ast.iter_child_nodes(node))


def _reachable_nodes(tree: ast.AST) -> list[ast.AST]:
    """展开 AST，只收「在常量意义上可能被执行」的节点。

    死分支（``if False:`` / ``while False:`` / ``if 1 == 0:``）里的语句不会出现在结果里，
    而注释与 docstring 本来就不是 ``ast.Assert``，因此两种伪写法都骗不过守卫。

    Args:
        tree: 待展开的 AST 根节点。

    Returns:
        可达节点列表。
    """
    collected: list[ast.AST] = []
    stack: list[ast.AST] = [tree]
    while stack:
        node = stack.pop()
        collected.append(node)
        stack.extend(_reachable_children(node))

    return collected


def _is_send_action_counter_ref(node: ast.AST) -> bool:
    """判断节点是否为 ``<x>.send_action_calls`` 这样的属性访问。

    Args:
        node: 待判定的 AST 节点。

    Returns:
        是计数器属性访问时为 ``True``。
    """
    return isinstance(node, ast.Attribute) and node.attr == SEND_ACTION_COUNTER_ATTR


def _is_int_zero_constant(node: ast.AST) -> bool:
    """判断节点是否为整数常量 ``0``（``False`` 这类 ``bool`` 常量被排除）。

    Args:
        node: 待判定的 AST 节点。

    Returns:
        是整数 0 时为 ``True``；``bool`` 恒为 ``False``。
    """
    if not isinstance(node, ast.Constant):
        return False
    if isinstance(node.value, bool):
        return False

    return isinstance(node.value, int) and node.value == 0


def _is_send_action_zero_assert(node: ast.AST) -> bool:
    """判断节点是否为 ``assert <x>.send_action_calls == 0`` 形态的真断言。

    形如 ``assert <x>.send_action_calls == 0`` 才算命中：``test`` 是 ``ast.Compare``，
    ``left`` 是 ``send_action_calls`` 属性访问，``ops`` 含 ``ast.Eq``，比较对象里有整数
    常量 ``0``。注释与 docstring 里的同款文字根本不会成为 ``ast.Assert``；``if False:``
    里的断言虽然仍是 ``ast.Assert``，但会被 ``_reachable_nodes`` 的可达性过滤挡掉。

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
    if not _is_send_action_counter_ref(test.left):
        return False
    if not any(isinstance(op, ast.Eq) for op in test.ops):
        return False

    return any(_is_int_zero_constant(comparator) for comparator in test.comparators)


@pytest.fixture(scope="module")
def example_module() -> ModuleType:
    """从文件路径加载 ``examples/minimal_dry_run.py`` 为模块，供行为级守卫复用。

    不往 ``sys.path`` 里塞 ``examples/``（避免污染），也不 import 本测试模块的私有 helper。
    示例已用 ``if __name__ == "__main__":`` 收口，因此加载本身不会执行 ``main()``。

    Returns:
        已执行模块级代码、但未调用 ``main()`` 的示例模块对象。
    """
    spec = importlib.util.spec_from_file_location("_example_minimal_dry_run", EXAMPLE_PATH)
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


def test_example_source_asserts_real_send_action_counter() -> None:
    """契约 D：示例源码的可达 AST 里必须有 ``assert <obj>.send_action_calls == 0``。"""
    nodes = _reachable_nodes(
        ast.parse(EXAMPLE_PATH.read_text(encoding="utf-8"), filename=str(EXAMPLE_PATH))
    )

    assert any(_is_send_action_zero_assert(node) for node in nodes), (
        "示例必须真的断言 send_action_calls == 0（AST 级检查），不能只打印字面量「0 次发送」；"
        "注释、docstring、if False: 死分支里的同款文字都不算数。"
        f"请在 {EXAMPLE_PATH} 里保留形如 `assert transport.send_action_calls == 0` 的语句"
        "（`hits = t.send_action_calls; assert hits == 0`、`assert 0 == t.send_action_calls`、"
        "`assert not t.send_action_calls` 都不被接受）"
    )

    counter_refs = sum(1 for node in nodes if _is_send_action_counter_ref(node))
    assert counter_refs >= MIN_SEND_ACTION_COUNTER_AST_REFS, (
        f"示例源码的可达 AST 里 send_action_calls 属性访问只有 {counter_refs} 处，少于要求的"
        f" {MIN_SEND_ACTION_COUNTER_AST_REFS} 处（__init__ 初始化 / send_action 自增 /"
        f" 断言条件 / 断言消息 / 结尾打印；注释、字符串与死分支不参与计数）：{EXAMPLE_PATH}"
    )


def test_example_fake_transport_counter_increments_per_send(example_module: ModuleType) -> None:
    """契约 D：示例的假 transport 真实现协议，且 ``send_action_calls`` 随下发自增而非常量。"""
    # 取示例模块里的常量与替身，不在这里复制一份关节角（ruff B009 禁绝常量 getattr）。
    fake_transport_cls = example_module._FakeTransport
    start_q_deg = example_module._START_Q_DEG

    transport = fake_transport_cls(start_q_deg)

    assert transport.send_action_calls == 0, "新建 transport 的 send_action_calls 应为 0"

    transport.send_action({"arm_left_shoulder_pan.pos": 0.0})
    assert transport.send_action_calls == 1, (
        "第一次 send_action 后 send_action_calls 应为 1（计数器必须自增，不能是常量）"
    )

    transport.send_action({"arm_left_shoulder_pan.pos": 0.0})
    assert transport.send_action_calls == 2, "第二次 send_action 后 send_action_calls 应为 2"

    assert isinstance(transport, RobotTransport), (
        "示例的假 transport 必须真实现 RobotTransport 协议"
        "（get_observation / send_action / prime_observation_request_window）"
    )

    observation = transport.get_observation()
    assert isinstance(observation, Mapping), (
        f"get_observation 必须返回 Mapping，实际类型 {type(observation)}"
    )

    transport.prime_observation_request_window(include_cameras=False)
    assert transport.prime_calls == 1, (
        "prime_observation_request_window 调用一次后 prime_calls 应为 1"
    )


def test_example_run_output_shows_real_session_reads_observations(
    example_run: subprocess.CompletedProcess[str],
) -> None:
    """契约 D：示例输出的读数来自真实会话——至少读过 1 帧观测，且下发次数确为 0。"""
    assert example_run.returncode == 0, f"示例脚本退出码非 0\n{_describe(example_run)}"

    frames_match = OBSERVATION_FRAMES_RE.search(example_run.stdout)
    assert frames_match is not None, (
        "stdout 里找不到「循环共读 N 帧观测」；会话必须真的读写过观测，"
        f"否则 dry-run 门禁未被验证\n{_describe(example_run)}"
    )
    frames = int(frames_match.group(1))
    assert frames >= MIN_OBSERVATION_FRAMES, (
        f"会话只读了 {frames} 帧观测（要求 >= {MIN_OBSERVATION_FRAMES}）：说明注入给会话的"
        " transport 与结尾被断言的不是同一个实例，dry-run 门禁根本没被验证到"
        f"\n{_describe(example_run)}"
    )

    sends_match = SEND_ACTION_COUNT_RE.search(example_run.stdout)
    assert sends_match is not None, (
        f"stdout 里找不到「send_action 调用次数: N 次发送」\n{_describe(example_run)}"
    )
    assert int(sends_match.group(1)) == EXPECTED_SEND_ACTION_CALLS, (
        f"示例报告的 send_action 调用次数是 {sends_match.group(1)}，"
        f"要求 {EXPECTED_SEND_ACTION_CALLS}\n{_describe(example_run)}"
    )
