"""``alohamini_cartesian.adapter.lerobot_version`` 的版本兼容性契约（契约锁定）。

本文件是生产模块 ``src/alohamini_cartesian/adapter/lerobot_version.py`` 的行为契约：
它锁住三件事，任何一件要改都得先改这里、再改实现。

1. **区间判定**——``LEROBOT_REQUIREMENT``（即 ``lerobot>=0.6.1,<0.7``）这一段的语义：

   - 区间内通过并返回去掉首尾空白后的写法；比较口径是整数段 + 预发布次序
     ``a < b < rc < 同号正式版``（``0.6.10 > 0.6.9``、``0.10.0 > 0.7``）。
   - **上界连同它的预发布一律判拒**：PEP 440 下 ``>=0.6.1,<0.7`` 不接纳 ``0.7.0rc1``
     （``SpecifierSet.contains`` 在 ``prereleases=True`` 时同样是 ``False``），检查不得比
     声明更宽，所以不因为「排序上 ``0.7.0rc1 < 0.7``」就放行。
   - 下界只接纳同号正式版及其以上的预发布：``0.6.1rc1`` 判拒、``0.6.9rc1`` 通过。
   - 写法子集收紧到 ASCII 数字、禁前导零、限数字段长度；非 ``str`` 入参、超长数字段、
     非 ASCII 数字一律抛 ``InvalidVersionString``，不得漏出 ``AttributeError`` /
     ``TypeError`` / 裸 ``ValueError`` 这类非域异常。所有失败消息须带出原始输入。
   - 常量：``LEROBOT_MIN_VERSION``、``LEROBOT_MAX_VERSION_EXCLUSIVE``、
     ``LEROBOT_VERSION_SPECIFIER``、``LEROBOT_REQUIREMENT``。
   - 异常：``InvalidVersionString(ValueError)``、``LerobotVersionUnsupported(RuntimeError)``、
     ``LerobotNotInstalled(LerobotVersionUnsupported)``。

2. **声明防漂移**——``pyproject.toml`` 的 ``runtime`` extra 与 ``docs/ADR.md`` 必须记着同一条
   requirement，避免代码、打包声明、决策记录三处各自漂移。

3. **不得 import ``lerobot``**——``installed_lerobot_version`` 与
   ``assert_lerobot_version_supported`` 只通过 ``importlib.metadata.version("lerobot")`` 取版本；
   源码里不得出现指向 ``lerobot`` 的 import（AST 守卫 + ``sys.modules`` 前后对照）。

全程离线：不连设备、不联网、不安装 ``lerobot``。两条 metadata 用例用 monkeypatch 顶替
``importlib.metadata.version``，所以被测实现必须以属性访问方式调用它
（``importlib.metadata.version("lerobot")``），不能在模块顶层
``from importlib.metadata import version``，否则 monkeypatch 打不进去。
"""

from __future__ import annotations

import ast
import importlib
import importlib.metadata
import re
import sys
import tomllib
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_NAME = "alohamini_cartesian.adapter.lerobot_version"
MODULE_PATH = REPO_ROOT / "src" / "alohamini_cartesian" / "adapter" / "lerobot_version.py"
PYPROJECT_PATH = REPO_ROOT / "pyproject.toml"
ADR_PATH = REPO_ROOT / "docs" / "ADR.md"

# 期望的声明窗口：与厂商 fork 的 pyproject `version = "0.6.1"` 及 Pi 现场 HEAD 同一版本线对齐。
EXPECTED_MIN_VERSION = "0.6.1"
EXPECTED_MAX_VERSION_EXCLUSIVE = "0.7"
EXPECTED_SPECIFIER = ">=0.6.1,<0.7"
EXPECTED_REQUIREMENT = "lerobot>=0.6.1,<0.7"
EXPECTED_LOWER_CLAUSE = f">={EXPECTED_MIN_VERSION}"
EXPECTED_UPPER_CLAUSE = f"<{EXPECTED_MAX_VERSION_EXCLUSIVE}"

# 要求区间在异常消息里的正则形态：用 ``re.escape`` 转义 ``.``，并用两个前瞻断言
# 让「含下限」与「含上界」与出现顺序无关（``>``、``<``、``=`` 本身不是正则元字符）。
WINDOW_CLAUSE_PATTERN = (
    rf"(?=.*{re.escape(EXPECTED_LOWER_CLAUSE)})(?=.*{re.escape(EXPECTED_UPPER_CLAUSE)})"
)


def _lerobot_version_module() -> ModuleType:
    """按模块名导入待实现的 lerobot 版本模块，让每个用例各自报出「模块还不存在」。"""
    return importlib.import_module(MODULE_NAME)


def _missing_package(name: str) -> str:
    """顶替 ``importlib.metadata.version``：模拟找不到分发，抛 ``PackageNotFoundError``。"""
    raise importlib.metadata.PackageNotFoundError(name)


def _reporting_input(name: str) -> str:
    """顶替 ``importlib.metadata.version``：返回固定版本串，并记录被查询的分发名。"""
    _QUERIED_NAMES.append(name)
    return "0.6.1"


def _must_not_be_called(name: str) -> str:
    """顶替 ``importlib.metadata.version``：一旦被调用就让用例失败，证明路径没读 metadata。"""
    raise AssertionError(f"传入显式版本时不应查询 metadata，实际查了 {name!r}")


_QUERIED_NAMES: list[str] = []


def _version_occurs_in(message: str, version: str) -> bool:
    """判断消息是否把该版本写成完整版号（两侧不再紧跟数字或点，避免 ``0.6`` 命中 ``0.6.1``）。"""
    return re.search(rf"(?<![\d.]){re.escape(version)}(?![\d.])", message) is not None


def _message_reports_input(message: str, raw: str) -> bool:
    """判断异常消息是否带出了原始输入的可辨识原文（``repr`` 形式或去空白后的原文均可）。"""
    stripped = raw.strip()
    return repr(raw) in message or (bool(stripped) and stripped in message)


# ---------------------------------------------------------------------------
# 1-3 区间内：下限本身、区间内正式版、区间内 pre-release
# ---------------------------------------------------------------------------


def test_check_accepts_lower_bound_version() -> None:
    """下限 ``0.6.1`` 本身属于支持区间，返回原样字符串。"""
    module = _lerobot_version_module()

    normalized = module.check_lerobot_version("0.6.1")

    assert normalized == "0.6.1"


@pytest.mark.parametrize(
    ("installed", "expected"),
    [("0.6.2", "0.6.2"), ("0.6.99", "0.6.99"), (" 0.6.2 ", "0.6.2")],
    ids=["patch-increment", "high-patch", "surrounding-whitespace"],
)
def test_check_accepts_versions_inside_window(installed: str, expected: str) -> None:
    """区间内正式版通过，返回值去掉了首尾空白（``" 0.6.2 "`` → ``"0.6.2"``）。"""
    module = _lerobot_version_module()

    normalized = module.check_lerobot_version(installed)

    assert normalized == expected


def test_check_accepts_in_window_pre_release() -> None:
    """区间内 pre-release ``0.6.2rc1`` 大于下限 ``0.6.1``，通过。"""
    module = _lerobot_version_module()

    normalized = module.check_lerobot_version("0.6.2rc1")

    assert normalized == "0.6.2rc1"


@pytest.mark.parametrize(
    "installed",
    ["0.6.10", "0.6.100"],
    ids=["patch-ten", "patch-hundred"],
)
def test_check_accepts_multi_digit_patch_inside_the_window(installed: str) -> None:
    """多位补丁号 ``0.6.10`` / ``0.6.100`` 在区间内通过，并原样返回规范化写法。"""
    module = _lerobot_version_module()

    normalized = module.check_lerobot_version(installed)

    assert normalized == installed


def test_check_accepts_multi_digit_and_single_digit_patch_alike() -> None:
    """``0.6.10`` 与 ``0.6.9`` 同样通过：补丁号位数变化不改变「是否在区间内」的判定。"""
    module = _lerobot_version_module()

    # 纯序比较下 0.6.10 > 0.6.9；两者都在 [0.6.1, 0.7) 内，因此都通过。
    # 这里不做跨版本排序 API 的断言，只锁「位数不影响接受/拒绝」。
    assert module.check_lerobot_version("0.6.10") == "0.6.10"
    assert module.check_lerobot_version("0.6.9") == "0.6.9"


# ---------------------------------------------------------------------------
# 4 低于下限：抛 LerobotVersionUnsupported，消息带原文与要求区间
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "installed",
    ["0.5.9", "0.6.0", "0.6", "0.6.1rc1", "0.6.1a1", "0.6.1b2"],
    ids=[
        "below-minor",
        "below-patch",
        "two-segments-before-bound",
        "rc-of-lower-bound",
        "alpha-of-lower-bound",
        "beta-of-lower-bound",
    ],
)
def test_check_rejects_versions_below_lower_bound(installed: str) -> None:
    """低于下限的写法（含下限的 pre-release）抛不支持，消息含版本原文与要求区间。"""
    module = _lerobot_version_module()

    with pytest.raises(module.LerobotVersionUnsupported) as excinfo:
        module.check_lerobot_version(installed)

    message = str(excinfo.value)
    assert _version_occurs_in(message, installed), message
    assert EXPECTED_LOWER_CLAUSE in message
    assert EXPECTED_UPPER_CLAUSE in message


# ---------------------------------------------------------------------------
# 5 到达 / 超过上限：同上口径
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "installed",
    ["0.7", "0.7.0", "0.7.1", "1.0"],
    ids=["upper-bound-two-segments", "upper-bound-three-segments", "above-upper-bound", "major"],
)
def test_check_rejects_versions_at_or_above_upper_bound(installed: str) -> None:
    """等于或高于上界 ``0.7`` 的写法抛不支持，消息含版本原文与要求区间。"""
    module = _lerobot_version_module()

    with pytest.raises(module.LerobotVersionUnsupported) as excinfo:
        module.check_lerobot_version(installed)

    message = str(excinfo.value)
    assert _version_occurs_in(message, installed), message
    assert EXPECTED_LOWER_CLAUSE in message
    assert EXPECTED_UPPER_CLAUSE in message


@pytest.mark.parametrize(
    "installed",
    ["0.7.0rc1", "0.7.0a1", "0.7.0b2", "0.7rc1", "0.7a1"],
    ids=[
        "rc-of-upper-bound-with-patch",
        "alpha-of-upper-bound-with-patch",
        "beta-of-upper-bound-with-patch",
        "rc-of-upper-bound",
        "alpha-of-upper-bound",
    ],
)
def test_check_rejects_pre_release_of_upper_bound(installed: str) -> None:
    """上界 ``0.7`` 的预发布一律判拒，消息带版本原文与要求区间。

    纯序比较会把 ``0.7.0rc1`` 判成 ``< 0.7`` 而放行，但声明 ``<0.7`` 在 PEP 440 下
    并不接纳它；检查比声明更宽就等于放行了声明明确排除的版本，fail-closed 口径要求
    在这里抛不支持。
    """
    module = _lerobot_version_module()

    with pytest.raises(module.LerobotVersionUnsupported) as excinfo:
        module.check_lerobot_version(installed)

    message = str(excinfo.value)
    assert _version_occurs_in(message, installed), message
    assert EXPECTED_LOWER_CLAUSE in message
    assert EXPECTED_UPPER_CLAUSE in message


@pytest.mark.parametrize(
    "installed",
    ["0.6.9rc1", "0.6.9a1"],
    ids=["rc-below-upper-bound", "alpha-below-upper-bound"],
)
def test_check_accepts_pre_release_below_upper_bound(installed: str) -> None:
    """上界之下的预发布照旧通过：它们同号正式版 ``0.6.9`` 也在区间内。

    收紧的是「上界自己的预发布」，不是所有预发布；这条与上一条配对，防止把判定改成
    「凡预发布皆拒」。
    """
    module = _lerobot_version_module()

    assert module.check_lerobot_version(installed) == installed


# ---------------------------------------------------------------------------
# 6 预发布次序：a < b < rc < 同号正式版
# ---------------------------------------------------------------------------

_ASCENDING_STAGE_VERSIONS = ["0.6.2a1", "0.6.2b1", "0.6.2rc1", "0.6.2"]


def test_pre_release_stage_order_is_ascending() -> None:
    """预发布次序 ``a < b < rc < 同号正式版`` 严格递增。

    公开入口 ``check_lerobot_version`` 只回答「在不在区间内」：这四个串都在
    ``[0.6.1, 0.7)`` 里、一律通过，反转次序也照样通过，所以必须另找口径。本条读的是
    模块内的私有排序键 ``_parse_version``——**它锁的是私有 ``_parse_version`` 的排序键，
    私有名一旦改名，本条需同步改**。选它而不是靠 monkeypatch 收敛 ``LEROBOT_MIN_VERSION`` /
    ``LEROBOT_MAX_VERSION_EXCLUSIVE`` 去探测边界，是因为后者会额外要求「常量在调用时读取、
    不得在导入期预计算」这一实现细节，正常重构就会误红。

    为免私有键与公开判定各说一套，末尾再用公开入口确认这四个串都在区间内、且原样返回。
    """
    module = _lerobot_version_module()

    keys = [module._parse_version(version) for version in _ASCENDING_STAGE_VERSIONS]

    paired = list(zip(_ASCENDING_STAGE_VERSIONS, keys, strict=True))

    assert keys == sorted(keys), f"预发布次序必须递增：{paired}"
    assert len(set(keys)) == len(keys), f"四个串的排序键必须两两不等：{paired}"
    accepted = [module.check_lerobot_version(version) for version in _ASCENDING_STAGE_VERSIONS]
    assert accepted == _ASCENDING_STAGE_VERSIONS


@pytest.mark.parametrize(
    "installed",
    ["0.10.0", "0.10", "0.9.0", "0.100.0"],
    ids=["minor-ten-with-patch", "minor-ten", "minor-nine", "minor-hundred"],
)
def test_check_rejects_multi_digit_minor_above_the_window(installed: str) -> None:
    """次版本号 ``0.10`` / ``0.9`` 按整数段比较已越过上界 ``0.7``，抛不支持。

    本组只作越界回归，**不具比较口径的判别力**：字符串比较下 ``0.10.0`` / ``0.10`` /
    ``0.100.0`` 会先撞下限（元组字符串比较里 ``"10" < "6"``，早于上界判定就被拦），
    照样抛不支持；``0.9.0`` 两种口径都拒。真正能分辨「整数段比较 vs 字符串比较」的参数在
    ``test_check_rejects_minor_that_only_string_comparison_would_accept``。
    """
    module = _lerobot_version_module()

    with pytest.raises(module.LerobotVersionUnsupported, match=WINDOW_CLAUSE_PATTERN) as excinfo:
        module.check_lerobot_version(installed)

    message = str(excinfo.value)
    assert _version_occurs_in(message, installed), message


@pytest.mark.parametrize(
    "installed",
    ["0.60.0", "0.69.9", "0.60"],
    ids=["minor-sixty-with-patch", "minor-sixty-nine-with-patch", "minor-sixty"],
)
def test_check_rejects_minor_that_only_string_comparison_would_accept(installed: str) -> None:
    """次版本号 ``60`` / ``69`` 按整数段比较越过上界 ``0.7``，必须判为不支持。

    这组参数专门分辨比较口径，是本文件唯一能抓住「把整数段比较换成字符串比较」的判据：
    字符串比较下 ``"60" > "6"``（不会被下限 ``0.6.1`` 拦下）且 ``"60" < "7"``（不会
    被上界 ``0.7`` 拦下），于是会被**错误接受**；只有整数段比较（``60 > 7``）才判为超过上限。
    上一条那组参数（``0.10.0`` 等）在字符串比较下会先撞下限，抓不住这个变异。
    """
    module = _lerobot_version_module()

    with pytest.raises(module.LerobotVersionUnsupported, match=WINDOW_CLAUSE_PATTERN) as excinfo:
        module.check_lerobot_version(installed)

    message = str(excinfo.value)
    assert _version_occurs_in(message, installed), message


# ---------------------------------------------------------------------------
# 7 无法解析的写法：抛 InvalidVersionString，消息带原文
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "raw",
    [
        "",
        "   ",
        "v0.6.1",
        "0.6.x",
        "latest",
        "0.6.1.post1",
        "0.6.1.dev2",
        "0.6.1+local",
        "0.6.1.2",
        "0.6.1rc",
    ],
    ids=[
        "empty",
        "whitespace-only",
        "v-prefix",
        "x-wildcard",
        "latest",
        "post-release",
        "dev-release",
        "local-version",
        "four-segments",
        "rc-without-number",
    ],
)
def test_check_rejects_unparsable_version_strings(raw: str) -> None:
    """PEP 440 子集之外的写法一律抛非法版本，消息带出原始输入。"""
    module = _lerobot_version_module()

    with pytest.raises(module.InvalidVersionString) as excinfo:
        module.check_lerobot_version(raw)

    message = str(excinfo.value)
    assert _message_reports_input(message, raw), message
    assert issubclass(module.InvalidVersionString, ValueError)


@pytest.mark.parametrize(
    "raw",
    ["0.6.01", "00.006.0001", "0.06.1", "٠.٦.١", "０.６.１"],
    ids=[
        "leading-zero-patch",
        "leading-zeros-everywhere",
        "leading-zero-minor",
        "arabic-indic-digits",
        "fullwidth-digits",
    ],
)
def test_check_rejects_non_ascii_or_padded_segments(raw: str) -> None:
    r"""数字段只认 ASCII 数字且不带前导零，非 ASCII 数字与前导零一律抛非法版本。

    Python 正则的 ``\d`` 默认匹配 Unicode 十进制数字，``٠.٦.١`` 与 ``０.６.１`` 都能被
    ``int()`` 转成 0/6/1 而悄悄通过；``0.6.01`` 也会被当成 ``0.6.1`` 放行。三种写法都要
    fail-closed 到「版本串非法」，消息带出原始输入。
    """
    module = _lerobot_version_module()

    with pytest.raises(module.InvalidVersionString) as excinfo:
        module.check_lerobot_version(raw)

    message = str(excinfo.value)
    assert _message_reports_input(message, raw), message


@pytest.mark.parametrize(
    "raw",
    ["1234567890.0.0", "0.1234567890.0", "0.0.1234567890"],
    ids=["ten-digit-major", "ten-digit-minor", "ten-digit-patch"],
)
def test_check_rejects_segment_longer_than_the_documented_limit(raw: str) -> None:
    """数字段超过文档写明的「每段至多 9 位」即抛非法版本，消息带出原始输入。

    把实现里的 ``_MAX_SEGMENT_DIGITS`` 从 9 放开到 100，这条必须红：文档与异常消息都
    承诺了 9 位上限，没有可执行判据时这个数字只是注释。三个位置（大/次/补丁号）都要守，
    免得只给其中一段加上限。
    """
    module = _lerobot_version_module()

    with pytest.raises(module.InvalidVersionString) as excinfo:
        module.check_lerobot_version(raw)

    message = str(excinfo.value)
    assert _message_reports_input(message, raw), message


@pytest.mark.parametrize(
    "raw",
    ["0.6.1RC1", "0.6.2RC1", "0.6.2A1", "0.6.2B1"],
    ids=["uppercase-rc-of-lower-bound", "uppercase-rc", "uppercase-alpha", "uppercase-beta"],
)
def test_check_rejects_uppercase_pre_release_suffix(raw: str) -> None:
    """预发布后缀只认小写 ``a`` / ``b`` / ``rc``；大写写法抛非法版本，消息带出原始输入。

    支持的写法子集明文收紧到小写后缀（PEP 440 本身大小写皆可，这里刻意更窄）。给正则
    加 ``re.IGNORECASE`` 就会把这四个串放行，这条必须红：宽度只能由文档声明决定，不能靠
    实现时的宽松默认。
    """
    module = _lerobot_version_module()

    with pytest.raises(module.InvalidVersionString) as excinfo:
        module.check_lerobot_version(raw)

    message = str(excinfo.value)
    assert _message_reports_input(message, raw), message


@pytest.mark.parametrize(
    "raw",
    [f"{'9' * 5000}.0.0", f"0.{'9' * 5000}.0"],
    ids=["oversized-major", "oversized-minor"],
)
def test_check_rejects_oversized_segments(raw: str) -> None:
    """超长数字段抛域内的 ``InvalidVersionString``，不是裸 ``ValueError``。

    ``InvalidVersionString`` 是 ``ValueError`` 子类，``pytest.raises`` 按子类匹配：实现若
    先把数字段交给 ``int()``，CPython 会抛「超过 4300 位上限」的裸 ``ValueError``，本条即红。
    消息里还要带出原始输入，并把那条 CPython 措辞排除掉，免得同样措辞冒充域内异常。
    """
    module = _lerobot_version_module()

    with pytest.raises(module.InvalidVersionString) as excinfo:
        module.check_lerobot_version(raw)

    message = str(excinfo.value)
    assert _message_reports_input(message, raw), message[:200]
    assert "Exceeds the limit" not in message, message[:200]


@pytest.mark.parametrize(
    "raw",
    [None, 6.1, b"0.6.1", ["0.6.1"]],
    ids=["none", "float", "bytes", "list"],
)
def test_check_rejects_non_string_input(raw: object) -> None:
    """非 ``str`` 入参抛 ``InvalidVersionString``，消息含入参的 ``repr``。

    版本串来自配置、环境变量或包元数据，类型不对是可预见的现场情形；fail-closed 要求把它
    归到「版本串非法」这一个域内失败面上，而不是漏出 ``AttributeError``（``None`` / ``float``
    / ``list`` 没有 ``.strip()``）或 ``TypeError``（正则打 bytes），否则调用方得捕两类异常。
    """
    module = _lerobot_version_module()

    with pytest.raises(module.InvalidVersionString) as excinfo:
        module.check_lerobot_version(raw)

    message = str(excinfo.value)
    assert repr(raw) in message, message


# ---------------------------------------------------------------------------
# 8 installed_lerobot_version：metadata 缺席 / 命中
# ---------------------------------------------------------------------------


def test_installed_version_translates_missing_package(monkeypatch: pytest.MonkeyPatch) -> None:
    """``lerobot`` 未安装时转成 ``LerobotNotInstalled``，消息含 ``lerobot`` 与要求区间。"""
    module = _lerobot_version_module()
    monkeypatch.setattr(importlib.metadata, "version", _missing_package)

    assert issubclass(module.LerobotNotInstalled, module.LerobotVersionUnsupported)

    with pytest.raises(module.LerobotNotInstalled) as excinfo:
        module.installed_lerobot_version()

    message = str(excinfo.value)
    assert "lerobot" in message
    assert EXPECTED_LOWER_CLAUSE in message
    assert EXPECTED_UPPER_CLAUSE in message


def test_missing_package_is_catchable_as_unsupported(monkeypatch: pytest.MonkeyPatch) -> None:
    """缺包异常能被父类 ``LerobotVersionUnsupported`` 捕获，调用方按一类处理即可。"""
    module = _lerobot_version_module()
    monkeypatch.setattr(importlib.metadata, "version", _missing_package)

    with pytest.raises(module.LerobotVersionUnsupported):
        module.installed_lerobot_version()


def test_installed_version_returns_metadata_string(monkeypatch: pytest.MonkeyPatch) -> None:
    """装好的 ``lerobot`` 返回 metadata 原样给出的版本串，并且查的分发名是 ``lerobot``。"""
    module = _lerobot_version_module()
    _QUERIED_NAMES.clear()
    monkeypatch.setattr(importlib.metadata, "version", _reporting_input)

    installed = module.installed_lerobot_version()

    assert installed == "0.6.1"
    assert _QUERIED_NAMES == ["lerobot"]


# ---------------------------------------------------------------------------
# 9 assert_lerobot_version_supported：默认读 metadata，显式入参不读
# ---------------------------------------------------------------------------


def test_assert_supported_accepts_installed_version(monkeypatch: pytest.MonkeyPatch) -> None:
    """不传参时读 metadata，返回通过校验的版本串。"""
    module = _lerobot_version_module()
    monkeypatch.setattr(importlib.metadata, "version", _reporting_input)

    supported = module.assert_lerobot_version_supported()

    assert supported == "0.6.1"


def test_assert_supported_rejects_too_old_installed_version(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """不传参且环境里的 ``0.6.0`` 低于下限时抛不支持。"""
    module = _lerobot_version_module()

    def _too_old(name: str) -> str:
        """顶替 metadata：模拟装了低于下限的 ``lerobot``。"""
        assert name == "lerobot"
        return "0.6.0"

    monkeypatch.setattr(importlib.metadata, "version", _too_old)

    with pytest.raises(module.LerobotVersionUnsupported):
        module.assert_lerobot_version_supported()


def test_assert_supported_uses_explicit_version_without_metadata(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """传入显式版本时只校验该串、不读 metadata（metadata 一旦被调用就让用例炸掉）。"""
    module = _lerobot_version_module()
    monkeypatch.setattr(importlib.metadata, "version", _must_not_be_called)

    supported = module.assert_lerobot_version_supported("0.6.3")

    assert supported == "0.6.3"


# ---------------------------------------------------------------------------
# 0 常量：四个公开名字的取值
# ---------------------------------------------------------------------------


def test_version_constants_lock_declared_window() -> None:
    """四个公开常量的取值与声明窗口一致，避免口头约定漂移。"""
    module = _lerobot_version_module()

    assert module.LEROBOT_MIN_VERSION == EXPECTED_MIN_VERSION
    assert module.LEROBOT_MAX_VERSION_EXCLUSIVE == EXPECTED_MAX_VERSION_EXCLUSIVE
    assert module.LEROBOT_VERSION_SPECIFIER == EXPECTED_SPECIFIER
    assert module.LEROBOT_REQUIREMENT == EXPECTED_REQUIREMENT


# ---------------------------------------------------------------------------
# 10 pyproject 声明防漂移
# ---------------------------------------------------------------------------


def _pyproject_project_section() -> dict[str, object]:
    """读仓库根 ``pyproject.toml`` 并返回 ``[project]`` 段。"""
    pyproject = tomllib.loads(PYPROJECT_PATH.read_text(encoding="utf-8"))
    project = pyproject["project"]
    assert isinstance(project, dict)
    return project


def test_pyproject_runtime_extra_holds_exact_requirement() -> None:
    """``runtime`` extra 存在且内容恰好是 ``[LEROBOT_REQUIREMENT]``。"""
    module = _lerobot_version_module()
    project = _pyproject_project_section()
    optional = project["optional-dependencies"]
    assert isinstance(optional, dict)

    assert "runtime" in optional
    runtime_extra = optional["runtime"]
    assert runtime_extra == [module.LEROBOT_REQUIREMENT]
    assert EXPECTED_REQUIREMENT in runtime_extra


def test_pyproject_keeps_lerobot_optional() -> None:
    """``lerobot`` 不得出现在必需依赖里，核心仍可零第三方包导入。"""
    project = _pyproject_project_section()
    core_dependencies = project["dependencies"]
    assert isinstance(core_dependencies, list)

    assert [dep for dep in core_dependencies if dep.lower().startswith("lerobot")] == []


def test_pyproject_requirement_matches_specifier_constant() -> None:
    """``LEROBOT_VERSION_SPECIFIER`` 能拼出 ``pyproject.toml`` 里那一段 requirement。"""
    module = _lerobot_version_module()
    project = _pyproject_project_section()
    optional = project["optional-dependencies"]
    assert isinstance(optional, dict)
    runtime_extra = optional["runtime"]
    assert isinstance(runtime_extra, list)

    assert f"lerobot{module.LEROBOT_VERSION_SPECIFIER}" in runtime_extra
    assert EXPECTED_REQUIREMENT in runtime_extra


# ---------------------------------------------------------------------------
# 11 ADR 防漂移：决策记录必须写下同一条约束
# ---------------------------------------------------------------------------


def test_adr_records_lerobot_version_window() -> None:
    """ADR 必须记录同一条 ``lerobot>=0.6.1,<0.7``，否则决策依据与实现脱节。"""
    adr_text = ADR_PATH.read_text(encoding="utf-8")

    assert EXPECTED_REQUIREMENT in adr_text, (
        f"ADR 需记录同一约束：{ADR_PATH} 里没有出现 {EXPECTED_REQUIREMENT!r}；"
        "版本区间只写进 pyproject 与代码、不写进 ADR，后续换版本时无从判断依据。"
    )


# ---------------------------------------------------------------------------
# 12 不得 import lerobot：源码 AST 级守卫 + 导入后 sys.modules 不变
# ---------------------------------------------------------------------------


def test_module_source_never_imports_lerobot() -> None:
    """模块源码里没有任何指向 ``lerobot`` 的 import 语句（AST 级守卫）。"""
    tree = ast.parse(MODULE_PATH.read_text(encoding="utf-8"))

    imported: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported.append(node.module or "")

    assert [name for name in imported if name == "lerobot" or name.startswith("lerobot.")] == []


def test_importing_module_does_not_pull_lerobot_into_sys_modules() -> None:
    """导入本模块不会把 ``lerobot`` 拉进 ``sys.modules``，兼容性检查不需要它。"""
    before = {name for name in sys.modules if name == "lerobot" or name.startswith("lerobot.")}

    importlib.import_module(MODULE_NAME)

    after = {name for name in sys.modules if name == "lerobot" or name.startswith("lerobot.")}
    assert after == before
