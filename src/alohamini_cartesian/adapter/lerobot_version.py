"""``lerobot`` 运行时版本兼容性检查：只用标准库，不 import ``lerobot``。

本库与厂商运行时之间只有 :mod:`alohamini_cartesian.adapter.protocol` 这一层协议面，
而协议面是否还对得上，取决于环境里装的是哪一段 ``lerobot``。本模块把「支持哪一段」
写成常量、把「装的是哪一个」读自 :mod:`importlib.metadata`，再给一个不依赖第三方包、
也不会把 ``lerobot`` 拉进 ``sys.modules`` 的比较函数。三个失败面各有专门异常、
都不静默降级：解析不了抛 :class:`InvalidVersionString`，越界抛
:class:`LerobotVersionUnsupported`，缺包抛 :class:`LerobotNotInstalled`。

支持的写法是 PEP 440 的一个子集：``X.Y`` / ``X.Y.Z``，可选 ``aN`` / ``bN`` / ``rcN``
预发布后缀（小写）。数字段必须是 ASCII 十进制、无前导零（``0`` 本身合法）、每段至多
``_MAX_SEGMENT_DIGITS`` 位：``0.6.01``、``00.006.0001``、``٠.٦.١``（阿拉伯-印度数字）、
``０.６.１``（全角数字）与几千位的数字段一律抛 :class:`InvalidVersionString`。长度检查写在
正则里、先于整数转换，超长段不会被送进 ``int()``，因此不会漏成「超过 4300 位」的裸
``ValueError``。非 ``str`` 入参（``None``、``6.1``、``bytes``、``list``）同样抛
:class:`InvalidVersionString` 并带出入参的 ``repr``；唯一例外是
:func:`assert_lerobot_version_supported` 的 ``None``——那表示「去读 metadata」，不是版本串。

区间判定按 **整数段** 比较（``0.6.10 > 0.6.9``、``0.10.0 > 0.7``，不按字符串比）。预发布的
**排序**按 PEP 440 纯序（``a`` < ``b`` < ``rc`` < 同号正式版）；**成员判定**比 PEP 440 的
``<`` **更严**：上界连同它的预发布一律判拒（``0.7.0rc1`` 判拒，不因为排序上
``0.7.0rc1 < 0.7`` 就放行），理由是声明窗口 ``<0.7`` 排除它，检查不得比声明宽松。下界一侧
只接纳同号正式版及其以上的预发布（``0.6.1rc1`` 判拒、``0.6.9rc1`` 通过）。
"""

from __future__ import annotations

import importlib.metadata
import re

# 下限取自厂商 fork 的 pyproject.toml（version = "0.6.1"），这正是 Pi 现场运行的版本线；
# 上界与上游无「不破坏兼容」承诺（classifier 为 Alpha）对应，跨 minor 未验证。
LEROBOT_MIN_VERSION = "0.6.1"
LEROBOT_MAX_VERSION_EXCLUSIVE = "0.7"
LEROBOT_VERSION_SPECIFIER = f">={LEROBOT_MIN_VERSION},<{LEROBOT_MAX_VERSION_EXCLUSIVE}"
LEROBOT_REQUIREMENT = f"lerobot{LEROBOT_VERSION_SPECIFIER}"

# 数字段只认 ASCII 十进制、无前导零、至多这么多位。上限写进正则而不是交给 int()：
# CPython 3.11+ 对超过 4300 位的十进制字符串转换会抛裸 ValueError，那会漏出域外异常。
_MAX_SEGMENT_DIGITS = 9
_SEGMENT = rf"0|[1-9][0-9]{{0,{_MAX_SEGMENT_DIGITS - 1}}}"
# 写法子集：X.Y / X.Y.Z，可选预发布后缀。
_VERSION_PATTERN = re.compile(
    rf"(?P<major>{_SEGMENT})\.(?P<minor>{_SEGMENT})"
    rf"(?:\.(?P<patch>{_SEGMENT}))?"
    rf"(?:(?P<stage>a|b|rc)(?P<serial>{_SEGMENT}))?"
)
# 纯序的预发布次序：a < b < rc < 同号正式版（正式版用 RELEASE_ORDER 表示）。
_STAGE_ORDER = {"a": 0, "b": 1, "rc": 2}
_RELEASE_ORDER = 3


class InvalidVersionString(ValueError):
    """版本字符串不在支持的写法子集里。

    fail-closed 语义：看不出来是哪个版本就抛错，不猜一个默认版本顶过去。
    """


class LerobotVersionUnsupported(RuntimeError):
    """``lerobot`` 的版本不在支持区间 ``[0.6.1, 0.7)`` 内。

    fail-closed 语义：越界的版本照样抛错，不截断到区间端点，也不静默放行。
    """


class LerobotNotInstalled(LerobotVersionUnsupported):
    """环境里找不到 ``lerobot`` 分发。

    继承 :class:`LerobotVersionUnsupported`，调用方要「对任何不可用情形一并处理」
    时只捕父类即可。
    """


def _parse_version(raw: str) -> tuple[int, int, int, int, int]:
    """把版本串解析成可比较的整数五元组，无法解析时抛 :class:`InvalidVersionString`。

    Args:
        raw: 待解析的版本串，允许首尾有空白；非 ``str`` 入参也在这里判为非法。

    Returns:
        ``(major, minor, patch, stage_rank, stage_serial)``；缺省补丁号为 ``0``，
        正式版的 ``stage_rank`` 为 ``_RELEASE_ORDER``。

    Raises:
        InvalidVersionString: 入参不是 ``str``，或写法不在 ``X.Y`` / ``X.Y.Z`` + 可选
            ``aN`` / ``bN`` / ``rcN`` 之内。数字段非 ASCII、带前导零或超过
            ``_MAX_SEGMENT_DIGITS`` 位同样在此判非法（长度检查写在正则里，先于 ``int()``）。
    """
    if not isinstance(raw, str):
        raise InvalidVersionString(
            f"lerobot 版本必须是 str，收到 {raw!r}（类型 {type(raw).__name__}）："
            "版本串来源可以是配置、环境变量或包元数据，类型不对一并按非法版本处理；"
            "「传 None 表示读 metadata」只对 assert_lerobot_version_supported() 成立"
        )
    match = _VERSION_PATTERN.fullmatch(raw.strip())
    if match is None:
        raise InvalidVersionString(
            f"无法解析 lerobot 版本字符串 {raw!r}：支持 X.Y 或 X.Y.Z，"
            "可带 aN / bN / rcN 预发布后缀（例如 0.6.1、0.6.2rc1）；"
            f"数字段须为 ASCII 十进制、无前导零、每段至多 {_MAX_SEGMENT_DIGITS} 位"
        )
    stage = match.group("stage")
    serial = match.group("serial")
    return (
        int(match.group("major")),
        int(match.group("minor")),
        int(match.group("patch") or 0),
        _RELEASE_ORDER if stage is None else _STAGE_ORDER[stage],
        int(serial or 0),
    )


def _exceeds_upper_bound(key: tuple[int, int, int, int, int]) -> bool:
    """判定解析后的键是否被上界拒绝：上界连同它的预发布都算越界（fail-closed）。

    PEP 440 的 ``<V`` 不接纳 V 自身的预发布（``<0.7`` 排除 ``0.7.0rc1``），所以判定比纯序
    更严：发布段三元组到了上界就拒，不因 ``0.7.0rc1 < 0.7`` 而放行——声明窗口排除它，
    检查不得比声明宽松。

    Args:
        key: :func:`_parse_version` 给出的整数五元组。

    Returns:
        ``True`` 表示到达上界发布段（含上界自身与它的预发布）或更高的版本。
    """
    upper_key = _parse_version(LEROBOT_MAX_VERSION_EXCLUSIVE)
    if key >= upper_key:
        return True
    return key[3] != _RELEASE_ORDER and key[:3] >= upper_key[:3]


def check_lerobot_version(installed: str) -> str:
    """校验一个版本串是否落在支持区间 ``[LEROBOT_MIN_VERSION, LEROBOT_MAX_VERSION_EXCLUSIVE)``。

    预发布日期排序按 PEP 440 纯序（``a`` < ``b`` < ``rc`` < 同号正式版），但成员判定比
    PEP 440 的 ``<`` 更严：上界连同它的预发布一律判拒（``0.7.0rc1`` 不在区间内，即使纯序上
    它小于 ``0.7``）；下界一侧只接纳同号正式版及其以上的预发布（``0.6.1rc1`` 判拒、
    ``0.6.9rc1`` 通过）。

    Args:
        installed: 待校验的版本串，允许首尾有空白；非 ``str`` 入参抛
            :class:`InvalidVersionString`（fail-closed，不是 ``TypeError``）。

    Returns:
        去掉首尾空白后的版本串；即 ``" 0.6.2 "`` 返回 ``"0.6.2"``。

    Raises:
        InvalidVersionString: 写法不在支持的子集内，或入参不是 ``str``。
        LerobotVersionUnsupported: 解析成功但低于下限、或到达/超过上界（含上界的预发布）。
    """
    key = _parse_version(installed)
    if key < _parse_version(LEROBOT_MIN_VERSION) or _exceeds_upper_bound(key):
        raise LerobotVersionUnsupported(
            f"lerobot 版本 {installed} 不在支持区间内：要求 {LEROBOT_VERSION_SPECIFIER}"
            f"（即 {LEROBOT_REQUIREMENT}）"
        )
    return installed.strip()


def installed_lerobot_version() -> str:
    """读环境里装着的 ``lerobot`` 版本串。

    只查包元数据，不 ``import lerobot``：兼容性检查不需要真的把机器人运行时载进来。

    Returns:
        :func:`importlib.metadata.version` 给出的版本串，原样返回、不做规范化。

    Raises:
        LerobotNotInstalled: 环境里没有 ``lerobot`` 分发。
    """
    try:
        return importlib.metadata.version("lerobot")
    except importlib.metadata.PackageNotFoundError as exc:
        raise LerobotNotInstalled(
            f"环境里找不到 lerobot 分发（importlib.metadata 报 PackageNotFoundError）："
            f"需要 {LEROBOT_REQUIREMENT}，请先按 lerobot 上游 README 安装"
        ) from exc


def assert_lerobot_version_supported(installed: str | None = None) -> str:
    """校验 ``lerobot`` 版本受支持，不受支持则抛错。

    Args:
        installed: 显式版本串；传 ``None`` 时读环境里的包元数据。

    Returns:
        通过校验的版本串（去掉首尾空白）。

    Raises:
        LerobotNotInstalled: 未传 ``installed`` 且环境里没有 ``lerobot``。
        InvalidVersionString: 版本串无法解析。
        LerobotVersionUnsupported: 版本越界。
    """
    if installed is None:
        installed = installed_lerobot_version()
    return check_lerobot_version(installed)
