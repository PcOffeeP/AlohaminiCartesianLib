#!/usr/bin/env python3
"""统一标定 CLI 入口：按子命令分发到 fetch / health / register。

用法::

    uv run --no-sync python tools/calib.py fetch    [--from-file PATH | --host HOST] ...
    uv run --no-sync python tools/calib.py health   [--calib-file PATH | --fingerprint PATH] ...
    uv run --no-sync python tools/calib.py register [--from-file PATH | --from-fingerprint PATH] ...

各子命令的行为、参数、stdout 机器可读行与退出码与对应独立脚本完全一致
（``tools/calib_fetch.py`` / ``tools/calib_health.py`` / ``tools/calib_register.py``），
三份旧脚本继续保留兼容；本入口作为推荐的统一命令行入口。
"""

from __future__ import annotations

import importlib.util
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from types import ModuleType

_TOOLS_DIR = Path(__file__).resolve().parent

_SUBCOMMANDS: dict[str, tuple[str, str]] = {
    "fetch": ("calib_fetch.py", "抓取标定文件、判定包络并可选登记（默认只读）"),
    "health": ("calib_health.py", "判定标定文件或 v2 指纹的代际状态与包络"),
    "register": ("calib_register.py", "把标定文件或 v2 指纹登记到代际登记表（默认 dry-run）"),
}


def _load_tool_main(filename: str) -> Callable[[Sequence[str] | None], int]:
    """按路径加载同目录下的子命令脚本并返回其 ``main`` 函数。"""
    path = _TOOLS_DIR / filename
    module_name = f"_alohamini_calib_tool_{path.stem}"
    existing = sys.modules.get(module_name)
    if isinstance(existing, ModuleType) and hasattr(existing, "main"):
        return existing.main
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:  # pragma: no cover - 文件缺失才会触发
        raise RuntimeError(f"无法加载标定子命令脚本：{path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module.main


def _usage_text() -> str:
    """渲染顶层帮助文本。"""
    lines = [
        "用法: calib.py <子命令> [子命令参数...]",
        "",
        "可用子命令:",
    ]
    for name, (_, summary) in _SUBCOMMANDS.items():
        lines.append(f"  {name:<10}{summary}")
    lines.append("")
    lines.append("查看各子命令完整参数与退出码: calib.py <子命令> --help")
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    """按第一个位置参数分派到对应标定工具的 ``main``。"""
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in ("-h", "--help"):
        print(_usage_text())
        return 0
    subcommand = args[0]
    entry = _SUBCOMMANDS.get(subcommand)
    if entry is None:
        print(f"未知子命令：{subcommand!r}\n\n{_usage_text()}", file=sys.stderr)
        return 2
    filename, _ = entry
    tool_main = _load_tool_main(filename)
    return int(tool_main(args[1:]))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
