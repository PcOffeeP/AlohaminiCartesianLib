"""统一标定 CLI ``tools/calib.py`` 的离线分发测试。"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
_CALIB_CLI = _REPO_ROOT / "tools" / "calib.py"
_FIXTURES = _REPO_ROOT / "tests" / "fixtures" / "calibration"
_OLD_FIXTURE = _FIXTURES / "AlohaMiniRobot.20261022.mac-snapshot.json"
_NEW_FIXTURE = _FIXTURES / "AlohaMiniRobot.20261110.pi-live.json"


def _run_cli(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(_CALIB_CLI), *args],
        capture_output=True,
        text=True,
        check=False,
        cwd=_REPO_ROOT,
    )


def test_calib_cli_help_lists_all_subcommands() -> None:
    """顶层 --help 退出 0 并列出 fetch / health / register 三个子命令。"""
    proc = _run_cli("--help")
    assert proc.returncode == 0
    for name in ("fetch", "health", "register"):
        assert name in proc.stdout


def test_calib_cli_unknown_subcommand_exits_two() -> None:
    """未知子命令退出 2 并在 stderr 说明。"""
    proc = _run_cli("unknown-cmd")
    assert proc.returncode == 2
    assert "unknown-cmd" in proc.stderr


def test_calib_cli_dispatches_health_register_and_fetch_offline(tmp_path: Path) -> None:
    """三条子命令均离线贯通，输出和退出码与独立脚本口径一致。"""
    empty_reg = tmp_path / "calib-registry.yaml"

    health = _run_cli(
        "health",
        "--calib-file",
        str(_NEW_FIXTURE),
        "--registry",
        str(empty_reg),
    )
    assert health.returncode == 1
    assert "CALIB_STATE: CALIB_NEW_UNREGISTERED" in health.stdout

    register = _run_cli(
        "register",
        "--from-file",
        str(_NEW_FIXTURE),
        "--registry",
        str(empty_reg),
    )
    assert register.returncode == 0
    assert "STRUCTURE: 合格" in register.stdout
    assert "dry-run" in register.stdout

    fetch_dry = _run_cli(
        "fetch",
        "--from-file",
        str(_OLD_FIXTURE),
        "--dry-run",
    )
    assert fetch_dry.returncode == 0
    assert "DRY-RUN:" in fetch_dry.stdout

    fetch = _run_cli(
        "fetch",
        "--from-file",
        str(_OLD_FIXTURE),
    )
    assert fetch.returncode == 1
    assert "CALIB_SOURCE: file" in fetch.stdout
