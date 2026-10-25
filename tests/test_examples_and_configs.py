"""``examples/minimal_dry_run.py`` 与 ``configs/*.yaml`` 的验收契约（TDD 红阶段）。

本文件在 ``coder`` 动手之前先落下契约：示例脚本与两份配置都还不存在，因此本文件预期
全红（子进程拉不到脚本、读文件抛 ``FileNotFoundError``）。

契约要点：

- A：示例脚本能离线跑通，退出码 0；stdout 里出现 ``0 次发送``（证明 ``send_action``
  一次都没被调用）、``steps`` 与 ``residual``。
- B：``configs/template.yaml`` 与 ``configs/site.yaml`` 的键集合等于
  ``overlay.SETTINGS_FIELDS``，取值等于冻结值，且 site 覆盖 template。
- C：``examples/`` 下任何 ``*.py`` 都不引用 ``lerobot``（本库只有
  ``src/alohamini_cartesian/adapter/protocol.py`` 可以引用它）。

全程离线：不连设备、不联网，唯一的外部动作是拉起一个子进程跑示例脚本。读真实配置时
一律传 ``check_devices=False``（本机没有这些 ``/dev`` 节点），并传 ``env={}`` 让断言不
受 ambient 环境变量干扰。
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest
import yaml

from alohamini_cartesian.config.overlay import SETTINGS_FIELDS, load_settings

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
    "observation_request_window": 1,
    "watchdog_timeout_ms": 500,
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


def test_loaded_window_satisfies_hard_validation() -> None:
    """契约 B：两份配置的 ``observation_request_window`` 都满足 ``>= 1`` 的硬校验。"""
    template_settings = load_settings(template=TEMPLATE_PATH, env={}, check_devices=False)
    site_settings = load_settings(site=SITE_PATH, env={}, check_devices=False)

    assert template_settings.observation_request_window >= 1
    assert site_settings.observation_request_window >= 1


# ---------------------------------------------------------------------------
# 契约 C：examples/ 静态断言
# ---------------------------------------------------------------------------


def test_examples_directory_contains_python_sources() -> None:
    """契约 C：``examples/`` 下至少有一个 ``*.py`` 源文件可供静态扫描。"""
    assert EXAMPLES_DIR.is_dir(), f"examples 目录尚未创建: {EXAMPLES_DIR}"
    assert sorted(EXAMPLES_DIR.glob("*.py")), f"examples 下没有任何 *.py: {EXAMPLES_DIR}"


def test_examples_do_not_import_lerobot() -> None:
    """契约 C：``examples/`` 下所有 ``*.py`` 都不含 ``import lerobot`` / ``from lerobot``。"""
    sources = sorted(EXAMPLES_DIR.glob("*.py"))
    assert sources, f"examples 下没有任何 *.py: {EXAMPLES_DIR}"

    forbidden = ("import lerobot", "from lerobot")
    offenders = [
        path.name
        for path in sources
        if any(marker in path.read_text(encoding="utf-8") for marker in forbidden)
    ]

    assert not offenders, f"这些示例引用了 lerobot（仅 adapter/protocol.py 允许）: {offenders}"
