"""``alohamini_cartesian.config.overlay`` 的契约测试（TDD 红阶段）。

当前 ``src/alohamini_cartesian/config/overlay.py`` 里所有函数体都是
``NotImplementedError``，本文件预期全红。所有文件读写都发生在 ``tmp_path`` 内，
不读真实 ``/dev``（仅用 ``Path.exists()`` 判断一个绝对路径是否存在）。
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from alohamini_cartesian.config import overlay as overlay_module
from alohamini_cartesian.config.overlay import (
    ENV_PREFIX,
    AlohaMiniSettings,
    ConfigError,
    load_settings,
)

MISSING_DEVICE = "/dev/does-not-exist-xyz"


def _write(tmp_path: Path, name: str, text: str) -> Path:
    """在 tmp_path 下写一个配置文件并返回其路径。"""
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return path


def test_module_constants_match_documented_contract() -> None:
    """环境变量前缀是对外契约的一部分。"""
    assert ENV_PREFIX == "ALOHAMINI_"


def test_defaults_are_returned_when_nothing_is_provided() -> None:
    """C1：不给任何来源时逐字段返回 dataclass 默认值。"""
    settings = load_settings(env={})

    assert isinstance(settings, AlohaMiniSettings)
    assert settings.robot_model == "alohamini1"
    assert settings.left_port == "/dev/ttyUSB0"
    assert settings.right_port == "/dev/ttyUSB1"
    assert settings.cameras == {}
    assert settings.observation_request_window == 1
    assert settings.watchdog_timeout_ms == 500
    assert settings.max_loop_freq_hz == 50


def test_template_overrides_defaults_and_leaves_others_untouched(tmp_path: Path) -> None:
    """C2：template 只改它提到的键，未提到的字段仍为默认值。"""
    template = _write(
        tmp_path,
        "template.yaml",
        "robot_model: alohamini2\nwatchdog_timeout_ms: 800\n",
    )

    settings = load_settings(template=template, env={})

    assert settings.robot_model == "alohamini2"
    assert settings.watchdog_timeout_ms == 800
    assert settings.left_port == "/dev/ttyUSB0"
    assert settings.right_port == "/dev/ttyUSB1"
    assert settings.cameras == {}
    assert settings.observation_request_window == 1
    assert settings.max_loop_freq_hz == 50


def test_site_overrides_template_and_keeps_template_only_keys(tmp_path: Path) -> None:
    """C3：site 覆盖同名键，template 独有的键继续生效。"""
    template = _write(
        tmp_path,
        "template.yaml",
        "left_port: /dev/ttyUSB8\nwatchdog_timeout_ms: 800\n",
    )
    site = _write(tmp_path, "site.yaml", "left_port: /dev/ttyUSB9\n")

    settings = load_settings(template=template, site=site, env={})

    assert settings.left_port == "/dev/ttyUSB9"
    assert settings.watchdog_timeout_ms == 800


def test_env_overrides_site(tmp_path: Path) -> None:
    """C4：环境变量优先级最高，压过 site 文件。"""
    site = _write(tmp_path, "site.yaml", "robot_model: alohamini2\n")

    settings = load_settings(site=site, env={"ALOHAMINI_ROBOT_MODEL": "alohamini1"})

    assert settings.robot_model == "alohamini1"


def test_env_only_run_overrides_defaults() -> None:
    """C4 补充：没有配置文件时环境变量仍生效。"""
    settings = load_settings(env={"ALOHAMINI_LEFT_PORT": "/dev/ttyUSB7"})

    assert settings.left_port == "/dev/ttyUSB7"
    assert settings.right_port == "/dev/ttyUSB1"


def test_env_keys_are_case_insensitive() -> None:
    """C5：小写的环境变量名也要被识别，且值按整数解析。"""
    settings = load_settings(env={"alohamini_watchdog_timeout_ms": "1234"})

    assert settings.watchdog_timeout_ms == 1234
    assert isinstance(settings.watchdog_timeout_ms, int)


def test_unknown_template_key_raises_config_error(tmp_path: Path) -> None:
    """C6：template 里的未知键必须报错，信息含该键名。"""
    template = _write(tmp_path, "template.yaml", "nope: 1\n")

    with pytest.raises(ConfigError, match="nope"):
        load_settings(template=template, env={})


def test_unknown_env_key_raises_config_error() -> None:
    """C6：环境变量里的未知键必须报错，信息含该键名（大小写不敏感）。"""
    with pytest.raises(ConfigError, match=r"(?i)nope"):
        load_settings(env={"ALOHAMINI_NOPE": "1"})


def test_observation_request_window_below_one_raises_from_site(tmp_path: Path) -> None:
    """C7：site 文件里 window=0 越界，错误信息含字段名。"""
    site = _write(tmp_path, "site.yaml", "observation_request_window: 0\n")

    with pytest.raises(ConfigError, match="observation_request_window"):
        load_settings(site=site, env={})


def test_observation_request_window_below_one_raises_from_env() -> None:
    """C7：环境变量给出的 window=0 同样越界。"""
    with pytest.raises(ConfigError):
        load_settings(env={"ALOHAMINI_OBSERVATION_REQUEST_WINDOW": "0"})


def test_non_numeric_env_integer_raises_config_error() -> None:
    """C8：环境变量里的非数字整数值必须报错，不能静默转成 0。"""
    with pytest.raises(ConfigError):
        load_settings(env={"ALOHAMINI_WATCHDOG_TIMEOUT_MS": "abc"})


def test_string_integer_in_file_is_not_silently_coerced(tmp_path: Path) -> None:
    """C8：文件里的 "5" 是字符串不是整数，不做静默转换。"""
    template = _write(tmp_path, "template.yaml", 'observation_request_window: "5"\n')

    with pytest.raises(ConfigError):
        load_settings(template=template, env={})


def test_bool_is_not_accepted_as_integer_in_file(tmp_path: Path) -> None:
    """C8：YAML 的 true 是 bool，bool 不算 int，必须报错。"""
    template = _write(tmp_path, "template.yaml", "observation_request_window: true\n")

    with pytest.raises(ConfigError):
        load_settings(template=template, env={})


def test_cameras_from_env_merge_with_file_cameras(tmp_path: Path) -> None:
    """C9：env 里 cameras 的键名后缀小写化，并与文件里的 cameras 合并而非替换。"""
    template = _write(
        tmp_path,
        "template.yaml",
        'cameras:\n  wrist: "/dev/video0"\n',
    )

    settings = load_settings(template=template, env={"ALOHAMINI_CAMERAS_FRONT": "/dev/video2"})

    assert settings.cameras == {"wrist": "/dev/video0", "front": "/dev/video2"}
    assert settings.cameras["wrist"] == "/dev/video0"
    assert settings.cameras["front"] == "/dev/video2"


def test_check_devices_false_skips_device_existence(tmp_path: Path) -> None:
    """C10：默认不校验设备节点，不存在的路径也接受。"""
    template = _write(tmp_path, "template.yaml", f"left_port: {MISSING_DEVICE}\n")

    settings = load_settings(template=template, env={}, check_devices=False)

    assert settings.left_port == MISSING_DEVICE


def test_check_devices_true_raises_for_missing_device(tmp_path: Path) -> None:
    """C10：check_devices=True 时缺失设备报错，信息含该路径。"""
    template = _write(tmp_path, "template.yaml", f"left_port: {MISSING_DEVICE}\n")

    with pytest.raises(ConfigError, match=re.escape(MISSING_DEVICE)):
        load_settings(template=template, env={}, check_devices=True)


def test_check_devices_true_accepts_existing_files(tmp_path: Path) -> None:
    """C10：把两个端口指向 tmp_path 里真实存在的文件，校验应通过。"""
    left = tmp_path / "left.sock"
    right = tmp_path / "right.sock"
    left.write_text("", encoding="utf-8")
    right.write_text("", encoding="utf-8")
    template = _write(tmp_path, "template.yaml", f"left_port: {left}\nright_port: {right}\n")

    settings = load_settings(template=template, env={}, check_devices=True)

    assert settings.left_port == str(left)
    assert settings.right_port == str(right)


def test_missing_template_file_raises_config_error(tmp_path: Path) -> None:
    """C11：模板文件不存在时报错，信息含文件名。"""
    template = tmp_path / "missing.yaml"

    with pytest.raises(ConfigError, match="missing.yaml"):
        load_settings(template=template, env={})


def test_non_mapping_yaml_document_raises_config_error(tmp_path: Path) -> None:
    """C12：顶层是 list 的 YAML 不是配置，必须报错。"""
    template = _write(tmp_path, "template.yaml", "- a\n- b\n")

    with pytest.raises(ConfigError):
        load_settings(template=template, env={})


def test_json_file_is_supported_without_yaml(tmp_path: Path) -> None:
    """C13：.json 后缀走标准库解析，同样能生效。"""
    template = _write(tmp_path, "template.json", '{"robot_model": "alohamini2"}')

    settings = load_settings(template=template, env={})

    assert settings.robot_model == "alohamini2"


def test_settings_are_frozen() -> None:
    """配置是不可变 dataclass：外部不能就地改配置。"""
    settings = load_settings(env={})

    with pytest.raises(Exception):  # noqa: B017 - FrozenInstanceError 是 dataclasses 私有名
        settings.robot_model = "alohamini2"  # type: ignore[misc]


def test_cameras_dict_is_not_shared_between_calls() -> None:
    """每次加载都返回独立的 cameras dict，避免跨调用互相污染。"""
    first = load_settings(env={})
    second = load_settings(env={})

    assert first.cameras == second.cameras == {}
    assert first.cameras is not second.cameras


# --------------------------------------------------------------------------- #
# D. 审查回归补充
# --------------------------------------------------------------------------- #


def test_site_cameras_merge_with_template_cameras(tmp_path: Path) -> None:
    """相机配置按键合并：site 覆盖同名键，template 独有键保留。"""
    template = _write(
        tmp_path,
        "template.yaml",
        'cameras:\n  wrist: "/dev/video0"\n  top: "/dev/video1"\n',
    )
    site = _write(
        tmp_path,
        "site.yaml",
        'cameras:\n  wrist: "/dev/video9"\n  front: "/dev/video2"\n',
    )

    settings = load_settings(template=template, site=site, env={})

    assert settings.cameras == {
        "wrist": "/dev/video9",
        "top": "/dev/video1",
        "front": "/dev/video2",
    }


def test_missing_pyyaml_raises_informative_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """缺 PyYAML 时读 .yaml 必须抛 ConfigError，且信息同时点出 PyYAML 与文件后缀。"""
    monkeypatch.setattr(overlay_module, "yaml", None)
    template = _write(tmp_path, "template.yaml", "robot_model: alohamini2\n")

    with pytest.raises(ConfigError, match="PyYAML") as excinfo:
        load_settings(template=template, env={})

    message = str(excinfo.value)
    assert "PyYAML" in message
    assert "yaml" in message
    assert "template.yaml" in message


def test_json_fallback_still_works_without_pyyaml(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """没有 PyYAML 时 .json 是零依赖退路，必须照常加载成功。"""
    monkeypatch.setattr(overlay_module, "yaml", None)
    template = _write(tmp_path, "template.json", '{"robot_model": "alohamini2"}')

    settings = load_settings(template=template, env={})

    assert overlay_module.yaml is None
    assert settings.robot_model == "alohamini2"
