"""配置加载与覆盖。

覆盖顺序（后者覆盖前者）：

    默认值 → template → site → 环境变量

未知键、类型不符、``observation_request_window < 1`` 都属于配置错误，直接
抛 :class:`ConfigError`，不做静默纠正。``check_devices=True`` 时还会校验
``/dev`` 下的设备节点存在。
"""

from __future__ import annotations

import json
import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:  # pragma: no cover - 仅在缺少 extra 时触发
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore[assignment]

ENV_PREFIX = "ALOHAMINI_"
CAMERAS_ENV_PREFIX = "CAMERAS_"

# 字段名 → 类型校验规则：文件与环境变量都按同一份清单识别。
INT_FIELDS = ("observation_request_window", "watchdog_timeout_ms", "max_loop_freq_hz")
STR_FIELDS = ("robot_model", "left_port", "right_port")
ENV_FIELD_BY_SUFFIX: dict[str, str] = {
    "ROBOT_MODEL": "robot_model",
    "LEFT_PORT": "left_port",
    "RIGHT_PORT": "right_port",
    "OBSERVATION_REQUEST_WINDOW": "observation_request_window",
    "WATCHDOG_TIMEOUT_MS": "watchdog_timeout_ms",
    "MAX_LOOP_FREQ_HZ": "max_loop_freq_hz",
}
SETTINGS_FIELDS = (*INT_FIELDS, *STR_FIELDS, "cameras")


class ConfigError(ValueError):
    """配置不可用：未知键、类型不符、取值越界或设备缺失。"""


@dataclass(frozen=True)
class AlohaMiniSettings:
    """一次运行所需的有效配置。"""

    robot_model: str = "alohamini1"
    left_port: str = "/dev/ttyUSB0"
    right_port: str = "/dev/ttyUSB1"
    cameras: dict[str, str] = field(default_factory=dict)
    observation_request_window: int = 1
    watchdog_timeout_ms: int = 500
    max_loop_freq_hz: int = 50


def load_settings(
    *,
    template: Path | None = None,
    site: Path | None = None,
    env: Mapping[str, str] | None = None,
    check_devices: bool = False,
) -> AlohaMiniSettings:
    """按 ``默认值 → template → site → 环境变量`` 的顺序装配配置。

    环境变量形如 ``ALOHAMINI_<大写下划线字段名>``，例如
    ``ALOHAMINI_LEFT_PORT``；键名匹配大小写不敏感。``cameras`` 可由文件提供，
    也可用 ``ALOHAMINI_CAMERAS_<相机名>`` 追加，环境变量与文件里的相机合并。
    ``cameras`` 在 template / site / 环境变量三层之间**按键合并**（同名键后者覆盖
    前者），其余字段保持整值覆盖语义。

    Args:
        template: 随包分发的默认配置模板路径，可为 ``None``。
        site: 现场覆盖配置路径，可为 ``None``。
        env: 环境变量映射，默认取 ``os.environ``。
        check_devices: 为 ``True`` 时校验串口设备节点存在。

    Returns:
        合并并校验后的配置。

    Raises:
        ConfigError: 文件不可读、后缀不支持、键未知、类型不符、取值越界或设备缺失。
    """
    environ = os.environ if env is None else env

    overrides: dict[str, Any] = {}
    for label, path in (("template", template), ("site", site)):
        if path is None:
            continue
        file_path = Path(path)
        values = _validated_values(_load_file(file_path), f"{label} 文件 {file_path.name}")
        _merge_layer(overrides, values)

    env_values, env_cameras, unknown_env = _parse_env(environ)
    if unknown_env:
        raise ConfigError(f"未知的环境变量键: {', '.join(sorted(unknown_env))}")
    _merge_layer(overrides, env_values)

    cameras: dict[str, str] = dict(overrides.get("cameras", {}))
    cameras.update(env_cameras)
    overrides["cameras"] = cameras

    settings = AlohaMiniSettings(**overrides)
    if settings.observation_request_window < 1:
        raise ConfigError(
            f"observation_request_window 必须 >= 1，拿到 {settings.observation_request_window}"
        )
    if settings.watchdog_timeout_ms < 1:
        raise ConfigError(f"watchdog_timeout_ms 必须 >= 1，拿到 {settings.watchdog_timeout_ms}")
    if settings.max_loop_freq_hz < 1:
        raise ConfigError(f"max_loop_freq_hz 必须 >= 1，拿到 {settings.max_loop_freq_hz}")

    if check_devices:
        _check_devices(settings)

    return settings


def _merge_layer(overrides: dict[str, Any], values: Mapping[str, Any]) -> None:
    """把一层配置并入 ``overrides``：``cameras`` 按键合并（后者覆盖同名键），其余字段整值覆盖。"""
    cameras: dict[str, str] = dict(overrides.get("cameras", {}))
    cameras.update(values.get("cameras", {}))
    overrides.update({key: value for key, value in values.items() if key != "cameras"})
    overrides["cameras"] = cameras


def _load_file(path: Path) -> dict[str, Any]:
    """读取配置文件：``.yaml``/``.yml`` 走 PyYAML，``.json`` 走标准库。

    Raises:
        ConfigError: 后缀不被支持，或缺少对应的解析依赖。
    """
    if not path.exists():
        raise ConfigError(f"配置文件不存在: {path}")

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"配置文件不可读: {path}") from exc
    if not text.strip():
        return {}

    suffix = path.suffix.lower()
    if suffix in (".yaml", ".yml"):
        if yaml is None:
            raise ConfigError(f"解析 {path.name} 需要 PyYAML，请安装 alohamini-cartesian[yaml]")
        try:
            data = yaml.safe_load(text)
        except yaml.YAMLError as exc:
            raise ConfigError(f"配置文件不是合法 YAML: {path}") from exc
    elif suffix == ".json":
        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ConfigError(f"配置文件不是合法 JSON: {path}") from exc
    else:
        raise ConfigError(f"不支持的配置文件后缀 {suffix!r}: {path.name}")

    if data is None:
        return {}
    if not isinstance(data, Mapping):
        raise ConfigError(f"配置文件顶层必须是映射，{path.name} 拿到 {type(data).__name__}")
    return dict(data)


def _validated_values(values: Mapping[str, Any], source: str) -> dict[str, Any]:
    """校验单个文件提供的键与类型，返回可合并的副本；不静默转换任何值。"""
    unknown = sorted(str(key) for key in values if key not in SETTINGS_FIELDS)
    if unknown:
        raise ConfigError(f"{source} 含未知键: {', '.join(unknown)}")

    validated: dict[str, Any] = {}
    for name, value in values.items():
        if name in INT_FIELDS:
            if isinstance(value, bool) or not isinstance(value, int):
                raise ConfigError(f"{source} 的 {name} 必须是整数，拿到 {value!r}")
            validated[name] = value
        elif name in STR_FIELDS:
            if not isinstance(value, str) or not value:
                raise ConfigError(f"{source} 的 {name} 必须是非空字符串，拿到 {value!r}")
            validated[name] = value
        else:  # cameras
            if not isinstance(value, Mapping):
                raise ConfigError(f"{source} 的 cameras 必须是映射，拿到 {type(value).__name__}")
            cameras: dict[str, str] = {}
            for camera_name, camera_value in value.items():
                if not isinstance(camera_name, str) or not isinstance(camera_value, str):
                    raise ConfigError(
                        f"{source} 的 cameras[{camera_name!r}] 必须是字符串，拿到 {camera_value!r}"
                    )
                cameras[camera_name] = camera_value
            validated[name] = cameras
    return validated


def _parse_env(
    environ: Mapping[str, str],
) -> tuple[dict[str, Any], dict[str, str], list[str]]:
    """解析环境变量覆盖：键名大小写不敏感；返回字段值、相机与未知键。"""
    values: dict[str, Any] = {}
    cameras: dict[str, str] = {}
    unknown: list[str] = []

    for raw_key, raw_value in environ.items():
        key = raw_key.upper()
        if not key.startswith(ENV_PREFIX):
            continue
        suffix = key[len(ENV_PREFIX) :]
        if suffix in ENV_FIELD_BY_SUFFIX:
            field_name = ENV_FIELD_BY_SUFFIX[suffix]
            if field_name in INT_FIELDS:
                values[field_name] = _parse_env_int(raw_key, field_name, raw_value)
            else:
                if not isinstance(raw_value, str) or not raw_value:
                    raise ConfigError(f"环境变量 {raw_key} 对应的 {field_name} 必须是非空字符串")
                values[field_name] = raw_value
        elif suffix.startswith(CAMERAS_ENV_PREFIX):
            camera_name = suffix[len(CAMERAS_ENV_PREFIX) :].lower()
            if not camera_name:
                raise ConfigError(f"环境变量 {raw_key} 缺少相机名")
            if not isinstance(raw_value, str):
                raise ConfigError(f"环境变量 {raw_key} 的值必须是字符串")
            cameras[camera_name] = raw_value
        else:
            unknown.append(raw_key)

    return values, cameras, unknown


def _parse_env_int(raw_key: str, field_name: str, value: Any) -> int:
    """把环境变量解析为十进制整数；不接受 float 字符串或 bool 字面量。"""
    if not isinstance(value, str):
        raise ConfigError(f"环境变量 {raw_key} 对应的 {field_name} 必须是字符串")
    try:
        return int(value, 10)
    except ValueError as exc:
        raise ConfigError(
            f"环境变量 {raw_key} 对应的 {field_name} 不是十进制整数: {value!r}"
        ) from exc


def _check_devices(settings: AlohaMiniSettings) -> None:
    """用 ``Path.exists()`` 确认串口与 ``/dev`` 相机节点存在，不读写设备内容。"""
    required = [settings.left_port, settings.right_port]
    required.extend(path for path in settings.cameras.values() if path.startswith("/dev/"))
    missing = [path for path in required if not Path(path).exists()]
    if missing:
        raise ConfigError(f"设备节点不存在: {', '.join(missing)}")
