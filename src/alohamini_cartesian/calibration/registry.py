"""标定代际登记表（``configs/calib-registry.yaml``）的读写。

登记表只回答一个问题：**这个 ``calib_id`` 以前见过吗**。见过了，说明这份标定不是新代际；
没见过，说明要么真换了标定，要么文件被改坏——判定交给调用方（``tools/calib_health.py``）。

文件形状固定为::

    schema_version: 1
    generations:
      - calib_id: <12 位十六进制>
        first_seen: <时间文本>
        source: <出处标记>
        file_sha256: <整份原件的 sha256>
        byte_size: <原件字节数>
        motors: {<电机名>: {id, drive_mode, homing_offset, range_min, range_max}}
        note: <人工说明或 null>
        acknowledged_by: <人工确认人或 null>

身份一律用 ``calib_id``；``file_sha256`` 只是溯源辅助（换个字节格式就变）。
登记按 ``calib_id`` 幂等：同一 id 再登记返回 ``False``，不追加、不覆盖首条。

依赖 PyYAML（与 ``config/overlay.py`` 同一套 try/import 兜底）；缺依赖时点名
``alohamini-cartesian[yaml]``，不引入新依赖。
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .identity import CANONICAL_FIELDS, is_tick_value

try:  # pragma: no cover - 仅在缺少 extra 时触发
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore[assignment]

# 登记表文件的 schema 版本；改动字段语义时才加。
SCHEMA_VERSION = 1
# 缺 PyYAML 时错误信息里点名的 extra。
YAML_EXTRA = "alohamini-cartesian[yaml]"


class RegistryError(RuntimeError):
    """登记表不可读或不可写：YAML 坏、schema 不符、字段取值不可用、落盘失败。"""


def _require_yaml() -> Any:
    """返回 PyYAML 模块；缺依赖时抛 :class:`RegistryError` 并点名 extra。"""
    if yaml is None:
        raise RegistryError(f"读写登记表需要 PyYAML，请安装 {YAML_EXTRA}")
    return yaml


@dataclass(frozen=True)
class GenerationRecord:
    """一条代际登记记录。

    ``first_seen`` 是**文本**，不是 ``datetime``：粒度由登记人决定（可以只到日，
    也可以到秒级 UTC 时点），不能假定它一定能解析成时间点。
    ``acknowledged_by`` 为 ``None`` 表示「这份代际的变动还没人确认过」，不是「没人看」。
    """

    calib_id: str
    first_seen: str
    source: str
    file_sha256: str
    byte_size: int
    motors: dict[str, dict[str, int]]
    note: str | None = None
    acknowledged_by: str | None = None

    def to_document(self) -> dict[str, Any]:
        """渲染成可 ``yaml.safe_dump`` 的映射，字段名与本类字段名一致。"""
        return {
            "calib_id": self.calib_id,
            "first_seen": self.first_seen,
            "source": self.source,
            "file_sha256": self.file_sha256,
            "byte_size": self.byte_size,
            "motors": {name: dict(entry) for name, entry in self.motors.items()},
            "note": self.note,
            "acknowledged_by": self.acknowledged_by,
        }

    @classmethod
    def from_document(cls, raw: Any) -> GenerationRecord:
        """从 YAML 读回的一条记录构造本类；形状不对抛 :class:`RegistryError`。"""
        if not isinstance(raw, Mapping):
            raise RegistryError(f"登记表里的记录不是映射，而是 {type(raw).__name__}")
        for field in ("calib_id", "first_seen", "source", "file_sha256"):
            value = raw.get(field)
            if not isinstance(value, str) or not value:
                raise RegistryError(f"登记表记录缺字符串字段 {field}：{value!r}")
        byte_size = raw.get("byte_size")
        if not is_tick_value(byte_size):
            raise RegistryError(f"登记表记录的 byte_size 不是整数：{byte_size!r}")
        motors = raw.get("motors")
        if not isinstance(motors, Mapping) or not motors:
            raise RegistryError(f"登记表记录 {raw.get('calib_id')!r} 的 motors 不是非空映射")
        canonical: dict[str, dict[str, int]] = {}
        for name, entry in motors.items():
            if not isinstance(entry, Mapping):
                raise RegistryError(f"登记表记录里的电机 {name} 条目不是映射")
            row: dict[str, int] = {}
            for field in CANONICAL_FIELDS:
                value = entry.get(field)
                if not is_tick_value(value):
                    raise RegistryError(f"登记表记录里的电机 {name} 字段 {field} 不是整数")
                row[field] = value
            canonical[str(name)] = row
        return cls(
            calib_id=raw["calib_id"],
            first_seen=raw["first_seen"],
            source=raw["source"],
            file_sha256=raw["file_sha256"],
            byte_size=byte_size,
            motors=canonical,
            note=None if raw.get("note") is None else str(raw["note"]),
            acknowledged_by=(
                None if raw.get("acknowledged_by") is None else str(raw["acknowledged_by"])
            ),
        )


class CalibrationRegistry:
    """一份登记表：有序记录列表 + 落盘路径。

    ``generations`` 的顺序就是登记顺序，:meth:`latest` 取最后一条——**不按
    ``calib_id`` 字典序排**，字典序没有时间含义。
    """

    def __init__(self, path: str | Path, generations: Iterable[GenerationRecord] = ()) -> None:
        """绑定落盘路径 ``path`` 与初始记录（首次登记不需要先手写文件）。"""
        self._path = Path(path)
        self._generations: list[GenerationRecord] = list(generations)

    @property
    def path(self) -> Path:
        """这份登记表的落盘路径。"""
        return self._path

    @property
    def generations(self) -> tuple[GenerationRecord, ...]:
        """全部记录，按登记顺序。"""
        return tuple(self._generations)

    @classmethod
    def load(cls, path: str | Path) -> CalibrationRegistry:
        """读一份登记表；文件不存在得到空表（不抛，首次登记不该要求先有文件）。"""
        module = _require_yaml()
        registry_path = Path(path)
        if not registry_path.exists():
            return cls(registry_path)
        try:
            text = registry_path.read_text(encoding="utf-8")
        except OSError as exc:
            raise RegistryError(f"登记表读不了：{registry_path}（{exc}）") from exc
        try:
            document = module.safe_load(text)
        except module.YAMLError as exc:
            raise RegistryError(f"登记表不是合法 YAML：{registry_path}（{exc}）") from exc
        if document is None:
            return cls(registry_path)
        if not isinstance(document, Mapping):
            raise RegistryError(f"登记表顶层不是映射：{registry_path}")
        version = document.get("schema_version")
        if version is not None and version != SCHEMA_VERSION:
            raise RegistryError(f"登记表 schema_version 是 {version!r}，本库只认 {SCHEMA_VERSION}")
        raw_generations = document.get("generations", [])
        if raw_generations is None:
            raw_generations = []
        if not isinstance(raw_generations, list):
            raise RegistryError(f"登记表的 generations 不是列表：{registry_path}")
        return cls(
            registry_path,
            (GenerationRecord.from_document(item) for item in raw_generations),
        )

    def find(self, calib_id: str) -> GenerationRecord | None:
        """按 ``calib_id`` 找记录；未登记返回 ``None``。"""
        for record in self._generations:
            if record.calib_id == calib_id:
                return record
        return None

    def latest(self) -> GenerationRecord | None:
        """最后登记的一条（按登记顺序）；空表返回 ``None``。"""
        return self._generations[-1] if self._generations else None

    def register(self, record: GenerationRecord) -> bool:
        """登记一条记录；同 ``calib_id`` 已存在时返回 ``False`` 且不追加、不覆盖。"""
        if self.find(record.calib_id) is not None:
            return False
        self._generations.append(record)
        return True

    def save(self) -> None:
        """把当前记录写回 :attr:`path`（YAML，UTF-8，不转义非 ASCII，不重排字段）。"""
        module = _require_yaml()
        document = {
            "schema_version": SCHEMA_VERSION,
            "generations": [record.to_document() for record in self._generations],
        }
        text = module.safe_dump(document, allow_unicode=True, sort_keys=False)
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            self._path.write_text(text, encoding="utf-8")
        except OSError as exc:
            raise RegistryError(f"登记表写不了：{self._path}（{exc}）") from exc
