"""文档里 ``文件名:行号`` 引用的文件路径存在性守卫（离线，只读文本）。

本守卫扫描仓库核心文档中的 ``<path>:<行号>`` / ``<path>:<起>-<止>`` 引用，
只校验**目标文件能否解析到仓库内真实文件（或登记在外部白名单）**，不再锁死
绝对行号与空行——避免每次正常重构源码都触发大面积假红，同时继续守住：

- 引用指向的仓库内文件确实存在（删文件或改路径会立刻暴露）；
- 裸续引 ``:NNN`` 必须有同行可继承的文件名；
- 仓库外文件（vendor fork 同名文件）必须显式登记在 ``EXTERNAL_FILENAMES``；
- 同名碰撞文件（如 ``model_specs.py``）的解析侧明确锁定。
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import NamedTuple

REPO_ROOT = Path(__file__).resolve().parents[1]

DOCUMENTS = (
    "TODO.md",
    "docs/acceptance-baseline.md",
    "docs/field-ops/README.md",
    "docs/field-ops/next-mission.md",
    "docs/field-ops/completed-work.md",
    "docs/field-ops/runbook/1-prepare.md",
    "docs/field-ops/runbook/2-host.md",
    "docs/field-ops/runbook/3-health.md",
    "docs/field-ops/runbook/4-experiments.md",
    "docs/field-ops/runbook/5-wrapup.md",
    "docs/field-ops/reference/facts.md",
    "docs/field-ops/reference/gates.md",
    "docs/field-ops/reference/params.md",
    "docs/field-ops/reference/artifacts.md",
    "docs/field-ops/reference/troubleshooting.md",
    "docs/field-ops/reference/env-rebuild.md",
    "docs/field-ops/reference/mapping.md",
    "docs/field-results.md",
)

SEARCH_DIRS = ("src", "benchmarks", "tests", "docs", "configs", "examples", "tools")

# 按契约跳过的仓库外引用（vendor fork 一侧），按 basename 登记。
EXTERNAL_FILENAMES = frozenset(
    {
        "alohamini.py",
        "alohamini_client.py",
        "alohamini_host.py",
        "commands.md",
        "config_alohamini.py",
        "feetech.py",
        "model_specs.py",
        "motors_bus.py",
        "tables.py",
        "teleoperate_bi.py",
    }
)

# 防止抽取规则退化为「零引用也通过」的下限；随文档清理可合理调整。
MIN_REFERENCES = 10

_EXTENSION = r"(?:py|md|toml|yaml|yml|json|csv)"
_NAME = rf"[A-Za-z0-9_][A-Za-z0-9_./-]*\.{_EXTENSION}"

_TOKEN = re.compile(
    rf"(?P<name>{_NAME})(?::(?P<path_start>\d+)(?:-(?P<path_end>\d+))?)?"
    r"|(?<=[`\s\u3000（(、，,]):(?P<bare_start>\d+)(?:-(?P<bare_end>\d+))?(?![\w])"
)


class LineReference(NamedTuple):
    """一条引用：文档位置、引用原文、解析出的目标文件名与区间。"""

    document: str
    document_line: int
    raw: str
    name: str
    start: int
    end: int


class Report(NamedTuple):
    """一次检查的结果：违规文案、被跳过的仓库外文件名、实际检查条数。"""

    violations: list[str]
    skipped: set[str]
    checked: int


def _iter_references(document: str, text: str) -> list[LineReference]:
    """从文档全文抽取全部路径引用，裸续引继承同行最近的文件名。"""
    references: list[LineReference] = []
    for document_line, line in enumerate(text.splitlines(), start=1):
        inherited = ""
        for match in _TOKEN.finditer(line):
            name = match.group("name")
            if name is not None:
                inherited = name
                if match.group("path_start") is None:
                    continue
                start = int(match.group("path_start"))
                end = int(match.group("path_end") or start)
            else:
                start = int(match.group("bare_start"))
                end = int(match.group("bare_end") or start)
            references.append(
                LineReference(document, document_line, match.group(0), inherited, start, end)
            )
    return references


def _repository_files() -> list[Path]:
    """``SEARCH_DIRS`` 下的全部文件（不含 ``.venv`` / ``.venvs``）。"""
    return [
        candidate
        for directory in SEARCH_DIRS
        for candidate in (REPO_ROOT / directory).rglob("*")
        if candidate.is_file()
    ]


def _resolve_repo_path(name: str) -> tuple[Path | None, str]:
    """解析引用里的文件名到仓库内的文件；返回（路径, 失败原因）。"""
    direct = REPO_ROOT / name
    if direct.is_file():
        return direct, ""
    files = _repository_files()
    basename = name.rsplit("/", 1)[-1]
    if "/" in name:
        by_suffix = [
            candidate
            for candidate in files
            if candidate.relative_to(REPO_ROOT).as_posix().endswith(name)
        ]
        if len(by_suffix) == 1:
            return by_suffix[0], ""
        if len(by_suffix) > 1:
            return None, f"引用 {name!r} 在仓库内有 {len(by_suffix)} 处后缀匹配，无法唯一解析"
        return None, ""
    matches = [candidate for candidate in files if candidate.name == basename]
    if len(matches) == 1:
        return matches[0], ""
    if len(matches) > 1:
        return None, f"文件名 {basename!r} 在仓库内有 {len(matches)} 处同名文件，无法唯一解析"
    return None, ""


def check_references(document: str, text: str) -> Report:
    """检查文档文本里的路径引用是否全部指向真实文件或登记外部白名单。"""
    violations: list[str] = []
    skipped: set[str] = set()
    checked = 0
    for reference in _iter_references(document, text):
        where = f"{document}:{reference.document_line} {reference.raw}"
        if not reference.name:
            violations.append(f"{where}：裸续引没有可继承的文件名，请写成 文件名:行号")
            continue
        basename = reference.name.rsplit("/", 1)[-1]
        if "/" not in reference.name and basename in EXTERNAL_FILENAMES:
            skipped.add(basename)
            continue
        path, problem = _resolve_repo_path(reference.name)
        if path is None:
            if problem:
                violations.append(f"{where}：{problem}")
            elif basename in EXTERNAL_FILENAMES:
                skipped.add(basename)
            else:
                violations.append(
                    f"{where}：仓库内找不到 {reference.name!r}，也不在 EXTERNAL_FILENAMES 里"
                )
            continue
        checked += 1
        if reference.start <= 0 or reference.end <= 0:
            violations.append(f"{where}：行号必须为正整数")
        elif reference.start > reference.end:
            violations.append(f"{where}：区间起点晚于终点（{reference.start} > {reference.end}）")
    return Report(violations, skipped, checked)


def _check_document(document: str) -> Report:
    """读一个文档并检查它，供各用例复用。"""
    return check_references(document, (REPO_ROOT / document).read_text(encoding="utf-8"))


def test_documents_have_no_broken_file_references() -> None:
    """文档里的路径引用必须全部解析到仓库内真实文件或外部白名单。"""
    violations: list[str] = []
    for document in DOCUMENTS:
        violations.extend(_check_document(document).violations)
    assert violations == [], "文档文件路径引用已失效：\n" + "\n".join(violations)


def test_external_filenames_whitelist_has_no_unregistered_skips() -> None:
    """被跳过的仓库外文件名必须全部属于 ``EXTERNAL_FILENAMES``。"""
    skipped: set[str] = set()
    for document in DOCUMENTS:
        skipped |= _check_document(document).skipped
    assert skipped <= set(EXTERNAL_FILENAMES)


def test_basename_collisions_with_external_filenames_are_pinned() -> None:
    """仓库内与 ``EXTERNAL_FILENAMES`` 同名的 basename 集合必须受控。"""
    repository_basenames = {path.name for path in _repository_files()}
    collisions = repository_basenames & set(EXTERNAL_FILENAMES)
    assert collisions == {"model_specs.py"}, (
        f"仓库内与外置白名单同名的 basename 变了：{sorted(collisions)}"
    )


def test_reference_count_meets_the_floor() -> None:
    """实际被检查的条数不得低于下限，防止抽取规则退化后「零引用也算过」。"""
    checked = sum(_check_document(document).checked for document in DOCUMENTS)
    assert checked >= MIN_REFERENCES, f"只检查到 {checked} 条引用，低于下限 {MIN_REFERENCES}"


def test_checker_flags_synthetic_bad_references() -> None:
    """合成坏引用（不存在文件、区间倒挂、行号为零、无继承裸续引）必须被拦截。"""
    text = (
        "- `nonexistent_module.py:12` 不存在\n"
        "- `measure_latency.py:0` 行号为零\n"
        "- `controller.py:10-3` 区间倒挂\n"
        "- 继承不到的裸续引 `:123`\n"
    )
    report = check_references("synthetic.md", text)
    assert len(report.violations) == 4, report.violations
    joined = "\n".join(report.violations)
    assert "nonexistent_module.py:12" in joined
    assert "measure_latency.py:0" in joined
    assert "controller.py:10-3" in joined
    assert ":123" in joined


def test_checker_accepts_existing_file_without_pinning_exact_line() -> None:
    """指向仓库内存在文件的引用不再因行号变化误报。"""
    text = "- `measure_latency.py:999999` 仅核文件存在\n- `controller.py:1` 首行\n"
    report = check_references("synthetic.md", text)
    assert report.violations == []
    assert report.checked == 2
