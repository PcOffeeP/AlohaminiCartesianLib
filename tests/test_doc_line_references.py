"""文档里 ``文件名:行号`` 引用的机器守卫（离线，只读文本）。

十个文档（``TODO.md``、``docs/acceptance-baseline.md``、``docs/field-results.md`` 与
``docs/field-ops/`` 下的 7 份现场手册子文档 ``00-opening.md`` … ``60-artifacts.md``）
用 ``文件名:行号`` 指代源码位置。
源码一改行号就漂，读者会点到不相关的代码。本文件把这类引用逐条按三条判定：
目标文件能解析到、行号落在 ``1..len(文件行数)`` 内（区间还要 ``起 <= 止``）、
目标行 ``str.strip()`` 后非空。

抽取口径：

- 完整引用 ``<path>:<行号>`` 与 ``<path>:<起>-<止>``。path 允许带目录，扩展名
  限 ``.py/.md/.toml/.yaml/.yml/.json/.csv``。
- 裸续引 ``:NNN``（前面没有路径）在同一行内继承该行最近出现的文件名；整行没有
  可继承的文件名按解析失败报违规，要求文档改成带文件名的写法。

解析顺序（见 ``_resolve_repo_path`` 与 ``check_references``）：

- 带目录的引用（``kinematics/model_specs.py``、``src/lerobot/robots/alohamini/x.py``）：
  先按仓库根原样解析；失败再按**路径后缀**匹配（仓库内某文件的相对路径以该引用结尾），
  唯一命中即用它。0 命中落到「仓库外 / 违规」判定，多命中报违规。
- 裸文件名（无目录，如 ``model_specs.py:80-87``）：**先查** ``EXTERNAL_FILENAMES``，命中
  即按仓库外跳过；未命中才做仓库内 basename 唯一匹配。这个顺序是必需的：
  ``model_specs.py`` 在本仓库（``src/alohamini_cartesian/kinematics/``）与 vendor fork
  （``src/lerobot/robots/alohamini/``）各有一份、行号不同，先查仓库会拿本仓库的行序去判
  vendor 的行号。要指本仓库那一份，写成带目录的 ``kinematics/model_specs.py``（后缀唯一）。

仓库外文件（vendor fork 里的同名文件）不判行号，但必须登记进 ``EXTERNAL_FILENAMES``：
实际跳过的文件名集合与该常量不一致也算红，避免新出现的仓库外引用被静默放过。
``MIN_REFERENCES`` 是条数下限，防止抽取规则退化后「零引用也算全过」。
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import NamedTuple

REPO_ROOT = Path(__file__).resolve().parents[1]

DOCUMENTS = (
    "TODO.md",
    "docs/acceptance-baseline.md",
    "docs/field-ops/00-opening.md",
    "docs/field-ops/10-health-baseline.md",
    "docs/field-ops/20-toolchain.md",
    "docs/field-ops/30-host-ops.md",
    "docs/field-ops/40-gates-params.md",
    "docs/field-ops/50-experiments.md",
    "docs/field-ops/60-artifacts.md",
    "docs/field-results.md",
)

# 裸文件名按 basename 唯一匹配时扫描的目录。已经列全仓库内的源码 / 文档 / 配置目录；
# 刻意不含 ``.venv`` / ``.venvs``：那里面有整份 vendor 运行时，命中它会拿错行序去判。
SEARCH_DIRS = ("src", "benchmarks", "tests", "docs", "configs", "examples", "tools")

# 按契约跳过的仓库外引用（vendor fork 一侧）。名字取 basename，因此
# ``src/lerobot/robots/alohamini/alohamini.py:516`` 与 ``alohamini.py:520``
# 归到同一条 ``alohamini.py``。多一个少一个都红。共 10 个名字。
# ``commands.md`` / ``teleoperate_bi.py`` 是 vendor fork 的 docs 与 examples（引用写作
# ``docs/alohamini/commands.md``、``examples/alohamini/teleoperate_bi.py``）。
# ``model_specs.py`` 两侧同名不同文件：vendor 是 ``src/lerobot/robots/alohamini/model_specs.py``，
# 本仓库是 ``src/alohamini_cartesian/kinematics/model_specs.py``。裸文件名一律按 vendor 侧跳过；
# 要指本仓库那一份必须写成带目录的 ``kinematics/model_specs.py``（后缀唯一命中）。
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

# 抽取规则退化（例如正则写错、只匹配到极少条目）时本用例会先红。
# 2026-11-08 实测：十个文档合计抽取 291 条引用，其中 179 条由机器逐条判行号
# （TODO 23 / acceptance-baseline 3 / field-ops 00-opening 0 / 10-health-baseline 14 /
#  20-toolchain 0 / 30-host-ops 0 / 40-gates-params 16 / 50-experiments 116 /
#  60-artifacts 2 / field-results 5），另有 112 条落在 10 个仓库外文件名上被跳过。
# 手册拆进 ``docs/field-ops/`` 不改变抽取条数：拆前那一版（单个 3295 行文件）单独是 198 条
# = 142 条仓库内 + 50 条仓库外 + 6 条指向旧文件自身（路径已不存，迁前判违规）；
# 拆后 7 份子文档仍是 198 条 = 148 条仓库内（142 + 原那 6 条指向新文件）+ 50 条仓库外。
# 外置跳过仍是 112 条（手册 50 + TODO 19 + field-results 43），抽取合计仍是 291 条。
# 旧注释里的 174 / 66 / 108 属更早一轮，已作废。
# 下限取 160：实测 179 已高于它 19 条（约 10.6%，与旧口径 60 对 66 的约 9% 同一比例），
# 再往上抬会让「文档正常删掉几条引用」也变红。
MIN_REFERENCES = 160

_EXTENSION = r"(?:py|md|toml|yaml|yml|json|csv)"
_NAME = rf"[A-Za-z0-9_][A-Za-z0-9_./-]*\.{_EXTENSION}"

# 裸续引的左边界取反引号与中文标点 / 空白，不含引号与字母数字：这样
# ``grep -E ":5555|:5556"``（端口）、``captured_utc=…T07:10:31``（钟点）、
# ``{reg:52s}``（格式化宽度）都不会被当成续引。
_TOKEN = re.compile(
    rf"(?P<name>{_NAME})(?::(?P<path_start>\d+)(?:-(?P<path_end>\d+))?)?"
    r"|(?<=[`\s\u3000（(、，,]):(?P<bare_start>\d+)(?:-(?P<bare_end>\d+))?(?![\w])"
)


class LineReference(NamedTuple):
    """一条引用：文档位置、引用原文、解析出的目标文件名与行号区间。"""

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


def _file_lines(path: Path) -> list[str]:
    """读文件的全部行（保留末尾空行计数口径：``splitlines`` 与编辑器行号一致）。"""
    return path.read_text(encoding="utf-8").splitlines()


def _iter_references(document: str, text: str) -> list[LineReference]:
    """按抽取口径从一个文档的全文里取出全部引用，裸续引继承同行最近的文件名。"""
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
    """解析引用里的文件名到仓库内的文件；返回（路径, 失败原因）。

    带目录的引用：先按仓库根原样解析，再按路径后缀（``SEARCH_DIRS`` 内某文件的仓库相对路径
    以该引用结尾，不含 ``.venv`` / ``.venvs``）唯一匹配。裸文件名：按 basename 在
    ``SEARCH_DIRS`` 里唯一匹配（外部白名单已在调用方先行处理）。
    解析不到时路径为 ``None``，由调用方决定是「仓库外跳过」还是「违规」。
    """
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
    """检查一段文档文本里的全部行号引用，返回违规清单与被跳过的文件名。"""
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
            # 裸文件名先看外部白名单：仓库内可能有同名文件（``model_specs.py``），
            # 先查仓库会拿本仓库的行序去判 vendor 的行号。
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
        lines = _file_lines(path)
        if not 1 <= reference.start <= len(lines):
            violations.append(f"{where}：起始行超出 {path.name} 的 1..{len(lines)}")
            continue
        if not 1 <= reference.end <= len(lines):
            violations.append(f"{where}：结束行超出 {path.name} 的 1..{len(lines)}")
            continue
        if reference.start > reference.end:
            violations.append(f"{where}：区间起点晚于终点（{reference.start} > {reference.end}）")
            continue
        if not lines[reference.start - 1].strip():
            violations.append(f"{where}：{path.name}:{reference.start} 是空行")
            continue
        if not lines[reference.end - 1].strip():
            violations.append(f"{where}：{path.name}:{reference.end} 是空行")
    return Report(violations, skipped, checked)


def _check_document(document: str) -> Report:
    """读一个文档并检查它，供各用例复用。"""
    return check_references(document, (REPO_ROOT / document).read_text(encoding="utf-8"))


def test_documents_have_no_broken_line_references() -> None:
    """十个文档里的行号引用必须全部落在真实、非空的行上。"""
    violations: list[str] = []
    for document in DOCUMENTS:
        violations.extend(_check_document(document).violations)
    assert violations == [], "行号引用已漂移：\n" + "\n".join(violations)


def test_external_filenames_whitelist_matches_what_is_skipped() -> None:
    """被跳过的仓库外文件名集合必须恰好等于 ``EXTERNAL_FILENAMES``。"""
    skipped: set[str] = set()
    for document in DOCUMENTS:
        skipped |= _check_document(document).skipped
    assert skipped == set(EXTERNAL_FILENAMES)


def test_basename_collisions_with_external_filenames_are_pinned() -> None:
    """仓库内与 ``EXTERNAL_FILENAMES`` 同名的 basename 集合必须恰好是写死的那一组。

    ``model_specs.py`` 是承重条目：仓库内（``src/alohamini_cartesian/kinematics/``）与 vendor
    fork 各有一份同名文件，「裸文件名先查白名单」这条规则决定裸 ``model_specs.py:NNN`` 按
    vendor 侧判。把它从白名单删掉时，其余用例仍会全绿（仓库内 basename 匹配同样唯一），
    所以删白名单本身要红；将来新增与仓库内文件同名的外部名也要来这里显式登记。
    """
    repository_basenames = {path.name for path in _repository_files()}
    collisions = repository_basenames & set(EXTERNAL_FILENAMES)
    assert collisions == {"model_specs.py"}, (
        f"仓库内与外置白名单同名的 basename 变了：{sorted(collisions)}；"
        "新增同名碰撞要显式更新本断言，删除承重条目会改变裸文件名引用的判定侧"
    )


def test_reference_count_meets_the_floor() -> None:
    """实际被检查的条数不得低于下限，防止抽取规则退化后「零引用也算过」。"""
    checked = sum(_check_document(document).checked for document in DOCUMENTS)
    assert checked >= MIN_REFERENCES, f"只检查到 {checked} 条引用，低于下限 {MIN_REFERENCES}"


def _first_blank_line(document: str) -> int:
    """取一个文档里第一个空行的行号，用来构造「指向空行」的坏引用。"""
    for number, line in enumerate(_file_lines(REPO_ROOT / document), start=1):
        if not line.strip():
            return number
    raise AssertionError(f"{document} 里没有空行")


def test_checker_flags_synthetic_bad_references() -> None:
    """合成一段坏文档，检查器要逐条报出违规，且文案里带文件名与行号。"""
    blank = _first_blank_line("benchmarks/alohamini_io/measure_latency.py")
    text = (
        f"- `measure_latency.py:999999` 越界\n"
        f"- `measure_latency.py:0` 行号为零\n"
        f"- `controller.py:10-3` 区间倒挂\n"
        f"- `measure_latency.py:{blank}` 指向空行\n"
        f"- 继承不到的裸续引 `:123`\n"
    )
    report = check_references("synthetic.md", text)
    assert len(report.violations) == 5, report.violations
    joined = "\n".join(report.violations)
    assert "measure_latency.py:999999" in joined
    assert "measure_latency.py:0" in joined
    assert "controller.py:10-3" in joined
    assert f"measure_latency.py:{blank}" in joined
    assert ":123" in joined


def test_checker_accepts_a_fixed_reference() -> None:
    """把坏引用改成合法行号后必须转为通过（正反对照）。"""
    good_line = _first_blank_line("benchmarks/alohamini_io/measure_latency.py") + 1
    text = f"- `measure_latency.py:{good_line}` 已对齐\n- `controller.py:1` 首行\n"
    report = check_references("synthetic.md", text)
    assert report.violations == []
    assert report.checked == 2
