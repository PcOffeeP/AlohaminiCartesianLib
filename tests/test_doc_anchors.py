"""文档间锚点引用的机器守卫（离线，只读文本，不联网、不连设备）。

**守卫什么**：仓库内 markdown 文档之间要用锚点互相跳转，锚点一律写成显式 HTML
``<a id="gate-d"></a>``。本文件把每一条锚点引用逐条判定：目标文档能解析到、目标文档里
存在同名 ``<a id>``；同时校验 id 在全扫描范围内唯一、形态合法、必需锚点一个不少。
文档结构重组（改文件名、挪小节）会让锚点静默失效，读者点开落到文档顶部或 404；
本守卫让这种失效在跑测试时就红。

**为什么用显式 ``<a id>`` 而不是 ``#标题`` slug**：中文标题的 slug 依赖渲染器的转义规则
（GitHub 与本地 markdown 预览对全角标点、空格的处理不一致），改标题措辞也会悄悄换 slug。
显式 id 与渲染器无关、与标题措辞解耦，跨文档引用因此可以稳定。

**抽取口径**（``check_documents`` 是对任意「路径 → 全文」集合调用的纯函数，仓库真实文件
只在 ``repository_documents`` 里做一层薄封装，各用例再各自调用）：

- 扫描范围：仓库内 markdown —— ``docs/**/*.md``、根 ``README.md``、``TODO.md``、``AGENTS.md``。
  ``.venv`` / ``.venvs`` / ``.git`` 不在上述路径下，自然不扫。
- 锚点引用两种写法：markdown 行内链接目标里带 fragment（``[文字](../reference/gates.md#gate-d)``、
  ``[文字](#gate-d)``），以及反引号代码段里的路径式引用（``docs/runbook/4-experiments.md#exp-e6``，
  路径必须以 ``.md`` 结尾）。裸 ``#标题`` 片段一律不算锚点，本仓库约定只用 ``<a id>``。
- 锚点定义：``<a id="xxx"></a>``，也接受单引号、无闭合标签的 ``<a id="xxx">``、自闭合
  ``<a id="xxx" />``，以及 id 之后还带其它属性的写法。
- 围栏代码块（三反引号或 ``~~~``）内的文字不抽取：那是示例文本，不是活链接。
- 外链目标（``http://`` / ``https://`` / ``file://`` / ``mailto:`` 等带 scheme 的）不校验。
- 引用目标必须是扫描范围内的 markdown 文档；指向扫描范围外的文件却带 fragment 时按
  「解析不到目标文件」报违规。

**``MIN_ANCHOR_REFERENCES`` 口径**：与 ``tests/test_doc_line_references.py`` 的
``MIN_REFERENCES`` 同理，是引用条数下限，防止抽取规则退化后「零引用也算全过」。文档重组
落地后实测：26 份被扫描的 markdown 里共抽取 **454** 条锚点引用（0 条违规），其中 **70** 条
来自过渡期对照表 ``docs/field-ops/reference/mapping.md``（见 ``TRANSITIONAL_DOCUMENTS``），
按 ``TRANSITIONAL_DOCUMENTS`` 扣除后计入下限的是 **384** 条，按 5% 留余量 19 条，下限取
**365**。``MIN_ANCHORS`` 取 ``REQUIRED_ANCHORS`` 的条数（47），是定义条数的天然下限；
实测定义了 52 个锚点。

**过渡期文件不计入下限**：``docs/field-ops/reference/mapping.md`` 是明确标注「过渡期，一轮
现场后删」的对照表，按计划删它是预期动作。它上面的引用照样逐条判定（目标能解析、``<a id>``
存在、id 唯一、形态合法都在扫描范围内），只是那 70 条引用不计入 ``MIN_ANCHOR_REFERENCES``
——所以删除它不会触发下限，也就不会逼人「为过测试而下调常量」。这与
``tests/test_doc_line_references.py`` 对同一份过渡表的处理一致：那里它 0 条引用，删它不触及
``MIN_REFERENCES``。注意：**不要**把 ``mapping.md`` 从扫描范围整体排除——那样它的链接坏了
也查不出来。

**``REQUIRED_ANCHORS`` 是「必须存在」的清单，不是「存在的全量」**：实测还有 5 个锚点被定义
但不在清单里（``csv-rtt-18`` / ``csv-tracking-14`` / ``exit-code-2`` / ``exit-code-9`` /
``sec-9-gates``）。其中 ``sec-9-gates`` 确实被引用 4 次（``reference/troubleshooting.md`` 1 次、
``reference/mapping.md`` 3 次）。它们进不进 ``REQUIRED_ANCHORS`` 是文档口径问题，本守卫不因
「被引用过」自动收编，也不因此放宽判定：引用能否解析已由「目标文档里必须有同名 ``<a id>``」
这条判据覆盖。
"""

from __future__ import annotations

import posixpath
import re
from pathlib import Path
from typing import NamedTuple

REPO_ROOT = Path(__file__).resolve().parents[1]

# 扫描范围：``docs/`` 下全部 markdown（含子目录）＋ 三份根文档。
DOCUMENT_DIRECTORY = "docs"
ROOT_DOCUMENTS = ("README.md", "TODO.md", "AGENTS.md")

# 过渡期文档：仍在扫描范围内、上面的引用照旧逐条判定，但引用条数不计入
# ``MIN_ANCHOR_REFERENCES``。作用是让「按计划删掉过渡表」不用动下限常量。
# 键是仓库相对的归一化 posix 路径（与 ``repository_documents`` 的键同形）。
TRANSITIONAL_DOCUMENTS = frozenset({"docs/field-ops/reference/mapping.md"})

# 重组方案的对外跳转目标：这些 id 必须在扫描范围内被定义。定义在哪个文件由文档作者决定，
# 本守卫只要求存在 —— 用来防「迁移时漏搬小节」这类静默丢失。
REQUIRED_ANCHORS = (
    "sec-0",
    "sec-1",
    "sec-2",
    "sec-3",
    "sec-4-1",
    "sec-4-1b-lerobot",
    "sec-4-1b",
    "sec-4-1c",
    "sec-4-2",
    "sec-4-3",
    "sec-4-4",
    "sec-5-1",
    "sec-5-2",
    "sec-5-3",
    "sec-8-1",
    "sec-8-2",
    "sec-8-3",
    "sec-9",
    "sec-10",
    "sec-10-5",
    "sec-10-6",
    "sec-10-7",
    "sec-10-8",
    "sec-11",
    "sec-12",
    "sec-13",
    "host-start",
    "exit-codes",
    "param-capture",
    "param-measure",
    "sidecar-csv",
    "gate-a",
    "gate-b",
    "gate-c",
    "gate-d",
    "gate-e",
    "gate-f",
    "exp-e1",
    "exp-e2",
    "exp-e3",
    "exp-e4",
    "exp-e5",
    "exp-e6",
    "exp-e7",
    "exp-e8",
    "appendix-a",
    "appendix-b",
)

# 引用条数下限。文档重组落地后 26 份被扫描文档共实测 454 条（0 条违规），其中 70 条出自过渡期
# 对照表 ``docs/field-ops/reference/mapping.md``（见 ``TRANSITIONAL_DOCUMENTS``，删它是计划内动作，
# 故不计入下限）：扣除后 384 条，留 5% 余量 19 条，取 365。
# 抽取规则退化成「几乎抓不到引用」（正则写错、扫描范围被缩小）时会立刻掉到下限以下：
# 例如漏扫 runbook/4-experiments.md（137 条）→ 247，漏扫 runbook/1-prepare.md（75 条）→ 309。
MIN_ANCHOR_REFERENCES = 365

# 定义条数下限：``REQUIRED_ANCHORS`` 全定义到就是 47，故直接取清单长度。
MIN_ANCHORS = len(REQUIRED_ANCHORS)

# ``<a id="xxx">``：单双引号都收，id 后面允许其它属性，允许自闭合与无闭合标签。
_ANCHOR_DEFINITION = re.compile(r"<a\s+id\s*=\s*[\"'](?P<id>[^\"']*)[\"'][^>]*>")
_VALID_ANCHOR_ID = re.compile(r"[a-z0-9-]+")
_FENCE = re.compile(r"^\s{0,3}(?P<fence>`{3,}|~{3,})")
_MARKDOWN_LINK = re.compile(r"(?<!!)\[[^\[\]]*\]\(\s*<?(?P<target>[^()\s<>]+)>?\s*\)")
_CODE_SPAN = re.compile(r"`(?P<content>[^`\n]+)`")
_CODE_SPAN_REFERENCE = re.compile(r"(?P<path>[^#\s`]+\.md)(?P<fragment>#[^#\s`]+)")
_EXTERNAL_TARGET = re.compile(r"^(?:[A-Za-z][A-Za-z0-9+.\-]*:|//)")


class AnchorReference(NamedTuple):
    """一条锚点引用：引用位置、引用原文、目标路径（空表示同文件 ``#anchor``）与 id。"""

    document: str
    document_line: int
    raw: str
    path: str
    anchor: str


class Report(NamedTuple):
    """一次检查的结果：违规文案、实际检查的引用条数、计入下限的引用条数、id 到定义位置的映射。"""

    violations: list[str]
    checked: int
    counted: int
    anchors: dict[str, list[str]]


def _normalize_path(name: str) -> str:
    """把文档路径归一成仓库相对的 posix 形式（折叠 ``.`` / ``..``，去掉前导斜杠）。"""
    normalized = posixpath.normpath(name.replace("\\", "/").lstrip("/"))
    return "" if normalized == "." else normalized


def _iter_lines(text: str) -> list[tuple[int, str]]:
    """返回围栏代码块之外的行，行号从 1 起且与编辑器一致。"""
    lines: list[tuple[int, str]] = []
    fence = ""
    for number, line in enumerate(text.splitlines(), start=1):
        marker = _FENCE.match(line)
        if marker is not None:
            token = marker.group("fence")
            if not fence:
                fence = token
            elif token[0] == fence[0] and len(token) >= len(fence):
                fence = ""
            continue
        if not fence:
            lines.append((number, line))
    return lines


def _iter_references(document: str, text: str) -> list[AnchorReference]:
    """按抽取口径从一个文档的全文里取出全部锚点引用。"""
    references: list[AnchorReference] = []
    for number, line in _iter_lines(text):
        spans = [match.span() for match in _CODE_SPAN.finditer(line)]
        for match in _MARKDOWN_LINK.finditer(line):
            if any(start <= match.start() < end for start, end in spans):
                continue
            target = match.group("target")
            if _EXTERNAL_TARGET.match(target):
                continue
            path, _, anchor = target.partition("#")
            if not anchor:
                continue
            references.append(AnchorReference(document, number, match.group(0), path, anchor))
        for match in _CODE_SPAN.finditer(line):
            found = _CODE_SPAN_REFERENCE.fullmatch(match.group("content"))
            if found is None:
                continue
            references.append(
                AnchorReference(
                    document,
                    number,
                    match.group(0),
                    found.group("path"),
                    found.group("fragment").lstrip("#"),
                )
            )
    return references


def _resolve(path: str, document: str, documents: dict[str, str]) -> str | None:
    """把引用里的路径解析成扫描范围内的文档；先按引用方目录，再按仓库根。"""
    directory = posixpath.dirname(_normalize_path(document))
    candidates = [posixpath.join(directory, path) if directory else path, path]
    for candidate in candidates:
        key = _normalize_path(candidate)
        if key in documents:
            return key
    return None


def check_documents(documents: dict[str, str]) -> Report:
    """检查给定的「文档相对路径 → 全文」集合，返回违规清单、引用条数与锚点表。"""
    texts = {_normalize_path(name): text for name, text in documents.items()}
    anchors: dict[str, list[str]] = {}
    defined: dict[str, set[str]] = {name: set() for name in texts}
    violations: list[str] = []

    for document, text in texts.items():
        for number, line in _iter_lines(text):
            for anchor_id in _ANCHOR_DEFINITION.findall(line):
                location = f"{document}:{number}"
                anchors.setdefault(anchor_id, []).append(location)
                defined[document].add(anchor_id)
                if not _VALID_ANCHOR_ID.fullmatch(anchor_id):
                    violations.append(
                        f"{location} 的 <a id={anchor_id!r}>：id 只允许小写字母、数字与连字符"
                        "（[a-z0-9-]+）"
                    )

    for anchor_id in sorted(anchors):
        locations = anchors[anchor_id]
        if len(locations) > 1:
            violations.append(
                f"锚点 id {anchor_id!r} 被重复定义 {len(locations)} 次：{'、'.join(locations)}；"
                "同一 id 在全扫描范围内只允许定义一次，否则跨文件链接指向哪个定义不确定"
            )

    checked = 0
    counted = 0
    for document, text in texts.items():
        for reference in _iter_references(document, text):
            checked += 1
            if document not in TRANSITIONAL_DOCUMENTS:
                counted += 1
            where = f"{reference.document}:{reference.document_line} {reference.raw}"
            target = document if not reference.path else _resolve(reference.path, document, texts)
            if target is None:
                violations.append(
                    f"{where}：解析不到引用目标文件 {reference.path!r}"
                    "（先按引用方文件所在目录、再按仓库根解析，且目标必须在扫描范围内）"
                )
                continue
            if reference.anchor not in defined[target]:
                violations.append(f"{where}：{target} 里没有 <a id={reference.anchor!r}>")
    return Report(violations, checked, counted, anchors)


def repository_documents() -> dict[str, str]:
    """读取仓库内全部被扫描的 markdown 文档（``docs/**/*.md`` ＋ 三份根文档）。"""
    paths = [REPO_ROOT / name for name in ROOT_DOCUMENTS]
    paths.extend(sorted((REPO_ROOT / DOCUMENT_DIRECTORY).rglob("*.md")))
    documents: dict[str, str] = {}
    for path in paths:
        if not path.is_file():
            continue
        documents[path.relative_to(REPO_ROOT).as_posix()] = path.read_text(encoding="utf-8")
    return documents


def test_repository_documents_have_no_broken_anchor_references() -> None:
    """仓库内每条锚点引用都要解析到真实文档里的真实 ``<a id>``，且 id 唯一、形态合法。"""
    report = check_documents(repository_documents())
    assert report.violations == [], "锚点引用已失效：\n" + "\n".join(report.violations)


def test_required_anchors_are_defined() -> None:
    """``REQUIRED_ANCHORS`` 里的每个 id 都必须在扫描范围内被定义。"""
    anchors = check_documents(repository_documents()).anchors
    missing = [anchor_id for anchor_id in REQUIRED_ANCHORS if anchor_id not in anchors]
    assert missing == [], (
        f"缺少必需锚点，共 {len(missing)} 个（清单 {len(REQUIRED_ANCHORS)} 个）："
        + "、".join(missing)
    )


def test_anchor_reference_count_meets_the_floor() -> None:
    """计入下限的锚点引用条数不得低于下限，防止抽取退化后「零引用也算过」。

    过渡期文档（``TRANSITIONAL_DOCUMENTS``）的引用不计入，删除它不触发本判据。
    """
    counted = check_documents(repository_documents()).counted
    assert counted >= MIN_ANCHOR_REFERENCES, (
        f"只计入 {counted} 条锚点引用，低于下限 {MIN_ANCHOR_REFERENCES}"
    )


def test_defined_anchor_count_meets_the_floor() -> None:
    """定义条数不得低于 ``MIN_ANCHORS``，防止锚点被成批删掉后其余用例仍绿。"""
    anchors = check_documents(repository_documents()).anchors
    assert len(anchors) >= MIN_ANCHORS, f"只定义到 {len(anchors)} 个锚点，低于下限 {MIN_ANCHORS}"


def test_checker_flags_reference_to_missing_anchor() -> None:
    """合成一条指向不存在锚点的引用，检查器要报缺锚点违规并点名该 id 与引用位置。"""
    documents = {"a.md": '<a id="other"></a>\n', "b.md": "见 [x](a.md#missing)。\n"}

    report = check_documents(documents)

    assert len(report.violations) == 1, report.violations
    assert "missing" in report.violations[0]
    assert "b.md:1" in report.violations[0]
    assert report.checked == 1


def test_checker_flags_duplicate_anchor_definitions() -> None:
    """同一个 id 定义两次必须报违规，且文案点名 id 与两处位置。"""
    documents = {"a.md": '<a id="gate-d"></a>\n', "b.md": '\n<a id="gate-d"></a>\n'}

    report = check_documents(documents)

    assert len(report.violations) == 1, report.violations
    message = report.violations[0]
    assert "gate-d" in message
    assert "a.md:1" in message
    assert "b.md:2" in message


def test_checker_flags_malformed_anchor_ids() -> None:
    """锚点 id 出现小写字母、数字、连字符之外的字符时要报违规。"""
    documents = {"a.md": '<a id="Gate_D"></a>\n'}

    report = check_documents(documents)

    assert len(report.violations) == 1, report.violations
    assert "Gate_D" in report.violations[0]


def test_checker_flags_unresolvable_reference_target() -> None:
    """引用目标文件既不在引用方目录也不在仓库根时必须报违规，不静默放过。"""
    documents = {"docs/runbook/a.md": "见 [x](missing.md#gate-d)。\n"}

    report = check_documents(documents)

    assert len(report.violations) == 1, report.violations
    assert "missing.md" in report.violations[0]
    assert report.checked == 1


def test_checker_accepts_matching_references() -> None:
    """引用与定义匹配时必须通过：同文件 ``#anchor``、相对路径跳转、反引号路径式引用都算。"""
    documents = {
        "docs/runbook/gates.md": (
            '<a id="gate-d"></a>\n'
            "同文件 [x](#gate-d)，另一份 [y](../reference/params.md#param-capture)，\n"
            "代码段 `docs/runbook/experiments.md#exp-e6`。\n"
        ),
        "docs/reference/params.md": '<a id="param-capture"></a>\n',
        "docs/runbook/experiments.md": '<a id="exp-e6"></a>\n',
    }

    report = check_documents(documents)

    assert report.violations == []
    assert report.checked == 3
    assert report.anchors["gate-d"] == ["docs/runbook/gates.md:1"]


def test_checker_excludes_transitional_document_references_from_floor() -> None:
    """过渡期文档的引用仍被检查（``checked``），但不计入下限（``counted``）。"""
    documents = {
        "docs/field-ops/reference/mapping.md": "见 [x](../runbook/gates.md#gate-d)。\n",
        "docs/field-ops/runbook/gates.md": '<a id="gate-d"></a>\n见 [y](#gate-d)。\n',
    }

    report = check_documents(documents)

    assert report.violations == []
    assert report.checked == 2
    assert report.counted == 1


def test_checker_still_judges_references_inside_transitional_documents() -> None:
    """过渡期文档里的坏引用照样报违规：只是不计入下限，不是不判定。"""
    documents = {
        "docs/field-ops/reference/mapping.md": "见 [x](gates.md#missing)。\n",
        "docs/field-ops/reference/gates.md": '<a id="gate-d"></a>\n',
    }

    report = check_documents(documents)

    assert len(report.violations) == 1, report.violations
    assert "missing" in report.violations[0]
    assert report.counted == 0


def test_checker_ignores_fenced_code_blocks() -> None:
    """围栏代码块里的锚点与引用都不抽取：示例文本不是活链接，也不算定义。"""
    documents = {
        "a.md": '```markdown\n<a id="gate-d"></a>\n[示例](b.md#nope)\n```\n',
        "b.md": "",
    }

    report = check_documents(documents)

    assert report.violations == []
    assert report.checked == 0
    assert report.anchors == {}
