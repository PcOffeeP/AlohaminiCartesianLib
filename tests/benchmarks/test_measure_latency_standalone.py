"""Standalone-execution regression test for ``benchmarks/alohamini_io/measure_latency.py``.

Verifies that ``measure_latency.py`` runs in an environment where
``alohamini_cartesian`` is NOT installed on ``sys.path`` and ``PYTHONPATH``
is unset (matching ``.venvs/alohamini-lerobot`` on field machines).
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPT = _REPO_ROOT / "benchmarks" / "alohamini_io" / "measure_latency.py"


def test_measure_latency_runs_standalone_without_pythonpath_or_installed_package() -> None:
    """Running ``measure_latency.py --help`` with repo ``src`` stripped exits 0."""
    # Strip repo src and editable hooks in a fresh child interpreter, then execute
    # measure_latency.py as __main__ so it must bootstrap sys.path itself.
    bootstrap = (
        "import runpy, sys; "
        f"repo_src = {str(_REPO_ROOT / 'src')!r}; "
        "sys.path[:] = [p for p in sys.path if p != repo_src and not p.endswith('/src')]; "
        "sys.meta_path[:] = ["
        "f for f in sys.meta_path if 'editable' not in type(f).__name__.lower()"
        "]; "
        f"sys.argv = [{str(_SCRIPT)!r}, '--help']; "
        f"runpy.run_path({str(_SCRIPT)!r}, run_name='__main__')"
    )
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    proc = subprocess.run(
        [sys.executable, "-c", bootstrap],
        capture_output=True,
        text=True,
        check=False,
        env=env,
        cwd=_REPO_ROOT,
    )
    assert proc.returncode == 0, f"stderr:\n{proc.stderr}\nstdout:\n{proc.stdout}"
    assert "--mode" in proc.stdout
