"""``tools/run_cartesian_move.py`` 的离线契约测试。"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from alohamini_cartesian.kinematics.joint_adapter import JOINT_ORDER

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = REPO_ROOT / "tools" / "run_cartesian_move.py"


def _load_tool() -> Any:
    spec = importlib.util.spec_from_file_location("run_cartesian_move", TOOL_PATH)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_offline_dry_run_9cm_succeeds_with_zero_sends_and_bounded_joint_hops(
    capsys: Any,
) -> None:
    """纯离线 --dry-run 沿 +Z 规划 9 cm：保持姿态、每步关节跳变 <= 2.0°、零发送。"""
    pytest.importorskip("placo")
    tool = _load_tool()
    code = tool.main(["--dry-run", "--dz", "0.09", "--json"])
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["ok"] is True
    assert payload["orientation_mode"] == "hold"
    assert payload["send_action_calls"] == 0
    assert payload["max_joint_hop_deg"] <= 2.0 + 1e-6
    assert payload["final_residual_m"] <= 0.005


def test_limits_only_without_signs_or_zeros_exits_15() -> None:
    """缺 --signs-json / --zeros-json 时必须按退出码 15 拒绝，绝不伪造标定。"""
    tool = _load_tool()
    code = tool.main(["--limits-only"])
    assert code == tool.EXIT_MISSING_CALIB


def test_motion_without_enable_motion_flag_exits_2(tmp_path: Path) -> None:
    """未显式传 --enable-motion 时拒绝真机运动并返回退出码 2。"""
    tool = _load_tool()
    signs_path = tmp_path / "signs.json"
    zeros_path = tmp_path / "zeros.json"
    signs_path.write_text(json.dumps(dict.fromkeys(JOINT_ORDER, 1.0)), encoding="utf-8")
    zeros_path.write_text(json.dumps(dict.fromkeys(JOINT_ORDER, 0.0)), encoding="utf-8")
    code = tool.main(["--signs-json", str(signs_path), "--zeros-json", str(zeros_path)])
    assert code == tool.EXIT_USAGE


def test_limits_only_reads_injected_client_with_zero_sends(tmp_path: Path, capsys: Any) -> None:
    """--limits-only 通过注入 client 读取有效限位窗口，严格零发送。"""
    tool = _load_tool()
    signs_path = tmp_path / "signs.json"
    zeros_path = tmp_path / "zeros.json"
    signs_path.write_text(json.dumps(dict.fromkeys(JOINT_ORDER, 1.0)), encoding="utf-8")
    zeros_path.write_text(json.dumps(dict.fromkeys(JOINT_ORDER, 0.0)), encoding="utf-8")

    class _FakeClient:
        def __init__(self) -> None:
            self.send_calls = 0
            self.latest_robot_metadata = {
                "schema_version": 2,
                "robot_model": "alohamini2pro",
                "motors": {
                    f"arm_left_{j}": {
                        "id": idx + 1,
                        "model": "sts3215",
                        "normalization": "range_m100_100",
                        "drive_mode": 0,
                        "range_min": 500,
                        "range_max": 3500,
                        "homing_offset": 0,
                        "resolution": 4096,
                    }
                    for idx, j in enumerate(JOINT_ORDER)
                },
            }

        def connect(self) -> None:
            pass

        def disconnect(self) -> None:
            pass

        def prime_observation_request_window(self, *, include_cameras: bool) -> None:
            pass

        def get_observation(self, *, include_cameras: bool = True) -> dict[str, Any]:
            assert include_cameras is False
            return {}

        def send_action(self, _action: Any) -> dict[str, Any]:
            self.send_calls += 1
            return {}

    fake = _FakeClient()
    code = tool.main(
        [
            "--limits-only",
            "--side",
            "left",
            "--signs-json",
            str(signs_path),
            "--zeros-json",
            str(zeros_path),
            "--site",
            str(REPO_ROOT / "configs" / "site.yaml"),
        ],
        client_factory=lambda _host, **_kw: fake,
    )
    assert code == 0
    assert fake.send_calls == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["robot_model"] == "alohamini2pro"
    assert len(payload["effective_limits_deg"]) == 6
    assert all(
        np.isfinite(lo) and np.isfinite(hi) and lo < hi
        for lo, hi in payload["effective_limits_deg"].values()
    )
