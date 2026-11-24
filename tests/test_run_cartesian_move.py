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


def test_architecture_doc_dry_run_command_with_points_and_out_file(
    tmp_path: Path, capsys: Any
) -> None:
    """文档示例命令 ``--dry-run --points 7 --out <path>`` 离线直通并写 7 段 JSON 产物。"""
    pytest.importorskip("placo")
    tool = _load_tool()
    out_file = tmp_path / "move-dryrun.json"
    code = tool.main(["--dry-run", "--points", "7", "--out", str(out_file)])
    assert code == 0
    stdout = capsys.readouterr().out
    assert "[dry-run] ok=True points=7" in stdout
    assert f"[产物] {out_file}" in stdout
    payload = json.loads(out_file.read_text(encoding="utf-8"))
    assert payload["ok"] is True
    assert payload["points"] == 7
    assert len(payload["segments"]) == 7
    assert payload["send_action_calls"] == 0
    assert payload["max_joint_hop_deg"] <= 2.0 + 1e-6


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
    # Non-zero offset (+50 deg) pushes calibration upper bound (~225.6 deg) above
    # URDF upper bound (+179.9087 deg) so URDF ∩ calibration truncation is truly exercised.
    zeros_path.write_text(json.dumps(dict.fromkeys(JOINT_ORDER, 50.0)), encoding="utf-8")
    code = tool.main(["--signs-json", str(signs_path), "--zeros-json", str(zeros_path)])
    assert code == tool.EXIT_USAGE


@pytest.mark.parametrize(
    "bad_argv",
    [
        ["--dry-run", "--points", "0"],
        ["--dry-run", "--settled-max-joint-delta-deg", "nan"],
        ["--dry-run", "--settled-delta-deg", "-0.5"],
        ["--dry-run", "--waypoint-lead-deg", "0"],
        ["--dry-run", "--waypoint-lead-deg", "inf"],
        ["--dry-run", "--follow-error-tolerance-deg", "0"],
    ],
)
def test_invalid_numeric_flags_exit_usage(bad_argv: list[str]) -> None:
    """非有限或非正参数以及 --points < 1 必须按退出码 2 拒绝。"""
    tool = _load_tool()
    assert tool.main(bad_argv) == tool.EXIT_USAGE


def test_limits_only_primes_window_intersects_urdf_and_never_sends(
    tmp_path: Path, capsys: Any
) -> None:
    """--limits-only 先 prime(include_cameras=False)、与 URDF 限位求交、严格零发送。"""
    tool = _load_tool()
    signs_path = tmp_path / "signs.json"
    zeros_path = tmp_path / "zeros.json"
    signs_path.write_text(json.dumps(dict.fromkeys(JOINT_ORDER, 1.0)), encoding="utf-8")
    # Non-zero offset (+50 deg) pushes calibration upper bound (~225.6 deg) above
    # URDF upper bound (+179.9087 deg) so URDF ∩ calibration truncation is truly exercised.
    zeros_path.write_text(json.dumps(dict.fromkeys(JOINT_ORDER, 50.0)), encoding="utf-8")

    class _FakeClient:
        def __init__(self) -> None:
            self.send_calls = 0
            self.prime_calls: list[bool] = []
            # Wide calibration range (~350 deg) so URDF limits actively intersect it.
            self.latest_robot_metadata = {
                "schema_version": 2,
                "robot_model": "alohamini2pro",
                "motors": {
                    f"arm_left_{j}": {
                        "id": idx + 1,
                        "model": "sts3215",
                        "normalization": "range_m100_100",
                        "drive_mode": 0,
                        "range_min": 50,
                        "range_max": 4045,
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
            self.prime_calls.append(include_cameras)

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
            "--json",
        ],
        client_factory=lambda _host, **_kw: fake,
    )
    assert code == 0
    assert fake.send_calls == 0
    assert fake.prime_calls == [False]
    payload = json.loads(capsys.readouterr().out)
    assert payload["robot_model"] == "alohamini2pro"
    assert len(payload["effective_limits_deg"]) == 6
    urdf_limits = tool._read_urdf_limits_deg(tool.DEFAULT_URDF_PATH)
    assert set(urdf_limits) == set(JOINT_ORDER)
    for joint in JOINT_ORDER:
        urdf_joint = tool.URDF_JOINT_NAMES[joint]
        lo, hi = payload["effective_limits_deg"][urdf_joint]
        u_lo, u_hi = urdf_limits[joint]
        assert np.isfinite(lo) and np.isfinite(hi) and lo < hi
        assert lo > u_lo + 1.0
        assert hi == pytest.approx(u_hi, abs=1e-6)


def test_dry_run_refuses_subdivision_that_cannot_meet_step_budget(capsys: Any) -> None:
    """The smallest remaining interval must still respect the requested joint limit."""
    pytest.importorskip("placo")
    tool = _load_tool()
    code = tool.main(["--dry-run", "--dz", ".001", "--max-joint-step-deg", ".0001", "--json"])
    result = json.loads(capsys.readouterr().out)
    assert code == tool.EXIT_PRECHECK_REJECTED
    assert "joint step" in result["error"]
    assert not result["ok"] and result["send_action_calls"] == 0


@pytest.mark.parametrize(
    "scenario", ["short_steps", "endpoint_drift", "second_refused", "lost_endpoint"]
)
def test_motion_cli_anchors_path_and_records_partial_results(
    scenario: str, monkeypatch: Any, tmp_path: Path
) -> None:
    """Exercise the motion CLI with a simulated arm that stops 2 mm short at each point."""
    from types import SimpleNamespace

    tool = _load_tool()
    start = np.array([0.12, -0.04, 0.2])
    position = start.copy()
    targets = []

    def measured_pose() -> np.ndarray:
        pose = np.eye(4)
        pose[:3, 3] = position
        if len(targets) == 7:
            if scenario == "endpoint_drift":
                pose[2, 3] = start[2] + 0.076
            if scenario == "lost_endpoint":
                raise tool.FeedbackLost("no final feedback")
        return pose

    def move_to(x: float, y: float, z: float, **kwargs: Any) -> Any:
        targets.append([x, y, z])
        if len(targets) == 2 and scenario == "second_refused":
            raise tool.OutOfRange("second segment unreachable")
        position[:] = [x, y, z - 0.002]
        return SimpleNamespace(steps=[1], residual_m=0.002)

    arm = SimpleNamespace(measured_pose=measured_pose, move_to=move_to)
    session = SimpleNamespace(left=arm, disconnect=lambda: None)
    monkeypatch.setattr(tool, "connect", lambda *a, **kw: session)
    signs, zeros, output = (tmp_path / name for name in ("signs.json", "zeros.json", "move.json"))
    signs.write_text(json.dumps(dict.fromkeys(JOINT_ORDER, 1)))
    zeros.write_text(json.dumps(dict.fromkeys(JOINT_ORDER, 0)))
    code = tool.main(
        [
            "--enable-motion",
            "--signs-json",
            str(signs),
            "--zeros-json",
            str(zeros),
            "--points",
            "7",
            "--dz",
            "0.09",
            "--out",
            str(output),
        ]
    )
    result = json.loads(output.read_text())
    assert result["exit_code"] == code
    assert result["start_xyz_m"] == pytest.approx(start)
    assert result["target_xyz_m"] == pytest.approx(start + [0, 0, 0.09])
    for i, target in enumerate(targets, start=1):
        assert target == pytest.approx(start + [0, 0, 0.09 * i / 7])
    if scenario == "second_refused":
        assert code == 10 and len(result["moves"]) == 2
        assert result["moves"][0]["steps"] == 1
        assert result["moves"][1]["error"] == "OutOfRange"
        assert result["moves"][1]["reason"] == "second segment unreachable"
        assert result["residual_m"] is None
    elif scenario == "lost_endpoint":
        assert code == 12 and len(result["moves"]) == 7
        assert result["residual_m"] is None and result["notes"]
    elif scenario == "endpoint_drift":
        assert code == 13 and result["residual_m"] == pytest.approx(0.014)
    else:
        assert code == 0 and result["residual_m"] == pytest.approx(0.002)


def test_field_commands_select_the_deployed_robot_configuration() -> None:
    """Parse all three field commands using the CLI's actual configuration loader."""
    import shlex

    tool = _load_tool()
    runbook = (REPO_ROOT / "docs/field-ops/runbook/4-experiments.md").read_text()
    commands = [
        shlex.split(line)
        for line in runbook.splitlines()
        if line.startswith("run 10.7-") and "tools/run_cartesian_move.py --site" in line
    ]
    assert {words[1] for words in commands} == {"10.7-limits", "10.7-single", "10.7-path"}
    for words in commands:
        args = tool._build_parser().parse_args(
            words[words.index("tools/run_cartesian_move.py") + 1 :]
        )
        settings = tool.load_settings(site=REPO_ROOT / args.site, env={})
        assert settings.robot_model == "alohamini2pro" and settings.observation_request_window == 3
