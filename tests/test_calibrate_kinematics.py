"""``tools/calibrate_kinematics.py`` 的离线契约测试。"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Any

from alohamini_cartesian.kinematics.joint_adapter import JOINT_ORDER

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = REPO_ROOT / "tools" / "calibrate_kinematics.py"


def _load_tool() -> Any:
    spec = importlib.util.spec_from_file_location("calibrate_kinematics", TOOL_PATH)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _sample_metadata() -> dict[str, Any]:
    return {
        "schema_version": 2,
        "robot_model": "alohamini2pro",
        "motors": {
            f"arm_left_{j}": {
                "id": idx + 1,
                "model": "sts3215",
                "normalization": "degrees",
                "drive_mode": 0,
                "range_min": 0,
                "range_max": 4095,
                "homing_offset": 0,
                "resolution": 4096,
            }
            for idx, j in enumerate(JOINT_ORDER)
        },
    }


def test_offline_calibration_computes_exact_zeros_and_passes_self_check(
    tmp_path: Path, capsys: Any
) -> None:
    """给定观测快照与 signs，反推 zeros 后自检误差为 0 并输出扁平六键 JSON。"""
    tool = _load_tool()
    signs = {j: (1.0 if i % 2 == 0 else -1.0) for i, j in enumerate(JOINT_ORDER)}
    motor_deg = [12.5, 45.0, -30.0, 5.0, -10.0, 22.0]
    obs_doc = {
        "observation": {f"arm_left_{j}.pos": motor_deg[i] for i, j in enumerate(JOINT_ORDER)},
        "metadata": _sample_metadata(),
    }
    signs_path = tmp_path / "signs.json"
    obs_path = tmp_path / "obs.json"
    zeros_out = tmp_path / "zeros.json"
    signs_path.write_text(json.dumps(signs), encoding="utf-8")
    obs_path.write_text(json.dumps(obs_doc), encoding="utf-8")

    code = tool.main(
        [
            "--side",
            "left",
            "--signs-json",
            str(signs_path),
            "--from-observation-json",
            str(obs_path),
            "--zeros-out",
            str(zeros_out),
            "--json",
        ]
    )
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["self_check"]["within_tolerance"] is True
    assert payload["self_check"]["max_error_deg"] < 1e-9
    saved_zeros = json.loads(zeros_out.read_text(encoding="utf-8"))
    assert tuple(saved_zeros.keys()) == JOINT_ORDER


def test_nested_signs_json_is_rejected_with_exit_15(tmp_path: Path) -> None:
    """嵌套包装（非扁平六键）的 signs 文件必须按退出码 15 拒绝。"""
    tool = _load_tool()
    bad_signs = tmp_path / "bad_signs.json"
    bad_signs.write_text(
        json.dumps({"left": dict.fromkeys(JOINT_ORDER, 1.0)}),
        encoding="utf-8",
    )
    code = tool.main(["--side", "left", "--signs-json", str(bad_signs)])
    assert code == tool.EXIT_MISSING_CALIB
