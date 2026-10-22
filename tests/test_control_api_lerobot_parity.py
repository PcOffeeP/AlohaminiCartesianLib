"""AlohaMini 控制层用例中必须用真实 ``AlohaMiniClient`` 才能成立的差分对照用例。

这两个用例断言的对象不是控制层行为，而是 ``AlohaMiniClient`` 自身的两个契约：

* ``send_action`` 线上载荷恰为调用方的键集合，0 填充的完整状态向量只出现在
  返回值里；
* ``get_observation`` 的返回观测与 ``latest_robot_metadata`` /
  ``latest_control_status`` 三面原子提交，且被拒帧不推进任何一面。

两者都需要生产 client 的序列化与帧校验实现作 oracle，本地假 client 会自证。
未装 lerobot 的环境由 ``pytest.importorskip`` 整体跳过——这是预期结果，不得为
了变绿删减断言。搬运来源：lerobot 仓库
``tests/robots/test_alohamini_control_api.py``（``test_char_1_*`` / ``test_char_2_*``）。
控制层自身的用例在 ``tests/test_control_api.py``。
"""

import json
from collections import deque
from typing import Any

import numpy as np
import pytest

pytest.importorskip("lerobot", reason="lerobot 差分对照用例只在装有 runtime extra 的环境运行")

from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient
from lerobot.utils.constants import ACTION, OBS_STATE

from alohamini_cartesian.kinematics.model_specs import arm_state_keys_for_robot_model

# ===========================================================================
# Harness: pure-object AlohaMiniClient (no zmq, no sockets, no threads).
# ===========================================================================


class _StubZmq:
    """Minimal ``zmq`` module stand-in for ``_send_observation_request``."""

    NOBLOCK = 4
    ZMQError = type("ZMQError", (Exception,), {})


class _RecordingCmdSocket:
    """Records every command string; the command seam under test."""

    def __init__(self) -> None:
        self.sent_strings: list[str] = []

    def send_string(self, message: str) -> int:
        self.sent_strings.append(message)
        return len(message)


class _FakeRequestSocket:
    """Records observation request tokens; never blocks, never fails."""

    def __init__(self) -> None:
        self.sent: list[bytes] = []

    def send(self, data: Any, flags: Any = 0) -> int:
        self.sent.append(bytes(data))
        return len(data)


def _make_command_client() -> AlohaMiniClient:
    """Client with a real ``_state_order`` (alohamini2pro) and a fake cmd socket."""
    client = object.__new__(AlohaMiniClient)
    (
        client._left_arm_state_keys,
        client._right_arm_state_keys,
    ) = arm_state_keys_for_robot_model("alohamini2pro")
    client._is_connected = True
    client.zmq_cmd_socket = _RecordingCmdSocket()
    return client


def _make_observation_client() -> AlohaMiniClient:
    """Client with an injected request socket and empty camera set.

    ``_state_order`` is derived through the real cached property so frame
    builders and assertions use the production alohamini2pro key layout.
    """
    client = object.__new__(AlohaMiniClient)
    (
        client._left_arm_state_keys,
        client._right_arm_state_keys,
    ) = arm_state_keys_for_robot_model("alohamini2pro")
    client._cameras_ft = {}
    client.logs = {}
    client.last_frames = {}
    client.last_remote_state = {}
    client.latest_host_timing = {}
    client.latest_robot_metadata = {}
    client.latest_control_status = None
    client._observation_sequence = 0
    client._observation_request_tokens = deque()
    client._observation_request_id = 0
    client._observation_request_sent_stamps = {}
    client._last_request_token = None
    client._last_request_received_monotonic_s = None
    client._host_session_id = None
    client._retired_host_session_ids = set()
    client._host_sample_sequence = None
    client.observation_request_window = 2
    client.polling_timeout_ms = 20
    client.connect_timeout_s = 1.0
    client.zmq_observation_socket = _FakeRequestSocket()
    client._zmq = _StubZmq()
    client._is_connected = True
    return client


def _valid_frame(client: AlohaMiniClient, *, sample_sequence: int = 7) -> dict:
    """One fully valid candidate frame with distinct state values."""
    frame = {key: 1.0 + index for index, key in enumerate(client._state_order)}
    frame["_robot_metadata"] = {"schema_version": 2, "robot_model": "alohamini2pro"}
    frame["_control_status"] = {
        "schema_version": 1,
        "host_session_id": "session-A",
        "sample_sequence": sample_sequence,
        "joint_holds": {},
        "gripper_holds": {},
    }
    return frame


def _install_frame(client: AlohaMiniClient, frame: dict) -> None:
    """Serve ``frame`` for every observation response."""
    parts = [json.dumps(frame).encode("utf-8")]

    def responder(request_token: Any, timeout_ms: Any) -> Any:
        return parts

    client._receive_observation_response = responder


# ===========================================================================
# Characterization baseline: AlohaMiniClient's own two contracts.
# ===========================================================================


def test_char_1_send_action_transmits_caller_key_set_and_zero_fills_only_return() -> None:
    """Wire payload == caller's key set; return value covers the full state order.

    A "reasonable but wrong" refactor that forwards the 0-filled state vector
    (``action_sent``) instead of the caller's mapping, or that drops the
    zero-fill from the return value, is caught here.
    """
    client = _make_command_client()
    action = {"x.vel": 0.25, "y.vel": -0.5, "theta.vel": 30.0}

    returned = client.send_action(action)

    assert len(client.zmq_cmd_socket.sent_strings) == 1
    wire = json.loads(client.zmq_cmd_socket.sent_strings[0])
    assert wire == action
    assert set(wire) == set(action), "0-filled keys must not appear on the wire"

    assert set(returned) == set(client._state_order) | {ACTION}
    assert returned["x.vel"] == pytest.approx(0.25)
    assert returned["y.vel"] == pytest.approx(-0.5)
    assert returned["theta.vel"] == pytest.approx(30.0)
    assert returned["arm_left_shoulder_pan.pos"] == 0.0
    assert returned[ACTION].shape == (len(client._state_order),)


def test_char_2_get_observation_separates_side_channels_and_commits_atomically() -> None:
    """Returned observation excludes side channels; attributes move with the frame.

    A wrong implementation that merges ``_robot_metadata`` / ``_control_status``
    into the observation, or that commits an attribute for a rejected frame,
    is caught here.
    """
    client = _make_observation_client()
    frame = _valid_frame(client, sample_sequence=7)

    _install_frame(client, frame)
    observation = client.get_observation()

    for side_channel in ("_robot_metadata", "_control_status", "_host_timing"):
        assert side_channel not in observation
    assert set(client._state_order).issubset(observation.keys())
    assert OBS_STATE in observation

    assert client.latest_robot_metadata == frame["_robot_metadata"]
    assert client.latest_control_status == frame["_control_status"]
    assert client.observation_sequence == 1

    # A rejected frame (missing required state key) must not advance any face.
    previous_metadata = client.latest_robot_metadata
    previous_status = client.latest_control_status
    previous_sequence = client.observation_sequence

    bad_frame = dict(frame)
    del bad_frame[client._state_order[0]]
    _install_frame(client, bad_frame)
    rejected_observation = client.get_observation()

    for key in client._state_order:
        assert rejected_observation[key] == observation[key]
    assert np.array_equal(rejected_observation[OBS_STATE], observation[OBS_STATE])
    assert client.latest_robot_metadata is previous_metadata
    assert client.latest_control_status is previous_status
    assert client.observation_sequence == previous_sequence
