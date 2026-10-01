import json
import socket
import time

import pytest

from Lcode.drone_link import (
    MODE_REPORT_ONLY,
    DroneLink,
    DroneLinkConfig,
    DroneLinkError,
    decode_message,
    encode_message,
)


def free_udp_port():
    endpoint = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    endpoint.bind(("127.0.0.1", 0))
    port = endpoint.getsockname()[1]
    endpoint.close()
    return port


def test_authenticated_command_round_trip_and_tamper_detection():
    encoded = encode_message(
        "execute_plan",
        {"points": ["P2", "P5"]},
        7,
        "run",
        "nonce",
        timestamp=100.0,
        secret="secret",
    )
    message = decode_message(
        encoded,
        secret="secret",
        expected_nonce="nonce",
        timestamp_skew_s=1.0,
        now=100.0,
    )
    assert message.payload["points"] == ["P2", "P5"]

    raw = json.loads(encoded)
    raw["payload"]["points"] = ["P1"]
    tampered = json.dumps(raw).encode()
    with pytest.raises(DroneLinkError, match="CRC"):
        decode_message(
            tampered,
            secret="secret",
            expected_nonce="nonce",
            now=100.0,
        )


def test_command_rejects_wrong_hmac_nonce_and_old_timestamp():
    encoded = encode_message(
        "execute_plan",
        {"points": ["P1"]},
        1,
        "run",
        "nonce",
        timestamp=100.0,
        secret="secret",
    )
    with pytest.raises(DroneLinkError, match="HMAC"):
        decode_message(
            encoded,
            secret="wrong",
            expected_nonce="nonce",
            now=100.0,
        )
    with pytest.raises(DroneLinkError, match="nonce"):
        decode_message(
            encoded,
            secret="secret",
            expected_nonce="different",
            now=100.0,
        )
    with pytest.raises(DroneLinkError, match="timestamp"):
        decode_message(
            encoded,
            secret="secret",
            expected_nonce="nonce",
            timestamp_skew_s=1.0,
            now=103.0,
        )


def test_link_accepts_one_plan_then_atomically_switches_report_only():
    drone_port = free_udp_port()
    ground_port = free_udp_port()
    ground = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ground.bind(("127.0.0.1", ground_port))
    ground.settimeout(1.0)
    config = DroneLinkConfig(
        enabled=True,
        bind_host="127.0.0.1",
        bind_port=drone_port,
        remote_host="127.0.0.1",
        remote_port=ground_port,
        allowed_host="127.0.0.1",
        heartbeat_s=0.05,
        execute_plan_wait_s=1.0,
    )
    link = DroneLink(config, "run-link", secret="secret")
    assert link.start()

    nonce = None
    deadline = time.monotonic() + 1.0
    while nonce is None and time.monotonic() < deadline:
        data, _ = ground.recvfrom(8192)
        message = decode_message(data)
        if message.message_type in {"hello", "heartbeat"}:
            nonce = message.payload["session_nonce"]
    assert nonce == link.nonce

    command = encode_message(
        "execute_plan",
        {"points": ["P2", "P5"]},
        10,
        "ground",
        nonce,
        secret="secret",
    )
    ground.sendto(command, ("127.0.0.1", drone_port))
    assert link.wait_for_execute_plan() == ("P2", "P5")
    assert link.mode == MODE_REPORT_ONLY

    second = encode_message(
        "execute_plan",
        {"points": ["P1"]},
        11,
        "ground",
        nonce,
        secret="secret",
    )
    ground.sendto(second, ("127.0.0.1", drone_port))
    time.sleep(0.1)
    assert link.stats()["commands_accepted"] == 1
    assert link.stats()["commands_rejected"] >= 1
    assert link.stop()
    ground.close()


def test_execute_plan_mode_requires_psk():
    config = DroneLinkConfig(enabled=True, execute_plan_wait_s=1.0)
    link = DroneLink(config, "run", secret="")
    assert not link.start()
    assert "PSK" in link.last_error
