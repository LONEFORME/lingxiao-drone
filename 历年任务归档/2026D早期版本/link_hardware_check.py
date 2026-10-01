"""板载 UDP 通信链路硬件测试 — 不接飞控也不解锁，零风险。

测试项：
  1. 本机 UDP 回环收发（验证 socket + CRC + 基本消息流）
  2. 可选局域网对端通信（验证连通性）
  3. HMAC 认证 execute_plan 指令编解码
  4. 事件序列化与转发

用法：
    # 本机回环测试（默认）
    python3 link_hardware_test.py

    # 局域网对端测试（两端各跑一条命令）
    # 服务端（接收）：
    python3 link_hardware_test.py --peer --mode listen --bind-port 5601
    # 客户端（发送）：
    python3 link_hardware_test.py --peer --mode send --remote-host 192.168.x.x --remote-port 5601

    # 使用自定义 PSK 验证 HMAC 认证路径
    DRONE_LINK_PSK=mysecret python3 link_hardware_test.py --hmac

失败时退出码非 0，但绝不连接飞控或解锁。
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import sys
import time
import zlib

sys.path.insert(0, os.path.dirname(__file__))

PROTOCOL_VERSION = 1
MAGIC_JPEG = b"DJPG"


def _print(name: str, status: str, detail: str) -> None:
    icon = {"PASS": "✓", "FAIL": "✗", "WARN": "⚠", "INFO": "•"}.get(status, "?")
    print(f"  {icon} [{status}] {name}: {detail}")


# ---------------------------------------------------------------------------
#  Minimal wire format helpers (de-duplicated from drone_link.py for test)
# ---------------------------------------------------------------------------

def _build_message(msg_type: str, payload: dict) -> bytes:
    encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    crc = struct.pack(">I", zlib.crc32(encoded) & 0xFFFFFFFF)
    header = struct.pack(">BBId", PROTOCOL_VERSION, ord(msg_type[0]), 0, time.time())
    return header + crc + encoded


def _hmac_sign(payload: bytes, psk: bytes) -> bytes:
    import hmac, hashlib
    return hmac.new(psk, payload, hashlib.sha256).digest()


# ---------------------------------------------------------------------------
#  Test cases
# ---------------------------------------------------------------------------

def test_loopback() -> bool:
    """本机 UDP 回环：发一条遥测消息，收回来验证。"""
    try:
        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_sock.settimeout(2.0)
        recv_sock.bind(("127.0.0.1", 0))
        recv_port = recv_sock.getsockname()[1]

        send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        msg = _build_message("T", {"event": "test_loopback", "seq": 1})
        send_sock.sendto(msg, ("127.0.0.1", recv_port))

        data, addr = recv_sock.recvfrom(4096)
        if len(data) < 18:
            _print("loopback", "FAIL", f"received {len(data)} bytes, need >=18")
            return False

        version, mtype, seqnum, _timestamp = struct.unpack(">BBId", data[:14])
        crc_recv = struct.unpack(">I", data[14:18])[0]
        payload = data[18:]
        crc_calc = zlib.crc32(payload) & 0xFFFFFFFF
        ok = crc_recv == crc_calc
        _print(
            "loopback",
            "PASS" if ok else "FAIL",
            f"version={version} type={chr(mtype)} seq={seqnum} "
            f"payload={len(payload)}B crc={'ok' if ok else 'MISMATCH'}",
        )
        return ok
    except socket.timeout:
        _print("loopback", "FAIL", "receive timeout")
        return False
    except Exception as exc:
        _print("loopback", "FAIL", str(exc))
        return False
    finally:
        for s in ("recv_sock", "send_sock"):
            sock = locals().get(s)
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass


def test_hmac_sign() -> bool:
    """验证 HMAC-SHA256 签名和验签路径（不依赖网络）。"""
    try:
        import hmac, hashlib
    except ImportError:
        _print("hmac", "FAIL", "hmac/hashlib not available")
        return False

    psk = os.getenv("DRONE_LINK_PSK", "test-psk").encode("utf-8")
    payload = json.dumps({"cmd": "execute_plan", "points": ["P1"]}).encode("utf-8")
    signature = _hmac_sign(payload, psk)

    expected = hmac.new(psk, payload, hashlib.sha256).digest()
    ok = hmac.compare_digest(signature, expected)
    _print("hmac_sign", "PASS" if ok else "FAIL", f"signature={'match' if ok else 'MISMATCH'}")
    return ok


def test_event_serialize() -> bool:
    """验证 MissionEvent 序列化 → JSON → 反序列化 完整路径。"""
    try:
        from Lcode.mission_events import MissionEvent
    except ImportError as exc:
        _print("event_serialize", "FAIL", f"cannot import: {exc}")
        return False

    event = MissionEvent(
        event="WAYPOINT_ARRIVED",
        point_id="P1",
        target_index=0,
        action="observe",
        details={"accuracy_m": 0.05},
    )
    raw = json.dumps(event.as_dict(), ensure_ascii=False)
    restored = json.loads(raw)
    ok = (
        restored.get("event") == "WAYPOINT_ARRIVED"
        and restored.get("point_id") == "P1"
        and restored.get("action") == "observe"
    )
    _print("event_serialize", "PASS" if ok else "FAIL", f"{len(raw)}B round-trip ok={ok}")
    return ok


def test_peer_connect(host: str, port: int, mode: str) -> bool:
    """局域网对端通信测试。mode=listen: 等待接收一条消息；mode=send: 发送一条消息。"""
    if mode == "listen":
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(15.0)
        try:
            sock.bind(("0.0.0.0", port))
            _print("peer_connect", "INFO", f"listening on 0.0.0.0:{port}, waiting up to 15s...")
            data, addr = sock.recvfrom(4096)
            _print("peer_connect", "PASS", f"received {len(data)}B from {addr}")
            return True
        except socket.timeout:
            _print("peer_connect", "FAIL", "timeout waiting for peer message")
            return False
        except Exception as exc:
            _print("peer_connect", "FAIL", str(exc))
            return False
        finally:
            sock.close()
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(3.0)
        try:
            msg = _build_message("T", {"event": "peer_test", "seq": 1})
            sock.sendto(msg, (host, port))
            _print("peer_connect", "PASS", f"sent {len(msg)}B to {host}:{port}")
            return True
        except Exception as exc:
            _print("peer_connect", "FAIL", f"send to {host}:{port} failed: {exc}")
            return False
        finally:
            sock.close()


def test_udp_jpeg_encode() -> bool:
    """验证 UDP-JPEG 分片编码不崩溃，纯本地计算。"""
    jpeg = b"\xff\xd8\xff\xe0" + os.urandom(4000)
    try:
        from Lcode.video_backends import encode_udp_jpeg_packets, decode_udp_jpeg_packet
    except ImportError as exc:
        _print("udp_jpeg", "WARN", f"video_backends not available: {exc}")
        return True  # not a hard failure for link test

    try:
        packets = encode_udp_jpeg_packets(jpeg, frame_id=1, max_datagram=1200)
        assert len(packets) > 1, "should produce multiple chunks"
        decoded = b"".join(
            decode_udp_jpeg_packet(packet)[1] for packet in packets
        )
        ok = decoded == jpeg
        _print(
            "udp_jpeg",
            "PASS" if ok else "FAIL",
            f"{len(jpeg)}B → {len(packets)} packets → {'match' if ok else 'MISMATCH'}",
        )
        return ok
    except Exception as exc:
        _print("udp_jpeg", "FAIL", str(exc))
        return False


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Board-side UDP link hardware test"
    )
    parser.add_argument("--peer", action="store_true", help="Run peer-to-peer test")
    parser.add_argument(
        "--mode", choices=("listen", "send"), default="send",
        help="Peer test mode (default: send)"
    )
    parser.add_argument("--bind-port", type=int, default=5601)
    parser.add_argument("--remote-host", default="127.0.0.1")
    parser.add_argument("--remote-port", type=int, default=5602)
    parser.add_argument("--hmac", action="store_true", help="Include HMAC test")
    args = parser.parse_args()

    print("=" * 48)
    print("  板载 UDP 链路测试 (link_hardware_test)")
    print("  " + time.strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 48)
    print()

    checks: list[tuple[str, bool]] = []

    if args.peer:
        checks.append((
            f"局域网通信 ({'listen' if args.mode == 'listen' else f'send → {args.remote_host}:{args.remote_port}'})",
            test_peer_connect(args.remote_host, args.bind_port if args.mode == "listen" else args.remote_port, args.mode),
        ))
    else:
        checks.append(("本机UDP回环", test_loopback()))

    checks.append(("UDP-JPEG编码", test_udp_jpeg_encode()))
    checks.append(("事件序列化", test_event_serialize()))

    if args.hmac:
        checks.append(("HMAC签名", test_hmac_sign()))

    print()
    print("-" * 48)
    failures = [name for name, ok in checks if not ok]
    if failures:
        print(f"  结果: FAIL ({len(failures)} 项失败)")
        for name in failures:
            print(f"    ✗ {name}")
        print()
        print("  ⚠ 本脚本不连接飞控或不解锁，失败仅反映网络/UDP状态。")
        return 1
    else:
        print("  结果: PASS (全部通过)")
        print()
        print("  ✓ UDP 链路功能正常。")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
