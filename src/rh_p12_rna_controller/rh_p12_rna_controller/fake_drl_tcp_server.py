#!/usr/bin/env python3
"""
Fake DRL TCP server for local testing (no robot needed).

Implements the same framed JSON protocol as DRL_SERVER_CODE:
  - Frame: 2-byte big-endian length + JSON(utf-8)
  - Client -> Server: {"type":"cmd","id":<int>,"frames":[<hexstr>, ...]}
  - Server -> Client: {"type":"ack","id":<int>,"ok":<bool>,"err":<str>}
  - Server -> Client: {"type":"state","cur":<int>,"pos":<int>}

This lets you validate:
  - TCP framing
  - ACK waiting logic
  - state parsing and scaling on ROS2 node side
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from dataclasses import dataclass


def send_frame(conn: socket.socket, obj: dict) -> None:
    payload = json.dumps(obj).encode("utf-8")
    conn.sendall(struct.pack(">H", len(payload)) + payload)


def recv_frames(conn: socket.socket, rxbuf: bytearray) -> list[dict]:
    try:
        chunk = conn.recv(4096)
    except socket.timeout:
        return []
    if not chunk:
        raise ConnectionError("client disconnected")
    rxbuf.extend(chunk)
    out: list[dict] = []
    while len(rxbuf) >= 2:
        n = (rxbuf[0] << 8) | rxbuf[1]
        if len(rxbuf) < 2 + n:
            break
        payload = bytes(rxbuf[2 : 2 + n])
        del rxbuf[: 2 + n]
        try:
            out.append(json.loads(payload.decode("utf-8", errors="ignore")))
        except Exception:
            pass
    return out


@dataclass
class SimState:
    torque: bool = True
    goal_pos_raw: int = 0
    present_pos_raw: float = 0.0
    goal_cur: int = 0
    present_cur: int = 0


def parse_modbus_and_update(state: SimState, pkt: bytes) -> None:
    # Very small parser: just enough for our test flows.
    if len(pkt) < 6:
        return
    fn = pkt[1]
    # FC06: [id][0x06][addrHi][addrLo][valHi][valLo][crcLo][crcHi]
    if fn == 0x06 and len(pkt) >= 8:
        addr = (pkt[2] << 8) | pkt[3]
        val = (pkt[4] << 8) | pkt[5]
        if addr == 256:  # torque enable
            state.torque = bool(val)
        elif addr == 275:  # goal current
            state.goal_cur = int(val)
        elif addr == 282:  # goal position (some firmwares)
            state.goal_pos_raw = int(val)
    # FC16: [id][0x10][addrHi][addrLo][qtyHi][qtyLo][byteCount][data...][crc..]
    elif fn == 0x10 and len(pkt) >= 9:
        addr = (pkt[2] << 8) | pkt[3]
        qty = (pkt[4] << 8) | pkt[5]
        if addr == 282 and qty >= 1 and len(pkt) >= 13:
            # take first register as goal pos raw
            val = (pkt[7] << 8) | pkt[8]
            state.goal_pos_raw = int(val)


def state_loop(conn: socket.socket, state: SimState, stop_ev: threading.Event) -> None:
    last_t = time.time()
    while not stop_ev.is_set():
        now = time.time()
        dt = max(0.0, now - last_t)
        last_t = now

        # simple first-order approach to goal
        if state.torque:
            alpha = min(1.0, dt / 0.3)  # time constant-ish
            state.present_pos_raw = (1 - alpha) * state.present_pos_raw + alpha * float(
                state.goal_pos_raw
            )
            state.present_cur = int(min(max(abs(state.goal_cur) // 10, 0), 500))
        else:
            state.present_cur = 0

        try:
            send_frame(
                conn,
                {
                    "type": "state",
                    "cur": int(state.present_cur),
                    "pos": int(round(state.present_pos_raw)),
                },
            )
        except Exception:
            break
        time.sleep(0.05)


def main() -> None:
    host = "127.0.0.1"
    port = 9000
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"[fake_drl] listening on {host}:{port}")

    conn, addr = srv.accept()
    print(f"[fake_drl] client connected: {addr}")
    conn.settimeout(0.1)

    state = SimState()
    stop_ev = threading.Event()
    t = threading.Thread(target=state_loop, args=(conn, state, stop_ev), daemon=True)
    t.start()

    rxbuf = bytearray()
    try:
        send_frame(conn, {"type": "hello", "info": "fake_drl"})
        while True:
            for msg in recv_frames(conn, rxbuf):
                if isinstance(msg, str) and msg == "STOP":
                    return
                if not isinstance(msg, dict):
                    continue
                if msg.get("type") == "cmd":
                    cmd_id = int(msg.get("id", 0))
                    ok = True
                    err = ""
                    frames = msg.get("frames", [])
                    try:
                        for hx in frames:
                            parse_modbus_and_update(state, bytes.fromhex(str(hx)))
                    except Exception as e:
                        ok = False
                        err = str(e)
                    send_frame(conn, {"type": "ack", "id": cmd_id, "ok": ok, "err": err})
    except KeyboardInterrupt:
        pass
    finally:
        stop_ev.set()
        try:
            conn.close()
        except Exception:
            pass
        try:
            srv.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()

