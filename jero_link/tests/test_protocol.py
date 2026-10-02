import json
import math
import time

import pytest
from jero_link.protocol import MAGIC, load_key
from jero_link.receiver import BUTTON_HOLD_S

from jero_link import BUTTONS, Command, JeroClient, ProtocolError, Receiver, UdpReceiver, decode, encode

KEY = bytes.fromhex("11" * 32)
ZERO = Command()


def frame(cmd=ZERO, seq=1, session="s1", key=KEY):
    return encode(cmd, seq=seq, session=session, key=key)


def test_roundtrip_signed():
    cmd = Command(vx=0.1, vy=-0.05, wz=0.5, head_yaw=0.2, buttons=("A",), left_trigger=0.3, ttl_ms=300)
    msg = decode(frame(cmd), KEY)
    assert msg.session == "s1" and msg.seq == 1
    assert msg.command.as_commands() == pytest.approx([0.1, -0.05, 0.5, 0.0, 0.0, 0.2, 0.0])
    assert msg.command.buttons == ("A",)
    assert msg.command.ttl_ms == 300


def test_values_are_clamped_to_policy_limits():
    msg = decode(frame(Command(vx=5, vy=-5, wz=9, head_pitch=-3, ttl_ms=99999)), KEY)
    c = msg.command
    assert (c.vx, c.vy, c.wz, c.head_pitch, c.ttl_ms) == (0.15, -0.2, 1.0, -0.78, 2000)


def test_signature_required_when_key_set():
    with pytest.raises(ProtocolError, match="unsigned"):
        decode(frame(key=None), KEY)
    with pytest.raises(ProtocolError, match="signature"):
        decode(frame(key=bytes.fromhex("22" * 32)), KEY)


def test_tampered_body_rejected():
    f = bytearray(frame(Command(vx=0.01)))
    f[-5] ^= 0x01
    with pytest.raises(ProtocolError):
        decode(bytes(f), KEY)


def test_unsigned_mode_accepts_unsigned():
    assert decode(frame(Command(vx=0.1), key=None), None).command.vx == 0.1


@pytest.mark.parametrize(
    "mutate",
    [
        lambda d: d.update(v=2),
        lambda d: d.update(seq=-1),
        lambda d: d.update(seq=True),
        lambda d: d.update(sid=""),
        lambda d: d["cmd"].update(vx="fast"),
        lambda d: d["cmd"].update(bogus=1.0),
        lambda d: d.update(btn=["Z"]),
        lambda d: d.update(trig=[0.1]),
        lambda d: d.update(estop="yes"),
    ],
)
def test_malformed_payloads_rejected(mutate):
    raw = frame(key=None)
    data = json.loads(raw[36:])
    mutate(data)
    bad = raw[:36] + json.dumps(data).encode()
    with pytest.raises(ProtocolError):
        decode(bad, None)


def test_non_finite_rejected():
    raw = frame(key=None)
    bad = raw[:36] + raw[36:].replace(b'"vx":0.0', b'"vx":NaN')
    with pytest.raises(ProtocolError):
        decode(bad, None)


def test_garbage_rejected():
    for junk in (b"", b"hello", MAGIC + b"\x00" * 40, b"\xff" * 2000):
        with pytest.raises(ProtocolError):
            decode(junk, None)


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_receiver_ttl_and_ordering():
    clock = FakeClock()
    rx = Receiver(key=KEY, clock=clock)
    assert rx.snapshot()[0] is None

    assert rx.handle(frame(Command(vx=0.1, ttl_ms=500), seq=5))
    assert rx.snapshot()[0].vx == 0.1
    assert not rx.handle(frame(Command(vx=0.05), seq=5))  # replay / duplicate
    assert not rx.handle(frame(Command(vx=0.05), seq=4))  # out of order
    assert rx.snapshot()[0].vx == 0.1

    clock.t += 0.6
    assert rx.snapshot()[0] is None  # stale -> robot falls back to zero command

    # a restarted client (new session) is accepted even with a lower seq
    assert rx.handle(frame(Command(vx=0.02), seq=1, session="s2"))
    assert rx.snapshot()[0].vx == 0.02
    assert rx.dropped == 2 and rx.accepted == 2


def test_receiver_button_hold():
    clock = FakeClock()
    rx = Receiver(key=KEY, clock=clock)
    rx.handle(frame(Command(buttons=("A",)), seq=1))
    assert rx.snapshot()[1]["A"] is True
    clock.t += BUTTON_HOLD_S + 0.01
    assert rx.snapshot()[1]["A"] is False
    assert set(rx.snapshot()[1]) == set(BUTTONS)


def test_receiver_estop_callback():
    hits = []
    rx = Receiver(key=KEY, on_estop=lambda: hits.append(1))
    rx.handle(frame(Command(estop=True), seq=1))
    rx.handle(frame(Command(), seq=2))
    assert hits == [1]


def test_client_to_udp_receiver_loopback():
    rx = UdpReceiver(bind="127.0.0.1", port=0, key=KEY)
    try:
        with JeroClient("127.0.0.1", rx.port, key=KEY, rate_hz=50) as duck:
            duck.drive(vx=0.08, wz=-0.3)
            duck.press("B")
            deadline = time.time() + 2
            seen_b = False
            while time.time() < deadline:
                cmd, pressed = rx.snapshot()
                seen_b |= pressed["B"]
                if cmd is not None and cmd.vx == 0.08 and seen_b:
                    break
                time.sleep(0.01)
            assert cmd is not None and cmd.vx == 0.08 and math.isclose(cmd.wz, -0.3)
            assert seen_b
        # close() sends a final zero command
        time.sleep(0.1)
        assert rx.snapshot()[0].vx == 0.0
    finally:
        rx.close()


def test_client_estop_reaches_receiver():
    hits = []
    rx = UdpReceiver(bind="127.0.0.1", port=0, key=KEY, on_estop=lambda: hits.append(1))
    try:
        client = JeroClient("127.0.0.1", rx.port, key=KEY)
        client.drive(vx=0.1)
        client.estop()
        for _ in range(3):
            client.send_once()
        time.sleep(0.2)
        assert hits, "estop not delivered"
        assert rx.snapshot()[0].vx == 0.0
        client.sock.close()
    finally:
        rx.close()


def test_client_rejects_bad_config():
    with pytest.raises(ValueError):
        JeroClient("127.0.0.1", rate_hz=1, ttl_ms=500)
    with pytest.raises(ValueError):
        JeroClient("127.0.0.1").press("Z")


def test_load_key(tmp_path):
    p = tmp_path / "k"
    p.write_text("ab" * 32 + "\n")
    assert load_key(str(p)) == bytes.fromhex("ab" * 32)
    p.write_text("abcd")
    with pytest.raises(ValueError):
        load_key(str(p))
    assert load_key(None) is None


def test_datagram_fits_single_packet():
    f = frame(Command(buttons=BUTTONS, estop=True))
    assert len(f) < 512
