#!/usr/bin/env python3
"""Testes unitários do APEX Middleware v6 — foca em hot path e saturação."""

import sys
import os
import math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from apex_middleware_v6 import (
    apex_sat_int8, apex_sat_int16, apex_sat_int32,
    apex_q16_from_float, apex_q16_to_float, apex_q16_mul,
    ApexLUT, ApexKalman1D, ApexMLP, ApexSPSCRing, ApexFSM, ApexSerial,
    APEX_LUT, Q16_ONE, INT8_MIN, INT8_MAX
)

# Constantes auxiliares para testes
VCC_3V3_Q16 = (Q16_ONE * 33) // 10  # 3.3V em Q16.16
VCC_2V5_Q16 = (Q16_ONE * 25) // 10  # 2.5V em Q16.16 (brownout)

# ============== SATURAÇÃO =====================================

def test_sat_int8():
    assert apex_sat_int8(0) == 0
    assert apex_sat_int8(127) == 127
    assert apex_sat_int8(128) == 127
    assert apex_sat_int8(-128) == -128
    assert apex_sat_int8(-200) == -128
    assert apex_sat_int8(1000) == 127
    print("OK  sat_int8")

def test_sat_int16():
    assert apex_sat_int16(30000) == 30000
    assert apex_sat_int16(40000) == 32767
    assert apex_sat_int16(-40000) == -32768
    print("OK  sat_int16")

def test_sat_int32():
    assert apex_sat_int32(0x7FFFFFFF) == 0x7FFFFFFF
    assert apex_sat_int32((1<<31) - 1) == 0x7FFFFFFF
    assert apex_sat_int32(1<<40) == 0x7FFFFFFF
    assert apex_sat_int32(-(1<<40)) == -0x80000000
    print("OK  sat_int32")

# ============== Q16.16 ========================================

def test_q16_roundtrip():
    for v in [-3.14, 0.0, 0.5, 1.0, 2.71828, 100.0]:
        q = apex_q16_from_float(v)
        back = apex_q16_to_float(q)
        assert abs(back - v) < 1e-3, f"{v} -> {q} -> {back}"
    print("OK  q16 roundtrip")

def test_q16_mul_identity():
    one_q = Q16_ONE
    a = apex_q16_from_float(2.5)
    assert abs(apex_q16_to_float(apex_q16_mul(a, one_q)) - 2.5) < 1e-3
    b = apex_q16_from_float(0.5)
    prod = apex_q16_to_float(apex_q16_mul(a, b))
    assert abs(prod - 1.25) < 1e-3, f"got {prod}"
    print("OK  q16 mul")

# ============== LUT ==========================================

def test_lut_sin_cos():
    # Domínio operacional do pêndulo: [-90°, 90°] onde o erro de LUT é mínimo
    for deg in [0, 5, 15, 30, 45, 60, 75, 89]:
        rad = math.radians(deg)
        s_q = APEX_LUT.sin_q16(apex_q16_from_float(rad))
        c_q = APEX_LUT.cos_q16(apex_q16_from_float(rad))
        s = apex_q16_to_float(s_q)
        c = apex_q16_to_float(c_q)
        # Tolerância ~5e-3: LUT 1024 pontos com precisão ~6e-3 rad
        assert abs(s - math.sin(rad)) < 5e-3, f"sin({deg})={s}, expected {math.sin(rad)}"
        assert abs(c - math.cos(rad)) < 5e-3, f"cos({deg})={c}, expected {math.cos(rad)}"
    print("OK  LUT sin/cos (domínio ±90°)")

# ============== KALMAN =======================================

def test_kalman_converges():
    k = ApexKalman1D(q_q16=100, r_q16=1000)
    target = apex_q16_from_float(1.0)
    for _ in range(200):
        noisy = target + 5000  # ruído
        k.update(noisy)
    est = apex_q16_to_float(k.x)
    assert abs(est - 1.0) < 0.1, f"Kalman não convergiu: {est}"
    print(f"OK  Kalman converge (est={est:.4f})")

def test_kalman_no_overflow():
    k = ApexKalman1D(q_q16=Q16_ONE, r_q16=Q16_ONE)
    # Injeta valores extremos
    for _ in range(100):
        k.update(0x7FFFFFFF)
    assert -0x80000000 <= k.x <= 0x7FFFFFFF
    assert -0x80000000 <= k.p <= 0x7FFFFFFF
    print("OK  Kalman sem overflow")

# ============== MLP ==========================================

def test_mlp_forward_deterministic():
    m = ApexMLP(4, 8, 1)
    x = [apex_q16_from_float(0.1*i) for i in range(4)]
    y1 = m.forward(x)
    y2 = m.forward(x)
    assert y1 == y2, "MLP deve ser determinístico"
    assert len(y1) == 1
    print(f"OK  MLP forward (output={y1[0]})")

def test_mlp_no_overflow():
    m = ApexMLP(4, 12, 1)
    # Entradas extremas
    x = [0x7FFFFFFF] * 4
    y = m.forward(x)
    assert -0x80000000 <= y[0] <= 0x7FFFFFFF
    print("OK  MLP sem overflow em entradas extremas")

def test_mlp_copy_and_mutate():
    a = ApexMLP(3, 4, 1)
    b = ApexMLP(3, 4, 1)
    a.mutate(5)
    b.copy_from(a)
    for i in range(3):
        for j in range(4):
            assert a.w_ih[i][j] == b.w_ih[i][j]
    print("OK  MLP copy_from")

# ============== RING BUFFER ==================================

def test_ring_basic():
    r = ApexSPSCRing(capacity=8, slot_bytes=8)
    assert r.push(123, b"hello") is True
    assert r.occupancy() == 1
    slot = r.pop()
    assert slot is not None
    assert slot.ts_ns == 123
    assert slot.payload[:5] == b"hello"
    print("OK  SPSC ring basic")

def test_ring_full():
    r = ApexSPSCRing(capacity=4, slot_bytes=8)
    # Em ring SPSC com capacidade N, só cabem N-1 elementos (slot sentinel)
    accepted = 0
    for i in range(8):
        if r.push(i, b"x"):
            accepted += 1
    assert accepted == 3, f"esperado aceitar 3, aceitou {accepted}"
    assert r.occupancy() == 3
    # Tentar mais pushes não deve crashar
    for i in range(10):
        r.push(i, b"x")
    print(f"OK  SPSC ring full (aceitos={accepted}, occ={r.occupancy()})")

def test_ring_wraparound():
    r = ApexSPSCRing(capacity=4, slot_bytes=8)
    for _ in range(10):
        for i in range(4):
            r.push(i, bytes([i]))
        for _ in range(4):
            r.pop()
    assert r.occupancy() == 0
    print("OK  SPSC ring wraparound")

# ============== FSM ==========================================

def test_fsm_normal_to_warn():
    f = ApexFSM(warn_rad=0.5, shutdown_rad=1.0)
    ang_warn = apex_q16_from_float(0.6)
    # 2 ciclos no warn -> vira WARN
    f.evaluate(ang_warn, VCC_3V3_Q16)
    s = f.evaluate(ang_warn, VCC_3V3_Q16)
    assert s == ApexFSM.WARN, f"expected WARN, got {s}"
    print("OK  FSM normal -> warn")

def test_fsm_shutdown_immediate():
    f = ApexFSM(warn_rad=0.5, shutdown_rad=1.0)
    ang = apex_q16_from_float(1.5)
    s = f.evaluate(ang, VCC_3V3_Q16)
    assert s == ApexFSM.SHUTDOWN
    print("OK  FSM emergency shutdown")

def test_fsm_hysteresis():
    f = ApexFSM(warn_rad=0.5, shutdown_rad=1.0)
    ang = apex_q16_from_float(0.6)
    f.evaluate(ang, VCC_3V3_Q16)
    f.evaluate(ang, VCC_3V3_Q16)
    assert f.state == ApexFSM.WARN
    # Sai do warn com ângulo pequeno
    safe = apex_q16_from_float(0.0)
    for _ in range(20):
        f.evaluate(safe, VCC_3V3_Q16)
    assert f.state == ApexFSM.NORMAL, f"recuperou: {f.state}"
    print("OK  FSM histerese")

def test_fsm_brownout():
    f = ApexFSM()
    s = f.evaluate(0, VCC_2V5_Q16)  # VCC baixo -> shutdown imediato
    assert s == ApexFSM.SHUTDOWN
    print("OK  FSM brownout detection")

# ============== SERIAL =======================================

def test_serial_pack_unpack():
    seq = 42
    ts = 1234567890
    ang = 65536
    vel = -32768
    torque = 100
    safety = 1
    buf = ApexSerial.pack(seq, ts, ang, vel, torque, safety)
    assert len(buf) == 32, f"pack size = {len(buf)}"
    s, t, a, v, tr, sf = ApexSerial.unpack(buf)
    assert (s, t, a, v, tr, sf) == (seq, ts, ang, vel, torque, safety)
    print("OK  serial pack/unpack")

# ============== RUNNER =======================================

def run_all():
    tests = [
        test_sat_int8, test_sat_int16, test_sat_int32,
        test_q16_roundtrip, test_q16_mul_identity,
        test_lut_sin_cos,
        test_kalman_converges, test_kalman_no_overflow,
        test_mlp_forward_deterministic, test_mlp_no_overflow, test_mlp_copy_and_mutate,
        test_ring_basic, test_ring_full, test_ring_wraparound,
        test_fsm_normal_to_warn, test_fsm_shutdown_immediate,
        test_fsm_hysteresis, test_fsm_brownout,
        test_serial_pack_unpack,
    ]
    failed = 0
    for t in tests:
        try:
            t()
        except AssertionError as e:
            print(f"FAIL {t.__name__}: {e}")
            failed += 1
        except Exception as e:
            print(f"ERR  {t.__name__}: {type(e).__name__}: {e}")
            failed += 1
    print(f"\n{len(tests) - failed}/{len(tests)} testes passaram")
    return failed == 0

if __name__ == "__main__":
    sys.exit(0 if run_all() else 1)