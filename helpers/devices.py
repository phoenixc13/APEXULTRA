import streamlit as st
import time
import random


DEFAULT_DEVICES = [
    {
        "name": "APEX-Bot-01",
        "type": "ROS2",
        "status_code": True,
        "status_hw": True,
        "latency_ms": 32,
    },
    {
        "name": "Arduino-Servo-01",
        "type": "USB",
        "status_code": False,
        "status_hw": False,
        "latency_ms": None,
    },
]


def ensure_devices():
    if "devices" not in st.session_state:
        st.session_state.devices = DEFAULT_DEVICES.copy()


def add_device(name, endpoint, conn_type):
    d = {
        "name": name,
        "endpoint": endpoint,
        "type": conn_type,
        "status_code": True,
        "status_hw": True,
        "latency_ms": random.randint(20, 120),
    }
    st.session_state.devices.append(d)


def render_devices():
    ensure_devices()
    devices = st.session_state.devices
    if not devices:
        st.info("Nenhum robô conectado. Conecte via API ou ROS2.")
        return

    for d in devices:
        cols = st.columns([2, 1, 1, 1])
        cols[0].markdown(f"**{d.get('name')}**  \n_{d.get('type')}_")
        cols[1].markdown("✅" if d.get("status_code") else "❌")
        cols[2].markdown("✅" if d.get("status_hw") else "❌")
        latency = d.get("latency_ms")
        cols[3].markdown(f"{latency} ms" if latency is not None else "—")


def simulate_realtime(update_seconds=1, cycles=10):
    ensure_devices()
    placeholder = st.empty()
    for _ in range(cycles):
        for d in st.session_state.devices:
            if d.get("status_code"):
                d["latency_ms"] = max(1, d.get("latency_ms", 50) + random.randint(-10, 10))
            else:
                d["latency_ms"] = None
        with placeholder.container():
            render_devices()
        time.sleep(update_seconds)
