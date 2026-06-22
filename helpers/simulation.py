import streamlit as st
import importlib
import plotly.graph_objects as go
import time


def has_apex_engine():
    try:
        import apex_engine
        return True
    except Exception:
        return False


def run_simulation_ui():
    if not st.session_state.get("logged_in"):
        st.warning("Faça login para acessar a simulação")
        return

    st.sidebar.subheader("Controles de Simulação")
    nodes = st.sidebar.slider("Número de nós", 1, 8, 2)
    freq = st.sidebar.slider("Frequência (Hz)", 10, 500, 50)
    inject_fault = st.sidebar.checkbox("Injetar falhas")

    st.write("## Simulação APEX")
    st.write(f"Nós: {nodes} • Frequência: {freq} Hz • Injetar falhas: {inject_fault}")

    # Try to use apex_engine's API if present
    try:
        import apex_engine
        sim = getattr(apex_engine, "run_simulation", None)
        if callable(sim):
            data = sim(nodes=nodes, freq=freq, inject_fault=inject_fault)
            # expect data dict with time, angle, velocity, torque
            t = data.get("time", list(range(len(data.get("angle", [])))))
            fig = go.Figure()
            fig.add_trace(go.Scatter(x=t, y=data.get("angle", []), name="angle"))
            fig.add_trace(go.Scatter(x=t, y=data.get("velocity", []), name="velocity"))
            fig.add_trace(go.Scatter(x=t, y=data.get("torque", []), name="torque"))
            st.plotly_chart(fig, use_container_width=True)
            st.metric("Ciclos", len(t))
            st.metric("Reward médio", data.get("reward_avg", "—"))
            st.metric("Watchdog timeouts", data.get("watchdog_timeouts", 0))
            return
    except Exception:
        pass

    # Fallback demo simulation
    st.info("Módulo `apex_engine` não disponível ou não tem API compatível. Mostrando demo.")
    demo_steps = st.slider("Passos da demo", 10, 1000, 200)
    t = list(range(demo_steps))
    angle = [0.1 * i % 6.28 for i in t]
    velocity = [0.05 * i % 3.14 for i in t]
    torque = [0.01 * i % 1.0 for i in t]
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=t, y=angle, name="angle"))
    fig.add_trace(go.Scatter(x=t, y=velocity, name="velocity"))
    fig.add_trace(go.Scatter(x=t, y=torque, name="torque"))
    st.plotly_chart(fig, use_container_width=True)
    st.metric("Ciclos", len(t))
    st.metric("Reward médio", round(sum(angle) / len(angle), 3))
    st.metric("Watchdog timeouts", 0)
