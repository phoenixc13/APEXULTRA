"""
APEX Middleware v6 — Dashboard Web (Streamlit)

Versão web do frontend PySide6. Mesma engine (apex_engine.py),
mesmos dados, mas renderizados em plotly + componentes nativos Streamlit.

Acessível publicamente via share.streamlit.io após deploy.

Uso local:
    py -m streamlit run streamlit_app.py
"""

import time
import math
import threading
from collections import deque
from typing import List

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import streamlit as st

import apex_engine as eng
import apex_middleware_v6 as apex


# ==============================================================================
# CONFIGURAÇÃO DA PÁGINA
# ==============================================================================

st.set_page_config(
    page_title="APEX v6 ULTRA — Real-Time Dashboard",
    page_icon="[robot]",
    layout="wide",
    initial_sidebar_state="expanded",
)

# CSS customizado para tema escuro tipo PySide6
st.markdown("""
<style>
    .stApp { background-color: #0e1117; }
    .stSidebar { background-color: #1e2530; }
    .metric-card {
        background-color: #1e2530;
        border: 1px solid #2a3340;
        border-radius: 6px;
        padding: 10px;
        color: #c9d1d9;
    }
    .stMetric > div { background-color: #1e2530; padding: 8px; border-radius: 6px; }
    h1, h2, h3 { color: #00d4ff !important; }
    .stProgress > div > div > div > div { background-color: #00d4ff; }
</style>
""", unsafe_allow_html=True)


# ==============================================================================
# CONSTANTES VISUAIS
# ==============================================================================

NODE_COLORS = ['#00d4ff', '#ff6b6b', '#ffd93d', '#6bcf7f',
               '#c780fa', '#ff9a3c', '#5ad2ff', '#ff80ab']
FSM_COLORS = {0: '#6bcf7f', 1: '#ffd93d', 2: '#ff6b6b'}
FSM_NAMES = {0: 'NORMAL', 1: 'WARN', 2: 'SHUTDOWN'}

HISTORY_SECONDS = 10.0


# ==============================================================================
# ESTADO DA SESSÃO (singleton por utilizador)
# ==============================================================================

@st.cache_resource
def get_engine(num_nodes: int, hz: int):
    """Cria uma instância única do engine por sessão de browser."""
    engine = eng.ApexEngine(num_nodes=num_nodes, hz=hz)
    engine.start()
    return engine


def init_session_state(num_nodes: int, history_points: int):
    """Inicializa buffers circulares no session_state do Streamlit."""
    if "history" not in st.session_state:
        st.session_state.history = {}
        for i in range(num_nodes):
            nid = f"APEX-EDG-{i+1:02d}"
            st.session_state.history[nid] = {
                "t": deque(maxlen=history_points),
                "theta": deque(maxlen=history_points),
                "vel": deque(maxlen=history_points),
                "torque": deque(maxlen=history_points),
            }
    if "sample_buffer" not in st.session_state:
        st.session_state.sample_buffer = []
    if "lock" not in st.session_state:
        st.session_state.lock = threading.Lock()
    if "engine_started" not in st.session_state:
        st.session_state.engine_started = False
    if "faults" not in st.session_state:
        st.session_state.faults = {}


def samples_callback(samples: List[eng.ApexSample]):
    """Callback executado pela thread do engine a cada ciclo de controlo."""
    t_now = time.perf_counter()
    with st.session_state.lock:
        for s in samples:
            hist = st.session_state.history.get(s.node_id)
            if hist is None:
                continue
            hist["t"].append(t_now)
            hist["theta"].append(s.filt_angle_deg)
            hist["vel"].append(s.filt_vel_dps)
            hist["torque"].append(s.torque)
        st.session_state.sample_buffer.append((t_now, samples))


# ==============================================================================
# SIDEBAR — CONTROLOS
# ==============================================================================

def render_sidebar() -> dict:
    """Renderiza controlos e devolve a configuração atual."""
    with st.sidebar:
        st.title("[robot] APEX v6 ULTRA")
        st.caption("Middleware de controle robótico em tempo real")
        st.caption("STM32H743 @ 480MHz (simulado)")
        st.divider()

        st.subheader("Configuração")
        num_nodes = st.slider("Número de nós", 2, 8, 4)
        hz = st.select_slider("Frequência de controlo",
                              options=[25, 50, 100, 200],
                              value=100,
                              format_func=lambda x: f"{x} Hz")
        st.caption(f"Período: {1000/hz:.1f} ms")

        st.divider()
        st.subheader("Controlos")

        col1, col2 = st.columns(2)
        with col1:
            pause = st.button("⏸ PAUSE" if st.session_state.get("running", True)
                              else "▶ RETOMAR",
                              use_container_width=True)
        with col2:
            reset = st.button("↻ RESET", use_container_width=True)

        st.divider()
        st.subheader("Injeção de falhas")

        fault_choice = st.selectbox(
            "Tipo de falha",
            ["— Nenhuma —", "Brownout (VCC 3.05V)", "Corte de sensor", "Todos"],
            index=0,
        )

        inject_now = st.button("⚡ INJETAR", type="primary", use_container_width=True)

        st.divider()
        st.subheader("Info")
        st.markdown(f"""
        - **MCU alvo:** STM32H743
        - **Aritmética:** Q16.16 + INT8
        - **Comunicação:** CAN-FD
        - **Heap:** 0 bytes
        - **Watchdog:** IWDG independente
        """)

        return {
            "num_nodes": num_nodes,
            "hz": hz,
            "pause": pause,
            "reset": reset,
            "fault_choice": fault_choice,
            "inject_now": inject_now,
        }


# ==============================================================================
# PAINEL PRINCIPAL
# ==============================================================================

def render_header(engine: eng.ApexEngine):
    n = len(engine.nodes) if engine.nodes else 0
    cycles = engine.nodes[0].exec_cycles if engine.nodes else 0
    samples = len(st.session_state.get("sample_buffer", []))

    col1, col2, col3, col4 = st.columns(4)
    col1.metric("Nós ativos", n)
    col2.metric("Ciclos executados", f"{cycles:,}")
    col3.metric("Amostras coletadas", samples)
    col4.metric("Engine rodando",
                "Sim" if engine.is_running() else "Não",
                delta="ok" if engine.is_running() else "stop")


def render_pendulum(engine: eng.ApexEngine):
    """Visualização 2D do pêndulo (substitui widget customizado do PySide6)."""
    if not engine.nodes:
        st.info("Engine a inicializar...")
        return

    fig = go.Figure()
    n = len(engine.nodes)

    # Chão
    fig.add_shape(type="line", x0=-1.5, x1=n+0.5, y0=0, y1=0,
                  line=dict(color='#2a3340', width=3))

    for i, node in enumerate(engine.nodes):
        ang_rad = apex.apex_q16_to_float(node.last_clean_angle_q16)
        ang_rad = max(-1.2, min(1.2, ang_rad))
        color = FSM_COLORS.get(node.fsm.state, '#ffffff')
        x_pivot = i + 0.5
        y_pivot = 0
        length = 1.0
        x_tip = x_pivot + length * math.sin(ang_rad)
        y_tip = y_pivot + length * math.cos(ang_rad)

        # Haste
        fig.add_shape(type="line", x0=x_pivot, x1=x_tip,
                      y0=y_pivot, y1=y_tip,
                      line=dict(color=color, width=4))
        # Bola
        fig.add_trace(go.Scatter(
            x=[x_tip], y=[y_tip],
            mode='markers',
            marker=dict(size=18, color=color, line=dict(color='white', width=1)),
            showlegend=False, hoverinfo='skip',
        ))
        # Joint (carrinho)
        fig.add_shape(type="rect", x0=x_pivot-0.08, x1=x_pivot+0.08,
                      y0=-0.05, y1=0.05,
                      fillcolor='#1e2530', line=dict(color=color, width=2))
        # Label
        deg = math.degrees(ang_rad)
        fig.add_annotation(x=x_pivot, y=-0.20, text=f"<b>APEX-{i+1:02d}</b>",
                          showarrow=False, font=dict(color='#c9d1d9', size=11))
        fig.add_annotation(x=x_pivot, y=-0.32, text=f"{deg:+5.1f}°",
                          showarrow=False, font=dict(color=color, size=10))

    fig.update_xaxes(range=[-1.5, n+0.5], visible=False)
    fig.update_yaxes(range=[-0.5, 1.3], visible=False,
                     scaleanchor="x", scaleratio=0.8)
    fig.update_layout(
        title="Visualização do Pêndulo (cores por FSM: verde=OK, amarelo=WARN, vermelho=SHUTDOWN)",
        plot_bgcolor='#0e1117', paper_bgcolor='#0e1117',
        font=dict(color='#c9d1d9'),
        height=340, margin=dict(l=10, r=10, t=40, b=10),
    )
    st.plotly_chart(fig, use_container_width=True, key=f"pend_{n}")


def render_time_series(engine: eng.ApexEngine):
    """Gráficos de séries temporais em plotly."""
    if not engine.nodes:
        return

    now = time.perf_counter()
    t_min = now - HISTORY_SECONDS

    fig = make_subplots(
        rows=3, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.06,
        subplot_titles=("Ângulo Filtrado (graus)",
                        "Velocidade Angular (graus/s)",
                        "Torque Aplicado"),
    )

    for i, (nid, hist) in enumerate(st.session_state.history.items()):
        if i >= len(engine.nodes):
            break
        node = engine.nodes[i]
        color = NODE_COLORS[i % len(NODE_COLORS)]
        name = nid

        data = [(t, v) for t, v in zip(hist["t"], hist["theta"]) if t >= t_min]
        if data:
            ts, vals = zip(*data)
            ts_rel = [t - now for t in ts]
            fig.add_trace(go.Scatter(x=ts_rel, y=vals, mode='lines',
                                     name=name, line=dict(color=color, width=2),
                                     legendgroup=name, showlegend=True),
                          row=1, col=1)

        data = [(t, v) for t, v in zip(hist["t"], hist["vel"]) if t >= t_min]
        if data:
            ts, vals = zip(*data)
            ts_rel = [t - now for t in ts]
            fig.add_trace(go.Scatter(x=ts_rel, y=vals, mode='lines',
                                     name=name, line=dict(color=color, width=2),
                                     legendgroup=name, showlegend=False),
                          row=2, col=1)

        data = [(t, v) for t, v in zip(hist["t"], hist["torque"]) if t >= t_min]
        if data:
            ts, vals = zip(*data)
            ts_rel = [t - now for t in ts]
            fig.add_trace(go.Scatter(x=ts_rel, y=vals, mode='lines',
                                     name=name, line=dict(color=color, width=2),
                                     legendgroup=name, showlegend=False),
                          row=3, col=1)

    fig.update_xaxes(range=[-HISTORY_SECONDS, 0], title_text="t (s)",
                     gridcolor='#1e2530', color='#c9d1d9')
    fig.update_yaxes(gridcolor='#1e2530', color='#c9d1d9')
    fig.update_layout(
        plot_bgcolor='#0e1117', paper_bgcolor='#0e1117',
        font=dict(color='#c9d1d9'),
        height=600, margin=dict(l=60, r=20, t=40, b=40),
        legend=dict(orientation="h", yanchor="bottom", y=1.02,
                    xanchor="right", x=1),
    )
    st.plotly_chart(fig, use_container_width=True, key="timeseries")


def render_node_cards(engine: eng.ApexEngine):
    """Cartões de status por nó."""
    if not engine.nodes:
        return
    cols = st.columns(len(engine.nodes))
    for i, (node, col) in enumerate(zip(engine.nodes, cols)):
        with col:
            fsm_color = FSM_COLORS.get(node.fsm.state, '#ffffff')
            st.markdown(f"""
            <div class="metric-card">
                <h4 style="margin:0; color:#00d4ff;">{node.node_id}</h4>
                <p style="margin:4px 0; color:{fsm_color}; font-weight:bold;">
                    FSM: {FSM_NAMES.get(node.fsm.state, '?')}
                </p>
                <p style="margin:2px 0; font-family:monospace; font-size:13px;">
                    Latência: <b>{node.avg_latency_us:.1f} µs</b><br>
                    Reward: <b>{node.recent_reward:.3f}</b><br>
                    Annealing T: <b>{node.temperature:.2f}</b>
                </p>
            </div>
            """, unsafe_allow_html=True)


def render_system_info(engine: eng.ApexEngine):
    """Coluna direita: info CAN + Mesh + sistema."""
    if not engine.nodes:
        return

    col1, col2, col3 = st.columns(3)

    with col1:
        st.subheader("CAN-FD Bus")
        if engine.bus:
            stats = engine.bus.stats()
            st.metric("Mensagens dropadas", stats['dropped'])
            st.metric("Mailboxes ativos", stats['mailboxes'])

    with col2:
        st.subheader("Federated Mesh")
        avg_reward = sum(n.recent_reward for n in engine.nodes) / len(engine.nodes)
        st.metric("Reward médio", f"{avg_reward:.3f}")
        st.metric("Annealing T (nó 0)", f"{engine.nodes[0].temperature:.3f}")

    with col3:
        st.subheader("Watchdog")
        total_resets = sum(wd.resets for wd in engine.wdts)
        st.metric("Resets acumulados", total_resets,
                  delta=f"{total_resets} WDG timeouts",
                  delta_color="inverse" if total_resets > 0 else "off")


# ==============================================================================
# LOOP PRINCIPAL
# ==============================================================================

def main():
    # Renderiza sidebar e obtém config
    cfg = render_sidebar()
    num_nodes = cfg["num_nodes"]
    hz = cfg["hz"]

    # Inicializa estado
    history_points = int(HISTORY_SECONDS * hz)
    init_session_state(num_nodes, history_points)

    # Obtém/cria engine (cacheado)
    engine = get_engine(num_nodes, hz)

    # Registra callback apenas uma vez
    if not st.session_state.engine_started:
        engine.on_sample(samples_callback)
        st.session_state.engine_started = True
        st.session_state.running = True

    # Processa controlos
    if cfg["pause"]:
        if st.session_state.running:
            engine.pause()
            st.session_state.running = False
        else:
            engine.resume()
            st.session_state.running = True

    if cfg["reset"]:
        for i, n in enumerate(engine.nodes):
            n.kalman_ang.reset(0)
            n.kalman_vel.reset(0)
            n.integral_q16 = 0
            n.fsm.state = 0
            n.recent_reward = 0.5
            n.temperature = 8.0
            engine.hils[i].theta = 0.18 * (1 if i % 2 == 0 else -1) * (1 + 0.1 * i)
            engine.hils[i].omega = 0.0
        with st.session_state.lock:
            for nid in st.session_state.history:
                for k in st.session_state.history[nid]:
                    st.session_state.history[nid][k].clear()
            st.session_state.sample_buffer.clear()
        st.success("Simulação reiniciada")
        time.sleep(1)
        st.rerun()

    if cfg["inject_now"]:
        if "Brownout" in cfg["fault_choice"]:
            for hil in engine.hils:
                hil.set_vcc(3.05)
            st.warning("Brownout injetado (VCC=3.05V) — 2s")
            def _restore():
                for hil in engine.hils:
                    hil.set_vcc(3.30)
            threading.Timer(2.0, _restore).start()
        if "Corte" in cfg["fault_choice"]:
            for n in engine.nodes:
                n.kalman_ang.reset(0)
                n.kalman_vel.reset(0)
            st.warning("Sensor cortado em todos os nós")
        time.sleep(1)
        st.rerun()

    # Renderiza painéis
    render_header(engine)
    render_node_cards(engine)
    render_pendulum(engine)
    render_time_series(engine)
    render_system_info(engine)

    # Auto-refresh do dashboard
    time.sleep(0.1)
    st.rerun()


if __name__ == "__main__":
    main()
