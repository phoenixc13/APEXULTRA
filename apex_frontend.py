"""
APEX Frontend Desktop — PySide6 + pyqtgraph.

Visualização em tempo real do APEX Middleware v6:
- Gráfico de séries temporais (ângulo filtrado de cada nó)
- Visualização do pêndulo animado
- Painel de status (FSM, latência, recompensas, temperatura)
- Botões de controlo (pause, reset, injetar falhas)
- LEDs de status do barramento CAN

Uso:
    py apex_frontend.py
"""

import sys
import math
import time
from collections import deque
from typing import Dict, List

from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from apex_engine import ApexEngine, ApexSample


# ============== CONFIGURAÇÃO VISUAL ==============

NODE_COLORS = ['#00d4ff', '#ff6b6b', '#ffd93d', '#6bcf7f',
               '#c780fa', '#ff9a3c', '#5ad2ff', '#ff80ab']

FSM_COLORS = {
    0: '#6bcf7f',  # NORMAL verde
    1: '#ffd93d',  # WARN amarelo
    2: '#ff6b6b',  # SHUTDOWN vermelho
}

FSM_NAMES = {0: 'NORMAL', 1: 'WARN', 2: 'SHUTDOWN'}

HISTORY_SECONDS = 10.0  # janela temporal visível


# ============== WIDGET PRINCIPAL ==============

class ApexMainWindow(QtWidgets.QMainWindow):
    def __init__(self, engine: ApexEngine):
        super().__init__()
        self.engine = engine
        self.setWindowTitle(f"APEX v6.0 ULTRA — Real-Time Dashboard ({engine.num_nodes} nodes @ {engine.hz}Hz)")
        self.resize(1400, 800)

        # Buffers circulares para séries temporais (por nó)
        self.max_points = int(HISTORY_SECONDS * engine.hz)
        self.t_hist: deque = deque(maxlen=self.max_points)
        self.node_history: Dict[str, Dict[str, deque]] = {
            f"APEX-EDG-{i+1:02d}": {
                "theta": deque(maxlen=self.max_points),
                "vel":   deque(maxlen=self.max_points),
                "torque": deque(maxlen=self.max_points),
            } for i in range(engine.num_nodes)
        }
        self.sample_counter = 0
        self.fault_countdown = 0
        self.fault_type = None

        self._build_ui()

        # Conecta callback do engine
        self.engine.on_sample(self._on_samples)
        self.engine.start()

        # Timer para refresh da GUI (limitado para não saturar)
        self.refresh_timer = QtCore.QTimer()
        self.refresh_timer.timeout.connect(self._refresh_view)
        self.refresh_timer.start(50)  # 20 FPS de refresh visual

        # Atualiza labels a 4Hz
        self.status_timer = QtCore.QTimer()
        self.status_timer.timeout.connect(self._update_status_labels)
        self.status_timer.start(250)

    # ---------- CONSTRUÇÃO DA UI ----------

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # ============== BARRA SUPERIOR: título + controlos ==============
        top_bar = QtWidgets.QHBoxLayout()

        title = QtWidgets.QLabel("APEX MIDDLEWARE v6.0 ULTRA")
        title.setStyleSheet("""
            font-size: 20px; font-weight: bold; color: #00d4ff;
            padding: 4px 12px;
            border: 2px solid #00d4ff; border-radius: 6px;
        """)
        top_bar.addWidget(title)

        top_bar.addStretch()

        self.btn_pause = QtWidgets.QPushButton("⏸ PAUSE")
        self.btn_pause.setCheckable(True)
        self.btn_pause.setStyleSheet(self._btn_style('#5ad2ff'))
        self.btn_pause.toggled.connect(self._toggle_pause)
        top_bar.addWidget(self.btn_pause)

        self.btn_brownout = QtWidgets.QPushButton("⚡ INJETAR BROWNOUT")
        self.btn_brownout.setStyleSheet(self._btn_style('#ffd93d'))
        self.btn_brownout.clicked.connect(self._inject_brownout)
        top_bar.addWidget(self.btn_brownout)

        self.btn_sensor_cut = QtWidgets.QPushButton("🔌 CORTAR SENSOR")
        self.btn_sensor_cut.setStyleSheet(self._btn_style('#ff6b6b'))
        self.btn_sensor_cut.clicked.connect(self._inject_sensor_cut)
        top_bar.addWidget(self.btn_sensor_cut)

        self.btn_reset = QtWidgets.QPushButton("↻ RESET")
        self.btn_reset.setStyleSheet(self._btn_style('#6bcf7f'))
        self.btn_reset.clicked.connect(self._reset_sim)
        top_bar.addWidget(self.btn_reset)

        root.addLayout(top_bar)

        # ============== BODY: 3 colunas ==============
        body = QtWidgets.QHBoxLayout()

        # ---- Coluna esquerda: pêndulo + status ----
        left_col = QtWidgets.QVBoxLayout()
        left_col.setSpacing(6)

        # Pêndulo (widget custom)
        self.pendulum_widget = PendulumWidget(self.engine.num_nodes)
        left_col.addWidget(self.pendulum_widget, stretch=3)

        # Painel de status de cada nó
        self.status_panels: List[NodeStatusPanel] = []
        for i in range(self.engine.num_nodes):
            panel = NodeStatusPanel(f"APEX-EDG-{i+1:02d}")
            self.status_panels.append(panel)
            left_col.addWidget(panel)
        left_col.addStretch()

        body.addLayout(left_col, stretch=2)

        # ---- Coluna central: gráficos de séries temporais ----
        center_col = QtWidgets.QVBoxLayout()
        center_col.setSpacing(4)

        pg.setConfigOptions(antialias=True, background='#0e1117', foreground='#c9d1d9')

        # Gráfico 1: ângulo
        self.plot_theta = self._make_plot("Ângulo Filtrado (graus)", "graus")
        self.curves_theta = {}
        for i in range(self.engine.num_nodes):
            color = NODE_COLORS[i % len(NODE_COLORS)]
            c = self.plot_theta.plot(pen=pg.mkPen(color, width=2),
                                     name=f"APEX-EDG-{i+1:02d}")
            self.curves_theta[i] = c
        center_col.addWidget(self.plot_theta, stretch=1)

        # Gráfico 2: velocidade
        self.plot_vel = self._make_plot("Velocidade Angular (graus/s)", "graus/s")
        self.curves_vel = {}
        for i in range(self.engine.num_nodes):
            color = NODE_COLORS[i % len(NODE_COLORS)]
            c = self.plot_vel.plot(pen=pg.mkPen(color, width=2),
                                   name=f"APEX-EDG-{i+1:02d}")
            self.curves_vel[i] = c
        center_col.addWidget(self.plot_vel, stretch=1)

        # Gráfico 3: torque
        self.plot_torque = self._make_plot("Torque Aplicado", "u (controle)")
        self.curves_torque = {}
        for i in range(self.engine.num_nodes):
            color = NODE_COLORS[i % len(NODE_COLORS)]
            c = self.plot_torque.plot(pen=pg.mkPen(color, width=2),
                                      name=f"APEX-EDG-{i+1:02d}")
            self.curves_torque[i] = c
        center_col.addWidget(self.plot_torque, stretch=1)

        body.addLayout(center_col, stretch=4)

        # ---- Coluna direita: telemetria global ----
        right_col = QtWidgets.QVBoxLayout()
        right_col.setSpacing(6)

        can_group = QtWidgets.QGroupBox("CAN-FD Bus")
        can_layout = QtWidgets.QVBoxLayout(can_group)
        self.lbl_can_drops = QtWidgets.QLabel("Drops: 0")
        self.lbl_can_mailboxes = QtWidgets.QLabel("Mailboxes: 0")
        for lbl in (self.lbl_can_drops, self.lbl_can_mailboxes):
            lbl.setStyleSheet("font-size: 12px; padding: 4px;")
            can_layout.addWidget(lbl)
        right_col.addWidget(can_group)

        mesh_group = QtWidgets.QGroupBox("Federated Mesh")
        mesh_layout = QtWidgets.QVBoxLayout(mesh_group)
        self.lbl_mesh_avg_reward = QtWidgets.QLabel("Avg Reward: --")
        self.lbl_mesh_temperature = QtWidgets.QLabel("Annealing T: --")
        for lbl in (self.lbl_mesh_avg_reward, self.lbl_mesh_temperature):
            lbl.setStyleSheet("font-size: 12px; padding: 4px;")
            mesh_layout.addWidget(lbl)
        right_col.addWidget(mesh_group)

        sysinfo_group = QtWidgets.QGroupBox("System Info")
        sysinfo_layout = QtWidgets.QVBoxLayout(sysinfo_group)
        self.lbl_target_mcu = QtWidgets.QLabel(f"MCU: {apex_mcu_label()}")
        self.lbl_freq = QtWidgets.QLabel(f"Freq: {self.engine.hz} Hz")
        self.lbl_cycles = QtWidgets.QLabel("Cycles: 0")
        self.lbl_total_samples = QtWidgets.QLabel("Samples: 0")
        for lbl in (self.lbl_target_mcu, self.lbl_freq, self.lbl_cycles, self.lbl_total_samples):
            lbl.setStyleSheet("font-size: 12px; padding: 4px;")
            sysinfo_layout.addWidget(lbl)
        right_col.addWidget(sysinfo_group)

        right_col.addStretch()

        body.addLayout(right_col, stretch=1)

        root.addLayout(body, stretch=1)

        # ============== BARRA DE STATUS INFERIOR ==============
        self.statusbar = QtWidgets.QStatusBar()
        self.setStatusBar(self.statusbar)
        self.statusbar.showMessage("APEX Engine: inicializando...")

    def _make_plot(self, title: str, ylabel: str) -> pg.PlotWidget:
        plot = pg.PlotWidget()
        plot.setTitle(title, color='#c9d1d9', size='11pt')
        plot.setLabel('left', ylabel, color='#c9d1d9')
        plot.setLabel('bottom', 't (s)', color='#c9d1d9')
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.addLegend(offset=(10, 10))
        plot.setMouseEnabled(x=False, y=False)
        return plot

    def _btn_style(self, color: str) -> str:
        return f"""
            QPushButton {{
                background-color: #1e2530; color: {color};
                border: 2px solid {color}; border-radius: 4px;
                padding: 6px 12px; font-weight: bold; font-size: 11px;
            }}
            QPushButton:hover {{ background-color: #2a3340; }}
            QPushButton:checked {{ background-color: {color}; color: #0e1117; }}
        """

    # ---------- CALLBACKS DE CONTROLO ----------

    def _toggle_pause(self, checked: bool):
        if checked:
            self.engine.pause()
            self.statusbar.showMessage("⏸ PAUSADO")
        else:
            self.engine.resume()
            self.statusbar.showMessage("▶ RODANDO")

    def _inject_brownout(self):
        # Marca para o próximo ciclo brownout_start = cycle+1
        self.fault_countdown = max(self.fault_countdown, self.engine.wdts[0]._resets)
        self.fault_type = 'brownout'
        # Envia via mecanismo simples: esperar próximo ciclo e usar flag
        # Como simplificação, definimos fault_cycles diretamente via hot_path
        self.statusbar.showMessage("⚡ Brownout programado para próximo ciclo")
        # Implementação direta: aplica via HIL após pequeno delay
        QtCore.QTimer.singleShot(100, self._do_brownout)

    def _do_brownout(self):
        for hil in self.engine.hils:
            hil.set_vcc(3.05)
        QtCore.QTimer.singleShot(2000, self._end_brownout)
        self.statusbar.showMessage("⚡ BROWNOUT ATIVO — restabelecerá em 2s")

    def _end_brownout(self):
        for hil in self.engine.hils:
            hil.set_vcc(3.30)
        self.statusbar.showMessage("✓ VCC restaurado para 3.30V")

    def _inject_sensor_cut(self):
        for n in self.engine.nodes:
            n.kalman_ang.reset(0)
            n.kalman_vel.reset(0)
        self.statusbar.showMessage("🔌 SENSOR CORTADO em todos os nós")
        QtCore.QTimer.singleShot(2000, lambda: self.statusbar.showMessage("▶ RODANDO"))

    def _reset_sim(self):
        for i, n in enumerate(self.engine.nodes):
            n.kalman_ang.reset(0)
            n.kalman_vel.reset(0)
            n.integral_q16 = 0
            n.fsm.state = 0
            n.recent_reward = 0.5
            n.temperature = 8.0
            # Reset HIL
            self.engine.hils[i].theta = 0.18 * (1 if i % 2 == 0 else -1) * (1 + 0.1 * i)
            self.engine.hils[i].omega = 0.0
        self.statusbar.showMessage("↻ Simulação reiniciada")

    # ---------- CALLBACK DO ENGINE (chamado em thread separada) ----------

    def _on_samples(self, samples: List[ApexSample]):
        """Recebe samples da thread de simulação. Armazena em buffers thread-safe."""
        t_now = time.perf_counter()
        for s in samples:
            hist = self.node_history[s.node_id]
            hist["theta"].append((t_now, s.filt_angle_deg))
            hist["vel"].append((t_now, s.filt_vel_dps))
            hist["torque"].append((t_now, s.torque))
        self.sample_counter += 1

    # ---------- REFRESH DA GUI ----------

    def _refresh_view(self):
        # Corta buffers para a janela visível
        now = time.perf_counter()
        t_min = now - HISTORY_SECONDS

        for i, (nid, hist) in enumerate(self.node_history.items()):
            theta_data = [(t, v) for t, v in hist["theta"] if t >= t_min]
            if theta_data:
                ts, vals = zip(*theta_data)
                ts_rel = [t - now for t in ts]
                self.curves_theta[i].setData(ts_rel, vals)

            vel_data = [(t, v) for t, v in hist["vel"] if t >= t_min]
            if vel_data:
                ts, vals = zip(*vel_data)
                ts_rel = [t - now for t in ts]
                self.curves_vel[i].setData(ts_rel, vals)

            torque_data = [(t, v) for t, v in hist["torque"] if t >= t_min]
            if torque_data:
                ts, vals = zip(*torque_data)
                ts_rel = [t - now for t in ts]
                self.curves_torque[i].setData(ts_rel, vals)

        # Ajusta eixo X
        for plot in (self.plot_theta, self.plot_vel, self.plot_torque):
            plot.setXRange(-HISTORY_SECONDS, 0)

        # Pêndulo
        if self.engine.nodes:
            angles = [apex_q16_of_node(n) for n in self.engine.nodes]
            fsms = [n.fsm.state for n in self.engine.nodes]
            self.pendulum_widget.update_angles(angles, fsms)

    def _update_status_labels(self):
        # Labels de cada nó
        for i, panel in enumerate(self.status_panels):
            if i < len(self.engine.nodes):
                n = self.engine.nodes[i]
                panel.update(n.avg_latency_us, n.recent_reward, n.temperature,
                             n.fsm.state, self.engine.wdts[i].resets)

        # Telemetria CAN
        if self.engine.bus:
            stats = self.engine.bus.stats()
            self.lbl_can_drops.setText(f"Drops: {stats['dropped']}")
            self.lbl_can_mailboxes.setText(f"Mailboxes: {stats['mailboxes']}")

        # Mesh
        if self.engine.nodes:
            self.lbl_mesh_avg_reward.setText(
                f"Avg Reward: {sum(n.recent_reward for n in self.engine.nodes)/len(self.engine.nodes):.3f}")
            self.lbl_mesh_temperature.setText(
                f"Annealing T: {self.engine.nodes[0].temperature:.3f}")

        # System
        if self.engine.nodes:
            self.lbl_cycles.setText(f"Cycles: {self.engine.nodes[0].exec_cycles}")
        self.lbl_total_samples.setText(f"Samples: {self.sample_counter}")

        # Status bar
        if self.engine.is_running():
            self.statusbar.showMessage(f"▶ Rodando | {self.sample_counter} amostras coletadas")
        else:
            self.statusbar.showMessage("⏹ Engine parada")

    # ---------- FECHAR ----------

    def closeEvent(self, event):
        self.engine.stop()
        event.accept()


# ============== WIDGETS AUXILIARES ==============

class PendulumWidget(QtWidgets.QWidget):
    """Desenha pêndulos invertidos lado a lado com cores por estado FSM."""

    def __init__(self, num_nodes: int, parent=None):
        super().__init__(parent)
        self.num_nodes = num_nodes
        self.angles: List[float] = [0.0] * num_nodes
        self.fsms: List[int] = [0] * num_nodes
        self.setMinimumHeight(160)
        self.setStyleSheet("background-color: #0e1117; border: 1px solid #1e2530;")

    def update_angles(self, angles_rad: List[float], fsms: List[int]):
        self.angles = angles_rad
        self.fsms = fsms
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        w = self.width()
        h = self.height()

        # Base (chão)
        painter.setPen(QtGui.QPen(QtGui.QColor('#2a3340'), 2))
        base_y = int(h * 0.85)
        painter.drawLine(20, base_y, w - 20, base_y)
        # Hachura
        for x in range(20, w - 20, 10):
            painter.drawLine(x, base_y, x - 8, h - 4)

        # Cada pêndulo
        col_w = (w - 40) // self.num_nodes
        for i in range(self.num_nodes):
            cx = 20 + col_w * i + col_w // 2
            top_y = base_y
            length = min(h * 0.7, 140)

            ang = max(-1.2, min(1.2, self.angles[i]))
            # ângulo pequeno = pêndulo quase vertical
            # ângulo grande = pêndulo inclinado
            # Para visual: theta=0 -> vertical; theta=±π/2 -> horizontal
            tip_x = cx + length * math.sin(ang)
            tip_y = top_y - length * math.cos(ang)

            color = QtGui.QColor(FSM_COLORS.get(self.fsms[i], '#ffffff'))
            pen = QtGui.QPen(color, 4)
            painter.setPen(pen)
            painter.drawLine(int(cx), int(top_y), int(tip_x), int(tip_y))

            # Bola na ponta
            painter.setBrush(QtGui.QBrush(color))
            painter.setPen(QtGui.QPen(QtGui.QColor('#ffffff'), 1))
            painter.drawEllipse(QtCore.QPoint(int(tip_x), int(tip_y)), 7, 7)

            # Joint (carrinho)
            painter.setBrush(QtGui.QBrush(QtGui.QColor('#1e2530')))
            painter.setPen(QtGui.QPen(color, 2))
            painter.drawRect(int(cx) - 12, int(top_y) - 4, 24, 8)

            # Label
            painter.setPen(QtGui.QPen(QtGui.QColor('#c9d1d9'), 1))
            f = painter.font()
            f.setPointSize(9)
            f.setBold(True)
            painter.setFont(f)
            label = f"APEX-{i+1:02d}"
            tw = painter.fontMetrics().horizontalAdvance(label)
            painter.drawText(int(cx - tw/2), int(h - 8), label)

            # Ângulo em graus
            f.setPointSize(8)
            f.setBold(False)
            painter.setFont(f)
            deg = math.degrees(ang)
            deg_txt = f"{deg:+6.1f}°"
            tw = painter.fontMetrics().horizontalAdvance(deg_txt)
            painter.drawText(int(cx - tw/2), int(top_y - 10), deg_txt)


class NodeStatusPanel(QtWidgets.QFrame):
    """Cartão pequeno de status por nó."""

    def __init__(self, node_id: str, parent=None):
        super().__init__(parent)
        self.node_id = node_id
        self.setStyleSheet("""
            QFrame {
                background-color: #1e2530;
                border: 1px solid #2a3340;
                border-radius: 6px;
            }
        """)
        self.setFixedHeight(70)

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)

        # ID + LED
        left = QtWidgets.QVBoxLayout()
        self.lbl_id = QtWidgets.QLabel(node_id)
        self.lbl_id.setStyleSheet("color: #c9d1d9; font-weight: bold; font-size: 12px;")
        left.addWidget(self.lbl_id)

        self.lbl_fsm = QtWidgets.QLabel("FSM: NORMAL")
        self.lbl_fsm.setStyleSheet("color: #6bcf7f; font-size: 10px;")
        left.addWidget(self.lbl_fsm)

        layout.addLayout(left)

        layout.addStretch()

        # Métricas
        right = QtWidgets.QVBoxLayout()
        self.lbl_lat = QtWidgets.QLabel("Lat:    0.0 µs")
        self.lbl_lat.setStyleSheet("color: #c9d1d9; font-size: 11px; font-family: monospace;")
        self.lbl_reward = QtWidgets.QLabel("Reward: 0.50")
        self.lbl_reward.setStyleSheet("color: #c9d1d9; font-size: 11px; font-family: monospace;")
        self.lbl_temp = QtWidgets.QLabel("T:    25.0°C  RST: 0")
        self.lbl_temp.setStyleSheet("color: #c9d1d9; font-size: 11px; font-family: monospace;")

        for lbl in (self.lbl_lat, self.lbl_reward, self.lbl_temp):
            lbl.setAlignment(QtCore.Qt.AlignRight)
            right.addWidget(lbl)

        layout.addLayout(right)

    def update(self, lat_us: float, reward: float, temperature: float,
               fsm_state: int, wdt_resets: int):
        self.lbl_lat.setText(f"Lat:   {lat_us:6.1f} µs")
        self.lbl_reward.setText(f"Reward: {reward:5.2f}")
        self.lbl_temp.setText(f"T:{temperature:5.1f}°C  RST:{wdt_resets}")
        self.lbl_fsm.setText(f"FSM: {FSM_NAMES.get(fsm_state, '?')}")
        self.lbl_fsm.setStyleSheet(f"color: {FSM_COLORS.get(fsm_state, '#ffffff')}; font-size: 10px; font-weight: bold;")


# ============== UTILITÁRIOS ==============

def apex_q16_of_node(node) -> float:
    """Converte last_clean_angle_q16 do nó para float (rad)."""
    return node.last_clean_angle_q16 / (1 << 16)


def apex_mcu_label() -> str:
    return "STM32H743 @ 480MHz"


# ============== ENTRYPOINT ==============

def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')

    # Estilo dark global
    app.setStyleSheet("""
        QMainWindow, QWidget { background-color: #0e1117; color: #c9d1d9; }
        QGroupBox {
            border: 1px solid #2a3340; border-radius: 6px;
            margin-top: 8px; padding-top: 8px; font-weight: bold;
        }
        QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
        QStatusBar { background-color: #1e2530; color: #00d4ff; }
    """)

    engine = ApexEngine(num_nodes=4, hz=100)
    window = ApexMainWindow(engine)

    window.show()

    try:
        exit_code = app.exec()
    finally:
        engine.stop()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()