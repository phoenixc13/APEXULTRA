"""Teste de smoke: abre a janela, espera 2s para coletar dados, captura screenshot e fecha."""
import sys
import time
from PySide6 import QtCore, QtWidgets, QtGui

sys.path.insert(0, r"C:\Users\User\apex_v6")
from apex_engine import ApexEngine
from apex_frontend import ApexMainWindow

app = QtWidgets.QApplication(sys.argv)
app.setStyle('Fusion')
app.setStyleSheet("""
    QMainWindow, QWidget { background-color: #0e1117; color: #c9d1d9; }
    QGroupBox {
        border: 1px solid #2a3340; border-radius: 6px;
        margin-top: 8px; padding-top: 8px; font-weight: bold;
    }
    QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
    QStatusBar { background-color: #1e2530; color: #00d4ff; }
""")

engine = ApexEngine(num_nodes=4, hz=50)  # 50Hz para poupar CPU
window = ApexMainWindow(engine)
window.resize(1400, 800)
window.show()

# Aguarda 6s para acumular dados nas séries temporais
def capture():
    pixmap = window.grab()
    pixmap.save(r"C:\Users\User\apex_v6\apex_screenshot.png")
    print(f"Screenshot salvo: {pixmap.width()}x{pixmap.height()}")
    print(f"Samples coletados: {window.sample_counter}")
    print(f"Estado no[0]: ang_filt={window.engine.nodes[0].last_clean_angle_q16/(1<<16):.3f} rad")
    print(f"FSM: {[n.fsm.state for n in window.engine.nodes]}")
    print(f"Latencia: {[f'{n.avg_latency_us:.1f}us' for n in window.engine.nodes]}")
    engine.stop()
    app.quit()

QtCore.QTimer.singleShot(6000, capture)
sys.exit(app.exec())