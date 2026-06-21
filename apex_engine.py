"""
APEX Engine — Loop de simulação desacoplado do frontend.

Executa o mesmo middleware v6 numa thread separada e emite amostras de
telemetria via callback (signal Qt no frontend, ou print no modo headless).

API pública:
    ApexEngine(num_nodes=4, hz=100, ...)
    engine.start()           # inicia a thread
    engine.stop()            # para a thread
    engine.on_sample(cb)     # registra callback: cb(samples: List[Sample])
    engine.pause()/resume()  # controlo de execução
"""

import time
import math
import random
import threading
from dataclasses import dataclass
from typing import List, Callable, Optional

# Importa tudo do middleware principal (sem o main async)
import apex_middleware_v6 as apex


@dataclass(slots=True)
class ApexSample:
    """Amostra de telemetria de UM nó num instante de tempo."""
    cycle: int
    node_id: str
    raw_angle_deg: float
    filt_angle_deg: float
    filt_vel_dps: float
    torque: float
    fsm_state: int
    latency_us: float
    reward: float
    temperature_c: float
    vcc_v: float
    wdt_resets: int


class ApexEngine:
    """Loop de simulação que executa em thread daemon e emite amostras."""

    def __init__(self, num_nodes: int = 4, hz: int = 100,
                 fault_cycles: Optional[dict] = None):
        self.num_nodes = num_nodes
        self.hz = hz
        self.period_s = 1.0 / hz
        self.fault_cycles = fault_cycles or {}

        self._stop = threading.Event()
        self._pause = threading.Event()
        self._pause.set()  # começa rodando
        self._thread: Optional[threading.Thread] = None

        self._callbacks: List[Callable[[List[ApexSample]], None]] = []
        self._cb_lock = threading.Lock()

        # Componentes da simulação (criados no start())
        self.nodes: List[apex.ApexNode] = []
        self.hils: List[apex.ApexHIL] = []
        self.noises: List[apex.ApexNoiseGen] = []
        self.bus: Optional[apex.ApexCANBus] = None
        self.mesh: Optional[apex.ApexMesh] = None
        self.telemetry: Optional[apex.ApexTelemetryRecorder] = None
        self.wdts: List[apex.ApexWatchdog] = []

    # ----------- API PÚBLICA -----------

    def on_sample(self, cb: Callable[[List[ApexSample]], None]):
        """Registra callback que recebe lista de samples (um por nó) a cada ciclo."""
        with self._cb_lock:
            self._callbacks.append(cb)

    def start(self):
        if self._thread is not None:
            return
        self._init_components()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True,
                                        name="APEX-Engine")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        # Para watchdogs
        for wd in self.wdts:
            wd.stop()

    def pause(self):
        self._pause.clear()

    def resume(self):
        self._pause.set()

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ----------- INTERNO -----------

    def _init_components(self):
        self.bus = apex.ApexCANBus(mailbox_capacity=256)

        for i in range(self.num_nodes):
            nid = f"APEX-EDG-{i+1:02d}"
            self.nodes.append(apex.ApexNode(nid, self.bus))
            init_theta = 0.18 * (1 if i % 2 == 0 else -1) * (1 + 0.1 * i)
            self.hils.append(apex.ApexHIL(init_theta_rad=init_theta))
            self.noises.append(apex.ApexNoiseGen(seed=0xC0FFEE + i,
                                                 spike_prob=0.06, spike_amp=1.5))

        self.mesh = apex.ApexMesh(self.nodes, self.bus)
        self.telemetry = apex.ApexTelemetryRecorder("apex_telemetry.csv")
        self.telemetry.start()

        for n in self.nodes:
            wd = apex.ApexWatchdog(n, timeout_ms=50)
            wd.start()
            self.wdts.append(wd)

    def _emit(self, samples: List[ApexSample]):
        with self._cb_lock:
            cbs = list(self._callbacks)
        for cb in cbs:
            try:
                cb(samples)
            except Exception as e:
                # Não derruba a simulação por causa do frontend
                print(f"[APEX-Engine] callback error: {e}", flush=True)

    def _run_loop(self):
        cycle = 0
        last_consensus_cycle = 0
        dt_q16 = apex.apex_q16_from_float(self.period_s)
        obs_mbox = self.bus.subscribe(0x180)

        while not self._stop.is_set():
            # Pausa controlada
            self._pause.wait()

            cycle += 1
            t0 = time.perf_counter()

            # Ambiente
            temp_q16 = apex.apex_q16_from_float(25.0 + 0.05 * cycle)
            for i, hil in enumerate(self.hils):
                hil.set_thermal(apex.apex_q16_to_float(temp_q16))
                # Brownout momentâneo
                if self.fault_cycles.get("brownout_start") == cycle:
                    hil.set_vcc(3.05)
                elif self.fault_cycles.get("brownout_end") == cycle:
                    hil.set_vcc(3.30)

            # Sensor cut
            sensor_cut_active = (cycle == self.fault_cycles.get("sensor_cut"))

            samples: List[ApexSample] = []

            for i, n in enumerate(self.nodes):
                hil = self.hils[i]

                if sensor_cut_active and i == 0:
                    raw_ang_q16 = 0
                    raw_vel_q16 = 0
                else:
                    raw_ang_q16, raw_vel_q16 = hil.read_sensors_q16()
                    raw_ang_q16 = self.noises[i].inject_q16(raw_ang_q16)

                _, torque_q16, safety = n.hot_path(raw_ang_q16, raw_vel_q16,
                                                  dt_q16, temp_q16)

                # HIL sub-steps
                torque = apex.apex_q16_to_float(torque_q16)
                for _ in range(5):
                    hil.step(torque, wind=random.uniform(-0.05, 0.05))

                # CAN publish
                payload = apex.ApexSerial.pack(
                    seq=n.exec_cycles,
                    ts_ns=int(time.perf_counter() * 1e9),
                    ang_q16=n.last_clean_angle_q16,
                    vel_q16=n.last_clean_vel_q16,
                    torque_q16=n.last_torque_q16,
                    safety=safety,
                )
                self.bus.publish(apex.ApexCANFrame(
                    cob_id=0x180 + i, rtr=False,
                    dlc=len(payload), data=payload,
                    ts_us=int(t0 * 1_000_000)))

                n.exec_cycles += 1

                # Aprendizado off-band
                n.learn_step(n.last_clean_angle_q16)

                # Latência
                t1 = time.perf_counter()
                lat = (t1 - t0) * 1_000_000.0
                n.avg_latency_us = n.avg_latency_us * 0.9 + lat * 0.1

                # Monta sample
                samples.append(ApexSample(
                    cycle=cycle,
                    node_id=n.node_id,
                    raw_angle_deg=math.degrees(apex.apex_q16_to_float(hil.theta)),
                    filt_angle_deg=math.degrees(apex.apex_q16_to_float(n.last_clean_angle_q16)),
                    filt_vel_dps=math.degrees(apex.apex_q16_to_float(n.last_clean_vel_q16)),
                    torque=apex.apex_q16_to_float(n.last_torque_q16),
                    fsm_state=safety,
                    latency_us=n.avg_latency_us,
                    reward=n.recent_reward,
                    temperature_c=apex.apex_q16_to_float(temp_q16),
                    vcc_v=3.3,  # HIL não expõe diretamente; placeholder
                    wdt_resets=self.wdts[i].resets,
                ))

            # Consenso federado a cada 10 ciclos
            if cycle - last_consensus_cycle >= 10:
                self.mesh.consensus(drop_rate=0.10)
                last_consensus_cycle = cycle

            # Telemetria CSV
            sched_us = int(t0 * 1_000_000)
            actual_us = int(time.perf_counter() * 1_000_000)
            jitter_us = abs(actual_us - sched_us)
            self.telemetry.record(cycle, sched_us, actual_us, jitter_us,
                                  int(self.nodes[0].avg_latency_us),
                                  self.wdts[0].resets,
                                  self.nodes[0].fsm.state)

            # Emite samples para o frontend
            self._emit(samples)

            # Compensação de jitter
            elapsed = time.perf_counter() - t0
            sleep_s = self.period_s - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

        # Cleanup
        if self.telemetry is not None:
            self.telemetry.close()


if __name__ == "__main__":
    # Modo headless de teste (sem GUI)
    eng = ApexEngine(num_nodes=3, hz=50)
    count = [0]

    def cb(samples):
        count[0] += 1
        if count[0] % 10 == 0:
            s = samples[0]
            print(f"cycle={s.cycle:4d} theta={s.filt_angle_deg:6.2f}deg "
                  f"lat={s.latency_us:5.1f}us reward={s.reward:.2f} fsm={s.fsm_state}")

    eng.on_sample(cb)
    eng.start()
    try:
        time.sleep(3.0)
    finally:
        eng.stop()
        print(f"\n{count[0]} ciclos emitidos em ~3s")