import os
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np


# Este try hace que se arranque la GUI aunque no esté conectada la estación de tierra
try:
    import serial
except Exception:
    serial = None


# CONFIGURACIÓN BÁSICA
EVENT_ERROR = "ERROR/ALARMA"
EVENT_CMD = "COMANDO"
EVENT_OBS = "OBSERVACIÓN"

ALL_TYPES = [EVENT_ERROR, EVENT_CMD, EVENT_OBS]

DEFAULT_PORT = "COM6"
BAUDRATE = 9600

# Mapa de comandos          DOCUMENTO !!!!!!!!!!!!!!!!
ACK = {
    # GLOBAL
    (0, 1): [101],  #iniciar
    (0, 2): [102],  #parar
    (0, 3): [103, 104, 105],#

    # DHT
    (1, 1): [200],
    (1, 2): [201],
    (1, 3): [202, 203],
    (1, 4): [204],
    (1, 5): [205],

    # SERVO
    (2, 1): [300, 301],          # mover ángulo manual OK / ERR
    (2, 2): [302],               # auto OFF
    (2, 3): [303],               # auto ON
    (2, 4): [304, 305, 306],     # telem off / set / err
    (2, 5): [307, 308],          # tiempo ciclo 180º okei / ERR al aplicarlo
    
    # DIST
    (3, 1): [400],
    (3, 2): [401],
    (3, 3): [402, 403],
}


# LOGGER (a fichero + notificación a la UI)
class Logger:
    def __init__(self, filepath: str):
        self.filepath = filepath
        self.lock = threading.Lock()
        self.subscribers = []

        # Asegurar que exista el fichero (sin truncar)
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
        except Exception:
            pass
        if not os.path.exists(filepath):
            with open(filepath, "w", encoding="utf-8") as f:
                f.write("")

    def subscribe(self, callback):
        self.subscribers.append(callback)

    def log(self, tipo: str, texto: str):
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        linea = f"{tipo} | {ts} | {texto}\n"

        with self.lock:
            with open(self.filepath, "a", encoding="utf-8") as f:
                f.write(linea)
                f.flush()

        for cb in self.subscribers:
            cb(linea)

    def read_all(self) -> str:
        with self.lock:
            if not os.path.exists(self.filepath):
                return ""
            with open(self.filepath, "r", encoding="utf-8") as f:
                return f.read()


# SERIAL MANAGER
class SerialManager:
    
    # Envia comandos tipo "grupo:codigo:valor|"
    # Si no se puede abrir el puerto, entra en modo demo
    
    def __init__(self, port: str, baudrate: int, logger: Logger):
        self.logger = logger
        self.port = port
        self.baudrate = baudrate
        self.com = None
        self.demo_mode = False
        self._rx_thread = None
        self._rx_running = False
        self.th_model = None
        self.radar_model = None

        
        # reintentos de envio hasta escuchar ecos
        self.ack_lock = threading.Lock()
        self.pending_acks = {}  # token -> {"event": Event, "expected": set(int), "got": int|None}
        self.ack_token = 0

        # Config por defecto
        self.ACK_TIMEOUT_S = 1.0
        self.ACK_MAX_RETRIES = 3



        if serial is None:
            self.demo_mode = True
            self.logger.log(EVENT_ERROR, "PySerial no está disponible. Modo DEMO activado.")
            return

        try:
            self.com = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            self.logger.log(EVENT_OBS, f"Puerto serie abierto: {port} @ {baudrate}")
        except Exception as e:
            self.demo_mode = True
            self.com = None
            self.logger.log(EVENT_ERROR, f"No se pudo abrir {port}. Modo DEMO activado. Detalle: {e}")

    
    def send_cmd(self, grupo: int, codigo: int, valor: int = 0, descripcion: str = ""):
        frame = f"{grupo}:{codigo}:{int(valor)}|"
        if self.demo_mode:
            # Simula envío
            self.logger.log(EVENT_CMD, f"[DEMO] {descripcion or ('Enviado ' + frame.strip())}")
            return

        try:
            self.com.write(frame.encode("ascii"))
            self.logger.log(EVENT_CMD, descripcion or f"Enviado {frame.strip()}")
        except Exception as e:
            self.logger.log(EVENT_ERROR, f"Error enviando {frame.strip()}: {e}")
    
    def _new_ack_token(self) -> int:
        with self.ack_lock:
            self.ack_token += 1
            return self.ack_token

    def _ack_register(self, expected_codes):
        token = self._new_ack_token()
        ev = threading.Event()
        with self.ack_lock:
            self.pending_acks[token] = {"event": ev, "expected": set(expected_codes), "got": None}
        return token, ev

    def _ack_resolve_if_match(self, code: int):
        # Cuando entra un STATUS code, comprobar si desbloquea algún envío pendiente
        with self.ack_lock:
            for token, info in list(self.pending_acks.items()):
                if code in info["expected"]:
                    info["got"] = code
                    info["event"].set()

    def _ack_cleanup(self, token: int):
        with self.ack_lock:
            if token in self.pending_acks:
                del self.pending_acks[token]

    def send_cmd_with_ack(self, grupo: int, codigo: int, valor: int = 0,
                          expected_status_codes=None,
                          descripcion: str = "",
                          timeout_s: float = None,
                          max_retries: int = None) -> bool:
        
        # Envía comando en texto a Tierra (g:c:v|) y espera eco de STATUS del satélite (1:CODE|).
        # Reintenta si no llega en timeout_s, hasta max_retries.
        # Devuelve True si llegó ACK, False si agotó reintentos.
        

        if expected_status_codes is None:
            # Si no defines qué eco esperar, manda “a pelo”
            self.send_cmd(grupo, codigo, valor, descripcion)
            return True

        timeout_s = self.ACK_TIMEOUT_S if timeout_s is None else float(timeout_s)
        max_retries = self.ACK_MAX_RETRIES if max_retries is None else int(max_retries)

        if self.demo_mode:
            # Simula que siempre llega ACK
            self.logger.log(EVENT_CMD, f"[DEMO ACK] {descripcion or f'{grupo}:{codigo}:{valor}'} -> OK")
            return True

        token, ev = self._ack_register(expected_status_codes)

        try:
            for attempt in range(1, max_retries + 1):
                # Enviar
                self.send_cmd(grupo, codigo, valor, descripcion + (f" (intento {attempt}/{max_retries})" if descripcion else ""))

                # Esperar eco
                ok = ev.wait(timeout=timeout_s)
                if ok:
                    with self.ack_lock:
                        got = self.pending_acks.get(token, {}).get("got", None)
                    self.logger.log(EVENT_CMD, f"ACK recibido: {got} para {grupo}:{codigo}:{valor}")
                    return True

                self.logger.log(EVENT_ERROR, f"Sin ACK en {timeout_s:.1f}s para {grupo}:{codigo}:{valor} -> reintento")

            self.logger.log(EVENT_ERROR, f"FALLO: sin ACK tras {max_retries} intentos ({grupo}:{codigo}:{valor})")
            return False
        finally:
            self._ack_cleanup(token)


    def start_reader(self):
        if self.demo_mode:
            self._rx_running = True
            self._rx_thread = threading.Thread(target=self._demo_loop, daemon=True)
            self._rx_thread.start()
            return

        if not self.com:
            return

        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def stop_reader(self):
        self._rx_running = False

    
    def _rx_loop(self):
        buffer = ""
        while self._rx_running:
            try:
                chunk = self.com.read(128)
                if not chunk:
                    continue
                buffer += chunk.decode("ascii", errors="ignore")

                while "|" in buffer:
                    frame, buffer = buffer.split("|", 1)
                    frame = frame.strip()
                    if frame:
                        self._handle_frame(frame)
            except Exception as e:
                self.logger.log(EVENT_ERROR, f"Error RX: {e}")
                time.sleep(0.2)

    
    def _handle_frame(self, frame: str):
        frame = frame.strip()
        if not frame:
            return

        # 1) Trama especial de checksum en Tierra
        if frame == "CHKERR":
            self.logger.log(EVENT_ERROR, "CHKERR recibido desde Tierra (checksum enlace Tierra<->Sat)")
            if self.th_model:
                self.th_model.add_error_sample()
            return

        # 2) Errores tipo "-2|" "-5|" (sin ':')
        if ":" not in frame:
            try:
                code = int(frame)
                # Son ids negativos del satélite (-2, -3, -5, -6)
                if code < 0:
                    self.logger.log(EVENT_ERROR, f"Error satélite id={code}")
                    if self.th_model:
                        self.th_model.add_error_sample()
                    return
            except Exception:
                self.logger.log(EVENT_ERROR, f"Trama inválida: '{frame}'")
                if self.th_model:
                    self.th_model.add_error_sample()
                return

        # 3) Tramas con ':' -> puede ser DATA (id:valor) o STATUS (1:code)
        try:
            tipo_s, val_s = frame.split(":", 1)
            tipo = int(tipo_s.strip())
            val_s = val_s.strip()
        except Exception:
            self.logger.log(EVENT_ERROR, f"Trama inválida: '{frame}'")
            if self.th_model:
                self.th_model.add_error_sample()
            return

        # STATUS: Tierra reenvía como "1:CODE"
        try:
            # Si es entero puro, lo tratamos como status candidate
            if all(ch in "+-0123456789" for ch in val_s) and len(val_s) > 0:
                code = int(val_s)
                if tipo == 1 and code >= 100:
                    self.logger.log(EVENT_OBS, f"STATUS satélite: {code}")
                    self._ack_resolve_if_match(code)
                    return
        except Exception:
            pass

        # 3b) DATA normal: id:float / id:int
        try:
            val = float(val_s)
        except Exception:
            self.logger.log(EVENT_ERROR, f"Trama inválida (valor no numérico): '{frame}'")
            if self.th_model:
                self.th_model.add_error_sample()
            return

        if not self.th_model:
            return

        # 2 humedad, 3 temperatura, 4 media, 5 distancia, 6 ángulo (byte)
        if tipo == 2:
            self.th_model.add_humidity(val)
        elif tipo == 3:
            self.th_model.add_temperature_and_commit(val)
        elif tipo == 4:
            self.th_model.add_mean_from_sat(val)
        elif tipo == 5:
            # distancia
            if self.radar_model:
                self.radar_model.add_distance_cm(val)
        elif tipo == 6:
            # ángulo real del servo (0..180)
            if self.radar_model:
                self.radar_model.add_angle_meas_0_180(val)
        else:
            pass


    
    def _demo_loop(self):
        t = 0.0
        while self._rx_running:
            time.sleep(0.5)
            t += 0.5
            if self.th_model:
                temp = 20 + 2*np.sin(t/3)
                hum = 55 + 10*np.sin(t/5)
                self.th_model.add_humidity(hum)
                self.th_model.add_temperature_and_commit(temp)

    
    def close(self):
        self.stop_reader()
        try:
            if self.com:
                self.com.close()
        except Exception:
            pass

        
# Intérprete de las gráficas del DHT
class THModel:
    
    # Guarda muestras T/H con media móvil (10) y ventana temporal.
    
    def __init__(self, ventana_tiempo=60.0): #segundos que se muestran en las graficas
        self.lock = threading.Lock()
        self.temperaturas = []
        self.humedades = []
        self.medias_10 = []
        self.tiempos = []

        self.temp = 0.0
        self.hum = 0.0
        self.t0 = None
        self.VENTANA_TIEMPO = ventana_tiempo

        self.temp_buffer = None
        self.hum_buffer = None

        self.mean_from_sat = False
        self.mean_sat_latest = np.nan
        self.mean_sat_last_valid = np.nan


    def commit_if_ready(self):
        # En el momento que llega una temperatura válida se ejecuta

        with self.lock:
            if self.temp_buffer is None:
                return

            self.temp = self.temp_buffer
            if self.hum_buffer is not None:
                self.hum = self.hum_buffer

            if self.t0 is None:
                self.t0 = time.time()

            t_rel = time.time() - self.t0

            self.tiempos.append(t_rel)
            self.temperaturas.append(self.temp)
            self.humedades.append(self.hum)

            # Media: satélite o Python (últimos 10 valores)
            if self.mean_from_sat:
                self.medias_10.append(self.mean_sat_last_valid)

            else:
                ult = [v for v in self.temperaturas[-10:] if v is not None and not np.isnan(v)]
                self.medias_10.append(sum(ult) / len(ult) if ult else np.nan)


    def add_humidity(self, h: float):
        with self.lock:
            self.hum_buffer = h

    def add_temperature_and_commit(self, t: float):
        with self.lock:
            self.temp_buffer = t
        self.commit_if_ready()

    def add_error_sample(self):
        # añade el valor nulo en la gráfica para que nos se pare
        with self.lock:
            if self.t0 is None:
                self.t0 = time.time()
            t_rel = time.time() - self.t0
            self.tiempos.append(t_rel)
            self.temperaturas.append(np.nan)
            self.humedades.append(np.nan)
            self.medias_10.append(np.nan)

    def set_mean_source(self, from_sat: bool):
        with self.lock:
            self.mean_from_sat = from_sat


    def add_mean_from_sat(self, m: float):
        with self.lock:
            self.mean_sat_latest = m
            self.mean_sat_last_valid = m


    def snapshot(self):

        # Devuelve una copia coherente para pintar sin bloquear demasiado.

        with self.lock:
            return (
                self.t0,
                self.VENTANA_TIEMPO,
                list(self.tiempos),
                list(self.temperaturas),
                list(self.humedades),
                list(self.medias_10),
                float(self.temp),
                float(self.hum),
            )

# Estimación del ángulo del radar
class RadarModel:
    
    # Estima ángulo del servo en grados [0..180] usando:
    #  - modelo cinemático con rebote (0<->180)
    #  - omega = 180 / tiempo_ciclo (deg/s)
    #  - corrección cuando llega medición real (id=6)

    # También guarda puntos (angulo, distancia) para dibujar el radar.
    
    def __init__(self, tiempo_ciclo_ms=3000, max_points=400):
        self.lock = threading.Lock()

        # Estado estimado
        self.ang_est = 90.0
        self.dir = +1              # +1 sube, -1 baja
        self.t_last = time.time()

        # Parámetros
        self.tiempo_ciclo_ms = float(tiempo_ciclo_ms)
        self.max_points = int(max_points)

        # “Filtro” simple (ganancias)
        self.k_corr = 0.35         # cuánto corriges el ángulo cuando llega medición
        self.k_dir = 0.20          # cuánto adaptas la dirección con medición

        # Última medición real
        self.ang_meas = None
        self.t_meas = None

        # Buffer de puntos para el radar: (theta_rad, r_cm, timestamp)
        self.points = []

        # Escalado
        self.rmax = 200.0          # cm inicial (autoajustable)

    def set_tiempo_ciclo_ms(self, ms: int):
        with self.lock:
            self.tiempo_ciclo_ms = max(200.0, float(ms))

    def set_manual_angle(self, ang_deg_0_180: float):
        # Cuando mandas servo manual desde GUI, puedes “resetear” el estimador
        with self.lock:
            self.ang_est = float(np.clip(ang_deg_0_180, 0.0, 180.0))
            self.t_last = time.time()

    def _omega_deg_s(self) -> float:
        # 180 grados en tiempo_ciclo_ms
        return 180.0 / (self.tiempo_ciclo_ms / 1000.0)

    def _propagate_nolock(self, now: float):
        dt = now - self.t_last
        if dt <= 0:
            return
        self.t_last = now

        omega = self._omega_deg_s()
        ang = self.ang_est + self.dir * omega * dt

        # Rebote 0..180 como el satélite
        while ang < 0.0 or ang > 180.0:
            if ang > 180.0:
                ang = 180.0 - (ang - 180.0)
                self.dir = -1
            elif ang < 0.0:
                ang = -ang
                self.dir = +1

        self.ang_est = ang

    def get_angle_est(self) -> float:
        with self.lock:
            now = time.time()
            self._propagate_nolock(now)
            return float(self.ang_est)

    def add_distance_cm(self, dist_cm: float):
        with self.lock:
            now = time.time()
            self._propagate_nolock(now)

            ang = float(self.ang_est)
            theta = np.deg2rad(ang)  # radar en [0..pi] si quieres semicírculo
            self.points.append((theta, float(dist_cm), now))
            if len(self.points) > self.max_points:
                self.points = self.points[-self.max_points:]

            # autoescalado suave
            valid = [r for (_, r, _) in self.points if np.isfinite(r) and r > 0]
            if valid:
                target = max(valid) * 1.10
                target = float(np.clip(target, 30.0, 700.0))
                self.rmax = 0.85 * self.rmax + 0.15 * target

    def add_angle_meas_0_180(self, ang_meas: float):
        with self.lock:
            now = time.time()
            self._propagate_nolock(now)

            ang_meas = float(np.clip(ang_meas, 0.0, 180.0))
            self.ang_meas = ang_meas
            self.t_meas = now

            # innovación (error)
            err = ang_meas - self.ang_est

            # Si el error es grande y estamos cerca de los bordes, suele ser por rebote/dirección:
            # no hacemos wrap circular porque es 0..180 con rebote.
            self.ang_est = self.ang_est + self.k_corr * err

            # Ajuste de dirección usando tendencia de la medición (si hay anterior)
            # (simple: si la medición sube respecto a est, empuja dir +, si baja -> dir -)
            if err > 1.0:
                self.dir = int(np.sign((1 - self.k_dir) * self.dir + self.k_dir * (+1)))
                if self.dir == 0: self.dir = +1
            elif err < -1.0:
                self.dir = int(np.sign((1 - self.k_dir) * self.dir + self.k_dir * (-1)))
                if self.dir == 0: self.dir = -1

            self.ang_est = float(np.clip(self.ang_est, 0.0, 180.0))

    def snapshot(self):
        with self.lock:
            now = time.time()
            self._propagate_nolock(now)
            ang_est = float(self.ang_est)
            ang_meas = None if self.ang_meas is None else float(self.ang_meas)
            rmax = float(self.rmax)
            pts = list(self.points)
            return ang_est, ang_meas, rmax, pts


# UI: Panel de Registro (derecha)
class LogPanel(ttk.Frame):
    def __init__(self, parent, logger: Logger):
        super().__init__(parent)
        self.logger = logger

        self.filter_var = tk.StringVar(value="TODOS")

        # Header: filtro
        header = ttk.Frame(self)
        header.pack(fill="x", pady=(0, 6))

        ttk.Label(header, text="Registro", font=("Segoe UI", 11, "bold")).pack(side="left")

        ttk.Label(header, text="Filtrar:").pack(side="left", padx=(10, 4))
        self.combo = ttk.Combobox(
            header,
            state="readonly",
            values=["TODOS"] + ALL_TYPES,
            textvariable=self.filter_var,
            width=14
        )
        self.combo.pack(side="left")
        self.combo.bind("<<ComboboxSelected>>", lambda e: self.reload_from_file())

        # Cuerpo: texto
        self.text = tk.Text(self, height=20, wrap="word", state="disabled")
        self.text.pack(fill="both", expand=True)

        # Footer: observaciones
        footer = ttk.Frame(self)
        footer.pack(fill="x", pady=(6, 0))

        ttk.Label(footer, text="Observación:").pack(side="left")
        self.entry_obs = ttk.Entry(footer)
        self.entry_obs.pack(side="left", fill="x", expand=True, padx=6)

        ttk.Button(footer, text="Guardar", command=self._save_obs).pack(side="left")
        self.entry_obs.bind("<Return>", lambda e: self._save_obs())

        # Suscribirse al logger (para añadir en caliente)
        self.logger.subscribe(self._on_new_log_line)

        # Cargar inicial
        self.reload_from_file()

    def _pass_filter(self, linea: str) -> bool:
        f = self.filter_var.get()
        if f == "TODOS":
            return True
        return linea.startswith(f + " |")

    def reload_from_file(self):
        contenido = self.logger.read_all()
        lines = contenido.splitlines(True)

        self.text.config(state="normal")
        self.text.delete("1.0", tk.END)
        for ln in lines:
            if self._pass_filter(ln):
                self.text.insert(tk.END, ln)
        self.text.see(tk.END)
        self.text.config(state="disabled")

    def _on_new_log_line(self, linea: str):
        # Este callback puede venir desde otros hilos: aseguramos UI thread con after
        def _ui():
            if not self._pass_filter(linea):
                return
            self.text.config(state="normal")
            self.text.insert(tk.END, linea)
            self.text.see(tk.END)
            self.text.config(state="disabled")
        self.after(0, _ui)

    def _save_obs(self):
        txt = self.entry_obs.get().strip()
        if not txt:
            return
        self.logger.log(EVENT_OBS, txt)
        self.entry_obs.delete(0, tk.END)


# UI: Controles (izq arriba) organizados por componentes
class ControlsPanel(ttk.Frame):
    def __init__(self, parent, serial_mgr: SerialManager, logger: Logger):
        super().__init__(parent)
        self.serial_mgr = serial_mgr
        self.logger = logger

        def send_ack(g, c, v=0, desc=""):
            expected = ACK.get((g, c), None)
            self.serial_mgr.send_cmd_with_ack(g, c, v, expected_status_codes=expected, descripcion=desc)

        # =========================
        # GLOBAL
        # =========================
        box_global = ttk.LabelFrame(self, text="GLOBAL", padding=8)
        box_global.pack(fill="x", pady=(0, 8))

        ttk.Button(box_global, text="START",
                   command=lambda: send_ack(0, 1, 0, "START global")).pack(side="left", padx=4)
        ttk.Button(box_global, text="STOP",
                   command=lambda: send_ack(0, 2, 0, "STOP global")).pack(side="left", padx=4)

        ttk.Label(box_global, text="Periodo (ms):").pack(side="left", padx=(14, 4))
        self.entry_t_global = ttk.Entry(box_global, width=8)
        self.entry_t_global.insert(0, "1000")
        self.entry_t_global.pack(side="left")
        ttk.Button(box_global, text="Aplicar",
                   command=lambda: self._apply_period(self.entry_t_global, 0, 3, "Periodo GLOBAL", send_ack)).pack(side="left", padx=6)

        # =========================
        # DHT
        # =========================
        box_dht = ttk.LabelFrame(self, text="DHT (Temperatura/Humedad)", padding=8)
        box_dht.pack(fill="x", pady=(0, 8))

        ttk.Button(box_dht, text="DHT ON",
                   command=lambda: send_ack(1, 1, 0, "DHT ON")).pack(side="left", padx=4)
        ttk.Button(box_dht, text="DHT OFF",
                   command=lambda: send_ack(1, 2, 0, "DHT OFF")).pack(side="left", padx=4)

        ttk.Label(box_dht, text="Periodo (ms):").pack(side="left", padx=(14, 4))
        self.entry_t_dht = ttk.Entry(box_dht, width=8)
        self.entry_t_dht.insert(0, "1000")
        self.entry_t_dht.pack(side="left")
        ttk.Button(box_dht, text="Aplicar",
                   command=lambda: self._apply_period(self.entry_t_dht, 1, 3, "Periodo DHT", send_ack)).pack(side="left", padx=6)

        # Checkbox: media en satélite vs media en Python
        self.var_mean_sat = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            box_dht,
            text="Media en satélite",
            variable=self.var_mean_sat,
            command=lambda: self._toggle_mean_source(send_ack)
        ).pack(side="left", padx=12)

        # =========================
        # DISTANCIA
        # =========================
        box_dist = ttk.LabelFrame(self, text="DISTANCIA (Ultrasonidos)", padding=8)
        box_dist.pack(fill="x", pady=(0, 8))

        ttk.Button(box_dist, text="DIST ON",
                   command=lambda: send_ack(3, 1, 0, "DIST ON")).pack(side="left", padx=4)
        ttk.Button(box_dist, text="DIST OFF",
                   command=lambda: send_ack(3, 2, 0, "DIST OFF")).pack(side="left", padx=4)

        ttk.Label(box_dist, text="Periodo (ms):").pack(side="left", padx=(14, 4))
        self.entry_t_dist = ttk.Entry(box_dist, width=8)
        self.entry_t_dist.insert(0, "1000")
        self.entry_t_dist.pack(side="left")
        ttk.Button(box_dist, text="Aplicar",
                   command=lambda: self._apply_period(self.entry_t_dist, 3, 3, "Periodo DIST", send_ack)).pack(side="left", padx=6)

        # =========================
        # SERVO
        # =========================
        box_servo = ttk.LabelFrame(self, text="SERVO", padding=8)
        box_servo.pack(fill="x", pady=(0, 0))

        ttk.Button(box_servo, text="AUTO STOP",
                   command=lambda: send_ack(2, 2, 0, "SERVO AUTO STOP")).pack(side="left", padx=4)
        ttk.Button(box_servo, text="AUTO START",
                   command=lambda: send_ack(2, 3, 0, "SERVO AUTO START")).pack(side="left", padx=4)

        ttk.Label(box_servo, text="Telemetría ang (ms, 0=OFF):").pack(side="left", padx=(14, 4))
        self.entry_t_servo_tlm = ttk.Entry(box_servo, width=8)
        self.entry_t_servo_tlm.insert(0, "1000")
        self.entry_t_servo_tlm.pack(side="left")
        ttk.Button(box_servo, text="Aplicar",
                   command=lambda: self._apply_period(self.entry_t_servo_tlm, 2, 4, "Servo TLM", send_ack)).pack(side="left", padx=6)

        ttk.Label(box_servo, text="Ángulo (-90..90):").pack(side="left", padx=(14, 4))
        self.entry_servo_ang = ttk.Entry(box_servo, width=6)
        self.entry_servo_ang.insert(0, "0")
        self.entry_servo_ang.pack(side="left")
        ttk.Button(box_servo, text="Mover",
                   command=lambda: self._apply_servo_angle(send_ack)).pack(side="left", padx=6)

        ttk.Label(box_servo, text="(mantiene ~4s)").pack(side="left", padx=(6, 0))

    def _toggle_mean_source(self, send_ack):
        if self.var_mean_sat.get():
            send_ack(1, 5, 0, "Media en SAT")
            if self.serial_mgr.th_model:
                self.serial_mgr.th_model.set_mean_source(True)
        else:
            send_ack(1, 4, 0, "Media en PY")
            if self.serial_mgr.th_model:
                self.serial_mgr.th_model.set_mean_source(False)

    def _apply_period(self, entry: ttk.Entry, grupo: int, codigo: int, label: str, send_ack):
        try:
            t = int(entry.get().strip())
        except ValueError:
            messagebox.showerror("Valor inválido", f"{label}: introduce un entero (ms).")
            return
        send_ack(grupo, codigo, t, f"{label} = {t} ms")

    def _apply_servo_angle(self, send_ack):
        try:
            a = int(self.entry_servo_ang.get().strip())
            if not (-90 <= a <= 90):
                raise ValueError
        except ValueError:
            messagebox.showerror("Ángulo inválido", "Introduce un entero entre -90 y 90.")
            return
        send_ack(2, 1, a, f"Servo manual -> {a}° (sat)")

# UI: Gráficas (izq abajo) - selector por barra superior
class GraphsPanel(ttk.Frame):
    def __init__(self, parent, logger: Logger, th_model: THModel):
        super().__init__(parent)
        self.logger = logger
        self.th = th_model

        # Vars para mostrar/ocultar
        self.var_show_t = tk.BooleanVar(value=True)
        self.var_show_h = tk.BooleanVar(value=True)
        self.var_show_m = tk.BooleanVar(value=True)

        # Layout: header arriba, gráfica abajo ocupando TODO
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        header = ttk.Frame(self)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        header.grid_columnconfigure(1, weight=1)

        ttk.Label(header, text="Gráficas", font=("Segoe UI", 11, "bold")).grid(row=0, column=0, sticky="w")

        toggles = ttk.Frame(header)
        toggles.grid(row=0, column=1, sticky="e")

        ttk.Checkbutton(
            toggles, text="Temperatura", variable=self.var_show_t,
            command=self._apply_visibility
        ).pack(side="left", padx=6)

        ttk.Checkbutton(
            toggles, text="Humedad", variable=self.var_show_h,
            command=self._apply_visibility
        ).pack(side="left", padx=6)

        ttk.Checkbutton(
            toggles, text="Media", variable=self.var_show_m,
            command=self._apply_visibility
        ).pack(side="left", padx=6)

        # Fuente (ok aquí, aunque idealmente se pone una vez global)
        plt.rcParams.update({
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "legend.fontsize": 9,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
        })

        # Dos ejes verticales
        self.fig, self.ax_t = plt.subplots(1, 1)
        self.ax_h = self.ax_t.twinx()  # eje derecho (humedad)

        # Márgenes fijos (mejor que tight_layout con twinx)
        self.fig.subplots_adjust(
            left=0.12,
            right=0.88,
            top=0.86,     # un pelín menos para que quepa bien el título y la leyenda
            bottom=0.18
        )

        # Líneas con colores pedidos
        self.line_t, = self.ax_t.plot([], [], label="Temperatura (°C)", color="red")
        self.line_m, = self.ax_t.plot([], [], label="Media T (°C)", color="maroon")
        self.line_h, = self.ax_h.plot([], [], label="Humedad (%)", color="blue")

        # Ejes
        self.ax_t.set_xlabel("Tiempo (s)")
        self.ax_t.set_ylabel("Temperatura (°C)")
        self.ax_h.set_ylabel("Humedad (%)")

        # Rangos fijos
        self.ax_t.set_ylim(0, 40)
        self.ax_h.set_ylim(0, 100)

        # Leyenda combinada (IMPORTANTE: pasar handles+labels explícitos)
        handles = [self.line_t, self.line_m, self.line_h]
        labels = [h.get_label() for h in handles]
        self.ax_t.legend(
            handles, labels,
            loc="upper center",
            bbox_to_anchor=(0.5, 1.18),
            ncol=3,
            frameon=False
        )

        # Canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.w = self.canvas.get_tk_widget()
        self.w.grid(row=1, column=0, sticky="nsew")

        self._apply_visibility()
        self.after(500, self._update_loop)

    def _apply_visibility(self):
        self.line_t.set_visible(self.var_show_t.get())
        self.line_h.set_visible(self.var_show_h.get())
        self.line_m.set_visible(self.var_show_m.get())
        self.canvas.draw_idle()

    def _update_loop(self):
        t0, VENTANA, xs, temps, hums, medias, temp_now, hum_now = self.th.snapshot()

        # Ventana temporal
        if t0 is not None:
            tiempo_actual = time.time() - t0
            t_max = max(VENTANA, tiempo_actual)
            t_min = max(0.0, t_max - VENTANA)
        else:
            t_min, t_max = 0.0, VENTANA

        # Actualizar datos
        self.line_t.set_data(xs, temps)
        self.line_h.set_data(xs, hums)
        self.line_m.set_data(xs, medias)

        # Eje X común
        self.ax_t.set_xlim(t_min, t_max)
        self.ax_h.set_xlim(t_min, t_max)

        # Recalcular solo temperatura/media (la humedad siempre va de 0-100%)
        self.ax_t.relim()
        self.ax_t.autoscale_view(scalex=False, scaley=True)

        self.ax_t.set_title(f"T={temp_now:.2f} °C | H={hum_now:.2f} %")

        self.canvas.draw_idle()
        self.after(500, self._update_loop)


# UI: Radar (centro arriba) + control servo
class RadarPanel(ttk.Frame):
    def __init__(self, parent, serial_mgr: SerialManager, logger: Logger, radar_model: RadarModel):
        super().__init__(parent)
        self.serial_mgr = serial_mgr
        self.logger = logger
        self.radar = radar_model

        header = ttk.Frame(self)
        header.pack(fill="x")

        ttk.Label(header, text="Radar", font=("Segoe UI", 11, "bold")).pack(side="left")

        # Control servo
        ctrl = ttk.Frame(header)
        ctrl.pack(side="right")

        ttk.Label(ctrl, text="Ángulo servo (0-180):").pack(side="left", padx=(0, 6))
        self.entry_ang = ttk.Entry(ctrl, width=6)
        self.entry_ang.insert(0, "90")
        self.entry_ang.pack(side="left")

        ttk.Button(ctrl, text="Aplicar", command=self._apply_angle).pack(side="left", padx=6)

        # Figura polar
        plt.rcParams.update({
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
        })

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="polar")

        # “Radar” tipo semicírculo:
        self.ax.set_theta_zero_location("W")     # 0° arriba
        self.ax.set_theta_direction(-1)          # sentido horario
        self.ax.set_thetamin(0)
        self.ax.set_thetamax(180)

        self.ax.grid(True)

        # Elementos: puntos + rayo (ángulo estimado) + rayo (ángulo real opcional)
        self.scatter = self.ax.scatter([], [], s=12)
        self.ray_est, = self.ax.plot([], [], linewidth=2)   # línea de barrido
        self.ray_meas, = self.ax.plot([], [], linewidth=2)  # línea real (si quieres)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, pady=(6, 0))

        self.after(60, self._update_loop)

    def _apply_angle(self):
        try:
            ang = int(self.entry_ang.get().strip())
            if not (0 <= ang <= 180):
                raise ValueError
        except ValueError:
            messagebox.showerror("Ángulo inválido", "Introduce un entero entre 0 y 180.")
            return

        # Satélite espera -90..90
        a_sat = ang - 90

        expected = ACK.get((2, 1), [300, 301])
        ok = self.serial_mgr.send_cmd_with_ack(
            2, 1, a_sat,
            expected_status_codes=expected,
            descripcion=f"Servo manual -> {ang}° (GUI)"
        )
        if ok:
            # IMPORTANTE: al mandar manual, “reseteamos” el estimador a ese ángulo
            self.radar.set_manual_angle(ang)

    def _update_loop(self):
        ang_est, ang_meas, rmax, pts = self.radar.snapshot()

        # Puntos
        if pts:
            thetas = [p[0] for p in pts]
            rs = [p[1] for p in pts]
            self.scatter.set_offsets(np.c_[thetas, rs])
        else:
            self.scatter.set_offsets(np.empty((0, 2)))

        # Rayos
        th_est = np.deg2rad(ang_est)
        self.ray_est.set_data([th_est, th_est], [0, rmax])

        if ang_meas is not None:
            th_m = np.deg2rad(ang_meas)
            self.ray_meas.set_data([th_m, th_m], [0, rmax])
        else:
            self.ray_meas.set_data([], [])

        # Reescalado dinámico (como en tu versión anterior)
        self.ax.set_rlim(0, rmax)

        self.ax.set_title(
            f"Ángulo estimado: {ang_est:.1f}°"
            + (f" | Real: {ang_meas:.1f}°" if ang_meas is not None else "")
            + f" | rmax={rmax:.0f} cm"
        )

        self.canvas.draw_idle()
        self.after(60, self._update_loop)


# UI: Órbita / Groundtrack (centro abajo) - placeholder
class OrbitPanel(ttk.Frame):
    def __init__(self, parent, selected_view_var: tk.StringVar):
        super().__init__(parent)
        self.selected_view_var = selected_view_var

        header = ttk.Frame(self)
        header.pack(fill="x")
        ttk.Label(header, text="Órbita / Groundtrack", font=("Segoe UI", 11, "bold")).pack(side="left")

        self.badge = ttk.Label(header, text="")
        self.badge.pack(side="right")

        self.stack = ttk.Frame(self)
        self.stack.pack(fill="both", expand=True, pady=(6, 0))

        self.views = {
            "Órbita 3D": self._make_placeholder("Placeholder: Órbita 3D (pendiente)"),
            "Groundtrack": self._make_placeholder("Placeholder: Groundtrack (pendiente)"),
        }

        for v in self.views.values():
            v.place(relx=0, rely=0, relwidth=1, relheight=1)

        self.selected_view_var.trace_add("write", lambda *_: self.show_selected())
        self.show_selected()

    def _make_placeholder(self, text: str) -> ttk.Frame:
        f = ttk.Frame(self.stack)
        ttk.Label(f, text=text, foreground="gray").pack(expand=True)
        return f

    def show_selected(self):
        sel = self.selected_view_var.get()
        if sel not in self.views:
            sel = "Órbita 3D"
            self.selected_view_var.set(sel)

        self.badge.config(text=f"Mostrando: {sel}")
        self.views[sel].tkraise()


# APP PRINCIPAL
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Control del satélite")
        self.geometry("1920x1080")

        # Logger
        log_path = os.path.join(os.getcwd(), "eventos_v4_2.txt")
        self.logger = Logger(log_path)

        # Serial
        self.serial_mgr = SerialManager(DEFAULT_PORT, BAUDRATE, self.logger)
        self.th_model = THModel(ventana_tiempo=60.0) # Intervalo de tiempo que se muestra en las graficas
        self.serial_mgr.th_model = self.th_model

        # Creacion y enlace de RadarModel 
        self.radar_model = RadarModel(tiempo_ciclo_ms=3000, max_points=400)
        self.serial_mgr.radar_model = self.radar_model


        # Variables de “menú superior”
        self.var_show_controls = tk.BooleanVar(value=True)
        self.var_show_graphs   = tk.BooleanVar(value=True)
        self.var_show_radar    = tk.BooleanVar(value=True)
        self.var_show_orbit    = tk.BooleanVar(value=True)
        self.var_show_log      = tk.BooleanVar(value=True)

        self.var_graph_view = tk.StringVar(value="Temperatura")
        self.var_orbit_view = tk.StringVar(value="Órbita 3D")

        # Layout base
        self._build_topbar()
        self._build_main_layout()
        self._apply_visibility()
        self.serial_mgr.start_reader()


        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_topbar(self):
        bar = ttk.Frame(self, padding=6)
        bar.pack(side="top", fill="x")

        # Bloque: mostrar/ocultar módulos
        ttk.Label(bar, text="Módulos:", font=("Segoe UI", 10, "bold")).pack(side="left", padx=(0, 8))

        def cb(text, var):
            ttk.Checkbutton(bar, text=text, variable=var, command=self._apply_visibility).pack(side="left", padx=4)

        cb("Controles", self.var_show_controls)
        cb("Gráficas",   self.var_show_graphs)
        cb("Radar",      self.var_show_radar)
        cb("Órbita",     self.var_show_orbit)
        cb("Registro",   self.var_show_log)

        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=10)

        # Selector: qué gráfica se ve en el bloque de gráficas
        ttk.Label(bar, text="Gráfica:", font=("Segoe UI", 10, "bold")).pack(side="left")
        ttk.Combobox(
            bar, state="readonly", width=14,
            values=["Temperatura", "Humedad", "Media T"],
            textvariable=self.var_graph_view
        ).pack(side="left", padx=6)

        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=10)

        # Selector: órbita vs groundtrack
        ttk.Label(bar, text="Vista central inferior:", font=("Segoe UI", 10, "bold")).pack(side="left")
        ttk.Combobox(
            bar, state="readonly", width=14,
            values=["Órbita 3D", "Groundtrack"],
            textvariable=self.var_orbit_view
        ).pack(side="left", padx=6)

    def _build_main_layout(self):
        
        # Columnas
          # 0 = izquierda (controles + gráficas)
          # 1 = centro (radar + órbita)
          # 2 = derecha (registro)

        main = ttk.Frame(self, padding=8)
        main.pack(side="top", fill="both", expand=True)

        main.columnconfigure(0, weight=1, uniform="col")
        main.columnconfigure(1, weight=1, uniform="col")
        main.columnconfigure(2, weight=1, uniform="col")

        main.rowconfigure(0, weight=1)
        main.rowconfigure(1, weight=1)

        # IZQUIERDA: controles (arriba) y gráficas (abajo)
        self.left_top = ttk.LabelFrame(main, text="Componentes / Comandos", padding=8)
        self.left_top.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 6))

        self.left_bottom = ttk.LabelFrame(main, text="Gráficas", padding=8)
        self.left_bottom.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(0, 0))

        self.controls_panel = ControlsPanel(self.left_top, self.serial_mgr, self.logger)
        self.controls_panel.pack(fill="both", expand=True)

        self.graphs_panel = GraphsPanel(self.left_bottom, self.logger, self.th_model)
        self.graphs_panel.pack(fill="both", expand=True)

        # CENTRO: radar (arriba) y órbita (abajo)
        self.center_top = ttk.LabelFrame(main, text="", padding=8)
        self.center_top.grid(row=0, column=1, sticky="nsew", padx=(0, 6), pady=(0, 6))

        self.center_bottom = ttk.LabelFrame(main, text="", padding=8)
        self.center_bottom.grid(row=1, column=1, sticky="nsew", padx=(0, 6), pady=(0, 0))

        self.radar_panel = RadarPanel(self.center_top, self.serial_mgr, self.logger, self.radar_model)
        self.radar_panel.pack(fill="both", expand=True)

        self.orbit_panel = OrbitPanel(self.center_bottom, self.var_orbit_view)
        self.orbit_panel.pack(fill="both", expand=True)

        # DERECHA: registro (ocupa arriba+abajo)
        self.right = ttk.LabelFrame(main, text="", padding=8)
        self.right.grid(row=0, column=2, rowspan=2, sticky="nsew", padx=(0, 0), pady=(0, 0))

        self.log_panel = LogPanel(self.right, self.logger)
        self.log_panel.pack(fill="both", expand=True)

    def _apply_visibility(self):
        # Izq arriba
        self._set_visible(self.left_top, self.var_show_controls.get())
        # Izq abajo
        self._set_visible(self.left_bottom, self.var_show_graphs.get())
        # Centro arriba
        self._set_visible(self.center_top, self.var_show_radar.get())
        # Centro abajo
        self._set_visible(self.center_bottom, self.var_show_orbit.get())
        # Derecha
        self._set_visible(self.right, self.var_show_log.get())

    def _set_visible(self, widget, visible: bool):
        # Grid widgets: esconder/mostrar sin destruir
        if visible:
            widget.grid()
        else:
            widget.grid_remove()

    def _on_close(self):
        self.serial_mgr.close()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
