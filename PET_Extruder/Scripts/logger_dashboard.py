import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import csv
import os
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque

class PETExtruderDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("PET Extruder Control Center")
        self.root.geometry("1000x700")
        
        # Carpetas de logs
        self.log_dir = "Logs"
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        self.csv_file_telemetry = None
        self.csv_writer_telemetry = None
        self.csv_file_events = None
        self.csv_writer_events = None
        
        # Ultimo estado de error para detectar cambios
        self.last_error_code = 0
        
        # Mapa de errores (debe coincidir con C)
        self.ERROR_MAP = {
            0: "OK",
            1: "NTC_DISCONNECTED",
            2: "OVERTEMP",
            3: "HEATING_TIMEOUT",
            4: "MOTOR_FAULT",
            5: "EMERGENCY_STOP"
        }

        # Manejo correcto del cierre de ventana
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self.ser = None
        self.reading_thread = None
        self.is_running = False
        
        # Control de animación
        self.after_id = None
        self.log_path_telem = None
        self.log_path_event = None

        self.current_flags = 0

        # --- Variables de Datos ---
        self.max_points = 200
        self.times = deque(maxlen=self.max_points)
        self.temps = deque(maxlen=self.max_points)
        self.targets = deque(maxlen=self.max_points)
        self.pwms = deque(maxlen=self.max_points)
        self.speeds = deque(maxlen=self.max_points)
        
        self.start_time = time.time()

        self.setup_ui()
        self.animate_loop()

    def setup_ui(self):
        # === PANEL LATERAL IZQUIERDO ===
        left_panel = ttk.Frame(self.root, padding="10")
        left_panel.pack(side=tk.LEFT, fill=tk.Y)

        # 1. Conexión
        ttk.Label(left_panel, text="Conexión Serial", font=("Arial", 11, "bold")).pack(pady=5)
        self.combo_ports = ttk.Combobox(left_panel, values=self.get_ports())
        self.combo_ports.pack(fill=tk.X, pady=2)
        
        self.btn_connect = ttk.Button(left_panel, text="Conectar", command=self.toggle_connection)
        self.btn_connect.pack(fill=tk.X, pady=5)
        
        self.btn_refresh = ttk.Button(left_panel, text="Refrescar", command=self.refresh_ports)
        self.btn_refresh.pack(fill=tk.X)

        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, pady=15)

        # 2. Control Térmico
        ttk.Label(left_panel, text="Calefacción", font=("Arial", 11, "bold")).pack(pady=5)
        
        frame_setpoint = ttk.Frame(left_panel)
        frame_setpoint.pack(fill=tk.X, pady=5)
        ttk.Label(frame_setpoint, text="Target:").pack(side=tk.LEFT)
        self.entry_temp = ttk.Entry(frame_setpoint, width=6)
        self.entry_temp.insert(0, "200")
        self.entry_temp.pack(side=tk.LEFT, padx=5)
        ttk.Button(frame_setpoint, text="Set", command=self.send_temp, width=4).pack(side=tk.LEFT)

        self.btn_heater_on = tk.Button(left_panel, text="🔥 ACTIVAR CALENTADOR", bg="#ffcccc", command=lambda: self.send_cmd("h1"))
        self.btn_heater_on.pack(fill=tk.X, pady=2)
        
        self.btn_heater_off = tk.Button(left_panel, text="❄️ DESACTIVAR", bg="#ccffcc", command=lambda: self.send_cmd("h0"))
        self.btn_heater_off.pack(fill=tk.X, pady=2)

        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, pady=15)

        # 3. Control Motor
        ttk.Label(left_panel, text="Extrusor", font=("Arial", 11, "bold")).pack(pady=5)
        
        # Indicador de estado de temperatura
        self.lbl_cold_protect = tk.Label(left_panel, text="ESTADO: DESCONOCIDO", bg="gray", fg="white", font=("Arial", 9, "bold"))
        self.lbl_cold_protect.pack(fill=tk.X, pady=2)

        frame_speed = ttk.Frame(left_panel)
        frame_speed.pack(fill=tk.X, pady=5)
        ttk.Label(frame_speed, text="Vel (mm/s):").pack(side=tk.LEFT)
        self.entry_speed = ttk.Entry(frame_speed, width=6)
        self.entry_speed.insert(0, "3.0")
        self.entry_speed.pack(side=tk.LEFT, padx=5)
        ttk.Button(frame_speed, text="Set", command=self.send_speed, width=4).pack(side=tk.LEFT)

        self.btn_motor_start = tk.Button(left_panel, text="▶ INICIAR MOTOR", bg="#ccffcc", command=lambda: self.send_cmd("r"))
        self.btn_motor_start.pack(fill=tk.X, pady=2)
        
        self.btn_motor_stop = tk.Button(left_panel, text="⏹ DETENER MOTOR", bg="#ffcccc", command=lambda: self.send_cmd("s"))
        self.btn_motor_stop.pack(fill=tk.X, pady=2)

        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, pady=15)

        # 4. Sintonización PID
        ttk.Label(left_panel, text="Sintonización PID", font=("Arial", 11, "bold")).pack(pady=5)
        
        frame_pid = ttk.Frame(left_panel)
        frame_pid.pack(fill=tk.X, pady=0)
        
        # P
        ttk.Label(frame_pid, text="Kp:").grid(row=0, column=0, padx=2, pady=2, sticky="e")
        self.entry_kp = ttk.Entry(frame_pid, width=6)
        self.entry_kp.insert(0, "4.0")
        self.entry_kp.grid(row=0, column=1, padx=2, pady=2)
        
        # I
        ttk.Label(frame_pid, text="Ki:").grid(row=1, column=0, padx=2, pady=2, sticky="e")
        self.entry_ki = ttk.Entry(frame_pid, width=6)
        self.entry_ki.insert(0, "0.15")
        self.entry_ki.grid(row=1, column=1, padx=2, pady=2)

        # D
        ttk.Label(frame_pid, text="Kd:").grid(row=2, column=0, padx=2, pady=2, sticky="e")
        self.entry_kd = ttk.Entry(frame_pid, width=6)
        self.entry_kd.insert(0, "10.0")
        self.entry_kd.grid(row=2, column=1, padx=2, pady=2)
        
        ttk.Button(frame_pid, text="Actualizar PID", command=self.send_pid).grid(row=3, column=0, columnspan=2, pady=5)

        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, pady=15)

        # 5. Persistencia
        self.btn_save_flash = tk.Button(left_panel, text="💾 GUARDAR EN MEMORIA", bg="#ddddff", command=self.save_to_flash)
        self.btn_save_flash.pack(fill=tk.X, pady=5)
        
        # 6. Restablecer Valores
        self.btn_restore = tk.Button(left_panel, text="♻ RESTABLECER VALORES", bg="#ffffcc", command=self.restore_defaults)
        self.btn_restore.pack(fill=tk.X, pady=5)
        
        self.lbl_status = ttk.Label(left_panel, text="Desconectado", font=("Consolas", 10))
        self.lbl_status.pack(side=tk.BOTTOM, pady=10)

        # === PANEL DERECHO (GRÁFICOS) ===
        right_panel = ttk.Frame(self.root)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Configuración de Gráficos (2 Filas, 1 Columna)
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 8), sharex=True, gridspec_kw={'height_ratios': [2, 1]})
        self.fig.subplots_adjust(left=0.1, bottom=0.1, right=0.9, top=0.95, hspace=0.2)
        
        # --- GRAFICO 1: TEMPERATURA (EJE IZQ) + PWM (EJE DER) ---
        self.ax1.set_title("Dinámica Térmica del Extrusor")
        self.ax1.set_ylabel("Temperatura (°C)", color='r')
        self.ax1.tick_params(axis='y', labelcolor='r')
        self.ax1.grid(True, linestyle=':', alpha=0.6)
        
        # Eje gemelo para PWM
        self.ax1_pwm = self.ax1.twinx()
        self.ax1_pwm.set_ylabel("Potencia PWM (%)", color='b')
        self.ax1_pwm.set_ylim(-5, 105) # Fijo 0-100%
        self.ax1_pwm.tick_params(axis='y', labelcolor='b')
        
        # Líneas (Inicialmente vacías)
        self.line_temp, = self.ax1.plot([], [], 'r-', linewidth=2, label='T. Actual')
        self.line_target, = self.ax1.plot([], [], 'g--', linewidth=1, label='T. Target')
        self.line_pwm, = self.ax1_pwm.plot([], [], 'b-', linewidth=0.8, alpha=0.4, label='PWM')
        
        # Leyenda combinada (truco para unir leyendas de dos ejes)
        lines = [self.line_temp, self.line_target, self.line_pwm]
        labels = [l.get_label() for l in lines]
        self.ax1.legend(lines, labels, loc="upper left")

        # --- GRAFICO 2: VELOCIDAD ---
        self.ax2.set_ylabel("Velocidad (mm/s)", color='k')
        self.ax2.set_xlabel("Tiempo (s)")
        self.ax2.grid(True)
        self.line_speed, = self.ax2.plot([], [], 'k-', linewidth=1.5, label='Velocidad')
        self.ax2.legend(loc="upper left")

        # Canvas Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def on_close(self):
        """Cierre seguro de la aplicación"""
        self.is_running = False
        
        # Cancelar animación pendiente para evitar error "invalid command name"
        if self.after_id:
            try:
                self.root.after_cancel(self.after_id)
                self.after_id = None
            except: pass

        if self.ser and self.ser.is_open:
            self.ser.close()
        
        # Cerrar archivos CSV
        if self.csv_file_telemetry: self.csv_file_telemetry.close()
        if self.csv_file_events: self.csv_file_events.close()
            
        self.root.destroy()
        print("\n=== Aplicación cerrada ===")
        if self.log_path_telem:
            print(f"📄 Log Telemetría: {self.log_path_telem}")
        if self.log_path_event:
            print(f"⚠️ Log Eventos:    {self.log_path_event}")

    def get_ports(self):
        return [com.device for com in serial.tools.list_ports.comports()]

    def refresh_ports(self):
        self.combo_ports['values'] = self.get_ports()

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.combo_ports.get()
        if not port: return
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.is_running = True
            
            # --- PREPARAR LOGS ---
            now_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            
            # 1. Archivo Telemetría (Datos Crudos)
            filename_telem = os.path.join(self.log_dir, f"telemetry_{now_str}.csv")
            self.log_path_telem = os.path.abspath(filename_telem) # Guardar ruta absoluta
            self.csv_file_telemetry = open(filename_telem, mode='w', newline='')
            self.csv_writer_telemetry = csv.writer(self.csv_file_telemetry)
            self.csv_writer_telemetry.writerow(["Timestamp_PC", "Tick_MCU", "Temp", "Target", "PWM", "Speed", "PID_Error", "System_Error_Code", "Status_Flags"])
            
            # 2. Archivo de Eventos/Seguridad
            filename_events = os.path.join(self.log_dir, f"safety_events_{now_str}.csv")
            self.log_path_event = os.path.abspath(filename_events) # Guardar ruta absoluta
            self.csv_file_events = open(filename_events, mode='w', newline='')
            self.csv_writer_events = csv.writer(self.csv_file_events)
            self.csv_writer_events.writerow(["Timestamp_PC", "Event_Type", "Description", "Last_Temp", "Last_Target"])
            
            self.last_error_code = 0 # Reset estado
            
            print(f"Logging started...")

            # Limpiar buffers para evitar basura vieja
            self.ser.reset_input_buffer()
            
            # Pedir configuración actual al arrancar (SYNC)
            time.sleep(0.5)
            self.send_cmd("?")
            
            self.reading_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
            self.reading_thread.start()
            
            # RESTART ANIMATION LOOP
            self.animate_loop() 

            self.btn_connect.config(text="Desconectar")
            self.lbl_status.config(text=f"Conectado a {port}", foreground="green")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def disconnect(self):
        self.is_running = False
        if self.ser:
            self.ser.close()
            
        # Cerrar archivos CSV
        if self.csv_file_telemetry: 
            self.csv_file_telemetry.close()
            self.csv_file_telemetry = None
        if self.csv_file_events:
            self.csv_file_events.close()
            self.csv_file_events = None
            
        self.btn_connect.config(text="Conectar")
        self.lbl_status.config(text="Desconectado", foreground="red")

    def read_serial_loop(self):
        while self.is_running and self.ser and self.ser.is_open:
            try:
                raw_line = self.ser.readline()
                line = raw_line.decode('utf-8', errors='ignore').replace('\x00', '').strip()
                
                if not line: continue

                # Debug en consola: Imprimir TODO lo que llega para ver si hay datos
                print(f"RX: {line}") 

                if line.startswith("OK") or line.startswith("[TH]") or line.startswith("ERR"):
                    # Estos ya se imprimen arriba
                    continue
                
                # PARSER DE PARAMETROS (SYNC)
                # PAR:T=250.0,V=3.0,P=4.00,I=0.15,D=10.00
                if line.startswith("PAR:"):
                    try:
                        # Limpiar 'PAR:' y dividir por comas
                        content = line[4:].strip()
                        parts = content.split(',')
                        for p in parts:
                            key, val = p.split('=')
                            if key == 'T': self.update_entry(self.entry_temp, val)
                            elif key == 'V': self.update_entry(self.entry_speed, val)
                            elif key == 'P': self.update_entry(self.entry_kp, val)
                            elif key == 'I': self.update_entry(self.entry_ki, val)
                            elif key == 'D': self.update_entry(self.entry_kd, val)
                        print("Sync Params Updated!")
                    except Exception as e:
                        print(f"Error parse PAR: {e}")
                    continue

                # CSV: Tick,Temp,Target,PWM,Speed,Error_PID,System_Error
                parts = line.split(',')
                # Aceptamos 5 (legacy), 6 (legacy+error) o 7 (nuevo con sys_error)
                if len(parts) >= 5:
                    try:
                        t_curr = float(parts[0]) / 1000.0
                        temp = float(parts[1])
                        target = float(parts[2]) 
                        pwm = float(parts[3])
                        speed = float(parts[4])
                        # Manejo flexible si faltan columnas nuevas
                        pid_error = float(parts[5]) if len(parts) > 5 else 0.0
                        sys_error_code = int(parts[6]) if len(parts) > 6 else 0
                        status_flags = int(parts[7]) if len(parts) > 7 else 0
                        
                        self.current_flags = status_flags

                        # --- LOGGING TELEMETRY ---
                        if self.csv_writer_telemetry:
                            timestamp_pc = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            self.csv_writer_telemetry.writerow([
                                timestamp_pc, parts[0], temp, target, pwm, speed, pid_error, sys_error_code, status_flags
                            ])
                            self.csv_file_telemetry.flush() # Guardar en disco inmediatamente

                        # --- LOGGING EVENTS (Safety Checks) ---
                        if sys_error_code != self.last_error_code:
                            # Cambio de estado de error detectado
                            if self.csv_writer_events:
                                timestamp_pc = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                                err_desc = self.ERROR_MAP.get(sys_error_code, f"UNKNOWN_ERROR_{sys_error_code}")
                                
                                # Si pasa de 0 a algo -> ERROR DISPARADO
                                if sys_error_code != 0:
                                    evt_type = "ERROR_TRIGGERED"
                                    msg = f"Safety System Triggered: {err_desc}"
                                else:
                                    # Si pasa de algo a 0 -> ERROR CLEARED
                                    evt_type = "SYSTEM_RECOVERED"
                                    msg = "System returned to NORMAL state"
                                    
                                self.csv_writer_events.writerow([timestamp_pc, evt_type, msg, temp, target])
                                self.csv_file_events.flush()
                                
                            self.last_error_code = sys_error_code

                        # 1. Guardar datos para gráficas
                        self.times.append(t_curr)
                        self.temps.append(temp)
                        self.targets.append(target)
                        self.pwms.append(pwm)
                        self.speeds.append(speed)
                        
                        # 2. Sincronizar UI con el Hardware (Solo ocasionalmente para no trabar)
                        self.update_target_entry(target) 

                    except ValueError as ve:
                        print(f"Error parsing CSV: {ve} in line: {line}")

            except Exception as e:
                print(f"Serial Error: {e}")
                
    def animate_loop(self):
        # Si cerramos la ventana o desconectamos, no seguir actualizando
        if not self.is_running: return

        if len(self.times) > 1:
            try:
                # Update Data
                # Convert deque to list to ensure matplotlib compatibility
                l_times = list(self.times)
                self.line_temp.set_data(l_times, list(self.temps))
                self.line_target.set_data(l_times, list(self.targets))
                self.line_pwm.set_data(l_times, list(self.pwms))
                self.line_speed.set_data(l_times, list(self.speeds))
                
                # Ajuste dinámico de eje X
                last_time = l_times[-1]
                self.ax1.set_xlim(l_times[0], last_time + 1)
                
                # Ajuste dinámico de eje Y (Temperatura)
                # Check for empty sequences to avoid max() arg is an empty sequence
                if self.temps and self.targets:
                    y_max = max(max(self.temps), max(self.targets))
                    self.ax1.set_ylim(0, max(50, y_max + 20))
                
                if self.speeds:
                   self.ax2.set_ylim(0, max(5, max(self.speeds) + 2))

                # Redibujar canvas (optimización leve)
                self.canvas.draw()
                
                # UPDATE STATUS COLD PROTECT
                # Bit 2: Temp Safe (1=Safe, 0=Cold)
                is_safe = (self.current_flags & (1 << 2)) != 0
                if is_safe:
                    self.lbl_cold_protect.config(text="TEMP SAFE: READY TO EXTRUDE", bg="#88ff88", fg="black")
                    self.btn_motor_start.config(state="normal", bg="#ccffcc")
                else:
                    self.lbl_cold_protect.config(text="COLD PROTECT: HEATING REQUIRED", bg="#ff5555", fg="white")
                    # Opcional: Deshabilitar boton start si se desea forzar desde GUI
                    # self.btn_motor_start.config(state="disabled", bg="#dddddd")
                
                self.lbl_status.config(text=f"T: {self.temps[-1]:.1f}°C | PWM: {int(self.pwms[-1])}% | Vel: {self.speeds[-1]:.1f} mm/s")
            
            except Exception as e:
                print(f"Animation Error: {e}")

        # Llamar a esta función de nuevo en 500ms
        if self.is_running:
            self.after_id = self.root.after(500, self.animate_loop)

    def send_cmd(self, cmd):
        if self.ser and self.ser.is_open:
            full_cmd = f"{cmd}\n"
            try:
                self.ser.write(full_cmd.encode('utf-8'))
                print(f"SENT: {cmd}")
            except Exception as e:
                print(f"Error TX: {e}")
        else:
            messagebox.showwarning("Warning", "No conectado")

    def update_entry(self, entry_widget, value):
        # Actualiza un campo Entry desde el thread de GUI
        self.root.after(0, lambda: self._delete_insert(entry_widget, value))

    def _delete_insert(self, entry, value):
        entry.delete(0, tk.END)
        entry.insert(0, str(value))

    def send_temp(self):
        try:
            val = float(self.entry_temp.get())
            self.send_cmd(f"t{val}")
        except: pass

    def send_speed(self):
        try:
            val = float(self.entry_speed.get())
            self.send_cmd(f"v{val}")
        except: pass

    def send_pid(self):
        try:
            p = float(self.entry_kp.get())
            i = float(self.entry_ki.get())
            d = float(self.entry_kd.get())
            
            self.send_cmd(f"p{p}")
            time.sleep(0.05)
            self.send_cmd(f"i{i}")
            time.sleep(0.05)
            self.send_cmd(f"d{d}")
        except ValueError:
            messagebox.showerror("Error", "Valores PID Inválidos")

    def save_to_flash(self):
        if messagebox.askyesno("Confirmar", "¿Guardar configuración actual en memoria permanente?"):
            self.send_cmd("w")

    def restore_defaults(self):
        if messagebox.askyesno("Confirmar", "¿Restaurar valores de fábrica? (Se guardarán automáticamente)"):
            self.send_cmd("x") # Comando Factory Reset

    def update_target_entry(self, current_target):
        # Solo actualizar si el usuario no está escribiendo (esto es simple visual)
        # Por ahora lo dejamos simple: el grafico te muestra la verdad.
        pass

if __name__ == "__main__":
    root = tk.Tk()
    app = PETExtruderDashboard(root)
    root.mainloop()