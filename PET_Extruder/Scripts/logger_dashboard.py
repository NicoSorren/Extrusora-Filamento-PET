import serial
import time
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button
import os
from datetime import datetime 

# ================= CONFIGURACIÓN DE RUTAS =================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(SCRIPT_DIR, 'Logs')

if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)
    print(f"Carpeta creada: {LOG_DIR}")

# Timestamp único para esta sesión
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

# ARCHIVO 1: Telemetría (Temperatura, Duty, Speed)
csv_telemetry = os.path.join(LOG_DIR, f'telemetry_{timestamp}.csv')

# ARCHIVO 2: Debug (Estados del Motor)
csv_debug = os.path.join(LOG_DIR, f'debug_{timestamp}.csv')

print("="*60)
print(f"📊 TELEMETRÍA: {csv_telemetry}")
print(f"🔍 DEBUG:      {csv_debug}")
print("="*60)

# ================= CONFIGURACIÓN SERIE =================
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"✅ Conectado a {SERIAL_PORT}")
    ser.reset_input_buffer()
except Exception as e:
    print(f"❌ Error abriendo puerto: {e}")
    exit()

# Inicializar CSVs
with open(csv_telemetry, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Time_ms", "Temp_C", "Setpoint_C", "Heater_Duty", "Speed_mm_s", "Error_C"])

with open(csv_debug, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Time_ms", "Message", "Hz"])  # ⬅️ NUEVO: Agregar columna Hz

# ================= LISTAS PARA GRAFICAR =================
# Gráfico 1: Temperatura
x_data_temp = []
temp_data = []
setpoint_data = []
duty_data = []

# Gráfico 2: Velocidad del Motor (NUEVO)
x_data_speed = []
speed_mm_s_data = []
speed_hz_data = []

# ⬅️ NUEVO: Variable para sincronizar tiempos
last_telemetry_time_ms = 0

# ================= CONFIGURACIÓN DE LA INTERFAZ (2 GRÁFICOS) =================
fig = plt.figure(figsize=(14, 8))
plt.subplots_adjust(bottom=0.15, hspace=0.3)

# --- GRÁFICO 1: TEMPERATURA (Arriba) ---
ax1 = plt.subplot(2, 1, 1)
line_temp, = ax1.plot([], [], 'r-', label='Temp Actual (°C)', linewidth=2)
line_set, = ax1.plot([], [], 'g--', label='Setpoint (°C)', alpha=0.7)
ax1_duty = ax1.twinx()
line_duty, = ax1_duty.plot([], [], 'orange', label='Heater Duty (%)', alpha=0.3)

ax1.set_title(f"Monitor de Extrusora PET - {timestamp}", fontsize=12, fontweight='bold')
ax1.set_xlabel("Tiempo (s)")
ax1.set_ylabel("Temperatura (°C)", color='red')
ax1_duty.set_ylabel("Potencia (%)", color='orange')
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.set_ylim(0, 300)
ax1_duty.set_ylim(0, 105)

lines1 = [line_temp, line_set, line_duty]
labels1 = [l.get_label() for l in lines1]
ax1.legend(lines1, labels1, loc='upper left')

# --- GRÁFICO 2: VELOCIDAD DEL MOTOR (Abajo) ---
ax2 = plt.subplot(2, 1, 2)
line_speed_mm, = ax2.plot([], [], 'b-', label='Velocidad (mm/s)', linewidth=2.5)
ax2_hz = ax2.twinx()
line_speed_hz, = ax2_hz.plot([], [], 'm--', label='Frecuencia (Hz)', linewidth=1.5, alpha=0.7)

ax2.set_title("Rampa de Velocidad del Motor", fontsize=11, fontweight='bold')
ax2.set_xlabel("Tiempo (s)")
ax2.set_ylabel("Velocidad (mm/s)", color='blue', fontweight='bold')
ax2_hz.set_ylabel("Frecuencia (Hz)", color='magenta', fontweight='bold')
ax2.grid(True, linestyle='--', alpha=0.6)
ax2.set_ylim(0, 5.5)  # Rango de 0 a 5 mm/s
ax2_hz.set_ylim(0, 9000)  # Rango de 0 a 8600 Hz (5 mm/s * 1724)

lines2 = [line_speed_mm, line_speed_hz]
labels2 = [l.get_label() for l in lines2]
ax2.legend(lines2, labels2, loc='upper left')

# ================= CONTROLES =================
def submit_speed(text):
    try:
        val = float(text)
        if 0.5 <= val <= 5.0:
            command = f"v{val}\n"
            ser.write(command.encode('utf-8'))
            print(f"✅ COMANDO ENVIADO: v{val}")
        else:
            print("❌ Velocidad fuera de rango (0.5 - 5.0 mm/s)")
    except ValueError:
        print("❌ Ingresa un número válido")

def on_stop_clicked(event):
    ser.write(b"s\n")
    print("🛑 STOP")

def on_resume_clicked(event):
    ser.write(b"r\n")
    print("▶️ START")

axbox = plt.axes([0.15, 0.04, 0.15, 0.05])
text_box = TextBox(axbox, 'Velocidad: ', initial="3.0")
text_box.on_submit(submit_speed)

axbtn_send = plt.axes([0.32, 0.04, 0.08, 0.05])
btn_send = Button(axbtn_send, 'Enviar', color='lightblue', hovercolor='0.975')
btn_send.on_clicked(lambda event: submit_speed(text_box.text))

axbtn_stop = plt.axes([0.42, 0.04, 0.08, 0.05])
btn_stop = Button(axbtn_stop, 'STOP', color='salmon', hovercolor='red')
btn_stop.on_clicked(on_stop_clicked)

axbtn_resume = plt.axes([0.52, 0.04, 0.08, 0.05])
btn_resume = Button(axbtn_resume, 'START', color='lightgreen', hovercolor='green')
btn_resume.on_clicked(on_resume_clicked)

# ================= PARSER DE DATOS =================
def parse_line(line):
    """Clasifica y procesa líneas del STM32"""
    global last_telemetry_time_ms
    
    # CASO 1: Mensajes de Debug (Empiezan con '[')
    if line.startswith('['):
        hz_value = None
        
        if 'Hz:' in line:
            try:
                hz_value = int(line.split('Hz:')[1].split()[0])
            except:
                pass
        elif 'Current:' in line:
            try:
                hz_value = int(line.split('Current:')[1].split()[0].strip())
            except:
                pass
        
        # Guardar en CSV de debug
        with open(csv_debug, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([last_telemetry_time_ms, line, hz_value if hz_value else 0])
        
        # ⬅️ YA NO ACTUALIZAMOS EL GRÁFICO AQUÍ (lo hace la telemetría)
        
        if 'DEBUG' in line or 'START' in line or 'Motor OFF' in line:
            print(f"🔧 {line}")
        
        return 'debug'
    
    # CASO 2: Respuestas a comandos UART (OK/ERR)
    elif line.startswith('OK') or line.startswith('ERR'):
        print(f"📩 {line}")
        return 'response'
    
    # CASO 3: Telemetría CSV (tiene comas)
    elif ',' in line:
        try:
            parts = line.split(',')
            if len(parts) >= 6:
                timestamp_ms = int(parts[0])
                last_telemetry_time_ms = timestamp_ms
                
                temp_actual = float(parts[1])
                temp_target = float(parts[2])
                heater_duty = int(parts[3])
                actual_speed_mm_s = float(parts[4])  # ⬅️ AHORA ES VELOCIDAD REAL
                
                # Actualizar título
                ax1.set_title(f"Monitor PET - Velocidad Actual: {actual_speed_mm_s:.2f} mm/s")
                
                # Guardar en CSV de telemetría
                with open(csv_telemetry, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(parts)
                
                # Actualizar gráfico de temperatura
                x_data_temp.append(timestamp_ms / 1000.0)
                temp_data.append(temp_actual)
                setpoint_data.append(temp_target)
                duty_data.append(heater_duty)
                
                # ⬅️ CAMBIAR: 600 puntos = 60 segundos (era 200)
                if len(x_data_temp) > 600:
                    x_data_temp.pop(0)
                    temp_data.pop(0)
                    setpoint_data.pop(0)
                    duty_data.pop(0)
                
                # Actualizar gráfico de velocidad
                x_data_speed.append(timestamp_ms / 1000.0)
                speed_mm_s_data.append(actual_speed_mm_s)
                speed_hz_data.append(int(actual_speed_mm_s * 1724.0))
                
                # ⬅️ CAMBIAR: 600 puntos = 60 segundos (era 150)
                if len(x_data_speed) > 600:
                    x_data_speed.pop(0)
                    speed_mm_s_data.pop(0)
                    speed_hz_data.pop(0)
                
                return 'telemetry'
        except:
            pass
    
    return 'unknown'

def animate(i):
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                parse_line(line)
        except Exception as e:
            print(f"⚠️ Error: {e}")
    
    # --- ACTUALIZAR GRÁFICO 1: TEMPERATURA ---
    if x_data_temp:
        line_temp.set_data(x_data_temp, temp_data)
        line_set.set_data(x_data_temp, setpoint_data)
        line_duty.set_data(x_data_temp, duty_data)
        
        # Calcular límites del eje X (ventana de 60 segundos)
        x_min = max(0, x_data_temp[-1] - 60)
        x_max = x_data_temp[-1] + 5
        ax1.set_xlim(x_min, x_max)
    
    # --- ACTUALIZAR GRÁFICO 2: VELOCIDAD DEL MOTOR ---
    if x_data_speed:
        line_speed_mm.set_data(x_data_speed, speed_mm_s_data)
        line_speed_hz.set_data(x_data_speed, speed_hz_data)
        
        # ⬅️ CAMBIO: Usar los MISMOS límites que el gráfico de temperatura
        if x_data_temp:  # Solo si hay datos de temperatura
            x_min = max(0, x_data_temp[-1] - 60)
            x_max = x_data_temp[-1] + 5
            ax2.set_xlim(x_min, x_max)
        else:
            # Fallback: Si todavía no hay datos de temperatura
            ax2.set_xlim(max(0, x_data_speed[-1] - 15), x_data_speed[-1] + 1)
    
    return line_temp, line_set, line_duty, line_speed_mm, line_speed_hz

ani = animation.FuncAnimation(fig, animate, interval=100, cache_frame_data=False)
plt.show()

ser.close()
print("🔌 Puerto cerrado.")