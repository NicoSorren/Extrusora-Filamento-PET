import serial
import time
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button
import os
from datetime import datetime

# ================= CONFIGURACIÓN =================
SERIAL_PORT = 'COM3'  # <--- ASEGÚRATE QUE ESTE SEA TU PUERTO CORRECTO
BAUD_RATE = 115200
LOG_DIR = 'Logs'

# Crear carpeta de logs si no existe
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)

# Nombre del archivo CSV con fecha
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
csv_filename = os.path.join(LOG_DIR, f'extruder_log_{timestamp}.csv')

# Inicializar Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Conectado a {SERIAL_PORT}")
    # Limpiar buffer inicial
    ser.reset_input_buffer()
except Exception as e:
    print(f"Error abriendo puerto serie: {e}")
    exit()

# Inicializar CSV
with open(csv_filename, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Time_ms", "Temp_C", "Setpoint_C", "Heater_Duty", "Motor_Hz", "Error_C"])

# Listas para graficar
x_data = []
temp_data = []
setpoint_data = []
duty_data = []

# Configuración de la Gráfica
fig, ax = plt.subplots(figsize=(10, 7))
plt.subplots_adjust(bottom=0.25) # Dejar espacio abajo para los controles

line_temp, = ax.plot([], [], 'r-', label='Temp Actual (°C)', linewidth=2)
line_set, = ax.plot([], [], 'g--', label='Setpoint (°C)', alpha=0.7)
ax2 = ax.twinx() # Eje Y secundario para la potencia
line_duty, = ax2.plot([], [], 'orange', label='Heater Duty (%)', alpha=0.3)
ax2.fill_between([], [], color='orange', alpha=0.1)

# Estética
ax.set_title(f"Monitor de Extrusora PET - {timestamp}")
ax.set_xlabel("Tiempo (s)")
ax.set_ylabel("Temperatura (°C)")
ax2.set_ylabel("Potencia (%) / Motor")
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_ylim(0, 300) # Rango fijo para temperatura
ax2.set_ylim(0, 105)

# Leyendas
lines = [line_temp, line_set, line_duty]
labels = [l.get_label() for l in lines]
ax.legend(lines, labels, loc='upper left')

# ================= CONTROLES (WIDGETS) =================

# Función para enviar velocidad
def submit_speed(text):
    try:
        val = float(text)
        if 0.1 <= val <= 10.0:
            command = f"v{val}\n"
            ser.write(command.encode('utf-8'))
            print(f"--> COMANDO ENVIADO: {command.strip()} mm/s")
        else:
            print("Error: Velocidad fuera de rango (0.1 - 10.0)")
    except ValueError:
        print("Error: Ingresa un número válido")

# Caja de Texto
axbox = plt.axes([0.2, 0.05, 0.2, 0.075]) # Posición [left, bottom, width, height]
text_box = TextBox(axbox, 'Velocidad (mm/s): ', initial="2.5")
text_box.on_submit(submit_speed)

# Botón de Enviar (Opcional, el Enter en la caja también funciona)
axbtn = plt.axes([0.45, 0.05, 0.1, 0.075])
btn = Button(axbtn, 'Enviar', color='lightblue', hovercolor='0.975')

def on_button_clicked(event):
    submit_speed(text_box.text)

btn.on_clicked(on_button_clicked)

# ================= BUCLE DE DATOS =================

def animate(i):
    # Leer todas las líneas disponibles en el buffer
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            
            # Parsear datos: "Tick,Temp,Set,Duty,Speed,Error"
            parts = line.split(',')
            if len(parts) >= 5:
                tick_ms = int(parts[0])
                temp = float(parts[1])
                setpoint = float(parts[2])
                duty = int(parts[3])
                
                # AHORA ESTO ES VELOCIDAD REAL, NO HZ
                real_speed = float(parts[4]) 
                
                # Actualizar título con la velocidad real que reporta el micro
                ax.set_title(f"Monitor PET - Vel Actual: {real_speed} mm/s")
                
                # Guardar en CSV
                with open(csv_filename, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(parts)
                
                # Actualizar listas (Mantener últimos 200 puntos para que no se trabe)
                x_data.append(tick_ms / 1000.0) # Convertir a segundos
                temp_data.append(temp)
                setpoint_data.append(setpoint)
                duty_data.append(duty)
                
                if len(x_data) > 200:
                    x_data.pop(0)
                    temp_data.pop(0)
                    setpoint_data.pop(0)
                    duty_data.pop(0)

        except ValueError:
            pass # Ignorar líneas corruptas
        except Exception as e:
            print(f"Error leyendo: {e}")

    # Actualizar gráfico
    if x_data:
        line_temp.set_data(x_data, temp_data)
        line_set.set_data(x_data, setpoint_data)
        line_duty.set_data(x_data, duty_data)
        
        ax.set_xlim(max(0, x_data[-1] - 60), x_data[-1] + 5) # Ventana móvil de 60 seg
        
    return line_temp, line_set, line_duty

# Iniciar animación
ani = animation.FuncAnimation(fig, animate, interval=100, cache_frame_data=False)
plt.show()

# Cerrar puerto al salir
ser.close()
print("Puerto cerrado.")