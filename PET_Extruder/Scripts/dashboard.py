import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time

# --- CONFIGURACIÓN ---
# ¡IMPORTANTE! Cambia esto por tu puerto (ej: COM3, COM5)
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200

# --- ESTRUCTURAS DE DATOS (MEMORIA) ---
# Usamos deque para mantener solo los últimos 100 datos (ventana deslizante)
MAX_POINTS = 100
times = deque(maxlen=MAX_POINTS)
temps = deque(maxlen=MAX_POINTS)
setpoints = deque(maxlen=MAX_POINTS)
speeds = deque(maxlen=MAX_POINTS)

# --- CONEXIÓN SERIAL ---
try:
    print(f"Intentando conectar a {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"¡Conectado exitosamente!")
    # Esperamos un momento porque al abrir el puerto, el STM32 suele reiniciarse
    time.sleep(2) 
    ser.reset_input_buffer() # Limpiar basura inicial
except serial.SerialException:
    print(f"ERROR CRÍTICO: No se pudo abrir el puerto {SERIAL_PORT}.")
    print("1. Verifica que el cable USB esté conectado.")
    print("2. Verifica en el Administrador de Dispositivos el número de COM.")
    print("3. Cierra cualquier otro programa que esté usando el puerto.")
    exit()

# --- CONFIGURACIÓN GRÁFICA ---
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
plt.style.use('seaborn-v0_8-darkgrid') # Un estilo visual agradable

def read_serial_data():
    """Lee todas las líneas disponibles en el buffer serial"""
    while ser.in_waiting:
        try:
            # Leemos una línea y quitamos espacios/saltos de línea
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # Esperamos formato: Tiempo,Temp,Setpoint,Heater,Speed
            parts = line.split(',')
            
            # Validación crítica: Deben ser exactamente 5 datos
            if len(parts) == 5:
                t_ms = float(parts[0])
                temp = float(parts[1])
                sp = float(parts[2])
                # heater = int(parts[3]) (No lo graficamos por ahora para no ensuciar)
                speed = int(parts[4])

                # Guardamos en los buffers circulares
                times.append(t_ms / 1000.0) # Convertir a segundos
                temps.append(temp)
                setpoints.append(sp)
                speeds.append(speed)
                
                # Debug opcional en consola para ver que llegan datos
                # print(f"Recibido: {temp}°C @ {speed}Hz")
                
        except ValueError:
            pass # Si llega un dato corrupto (texto a medias), lo ignoramos

def animate(i):
    """Función que Matplotlib llama repetidamente para actualizar la pantalla"""
    read_serial_data()
    
    if len(times) > 1:
        # Gráfico 1: Temperatura
        ax1.clear()
        ax1.plot(times, temps, label='Temp Actual', color='#e74c3c', linewidth=2)
        ax1.plot(times, setpoints, label='Setpoint', color='#2ecc71', linestyle='--')
        ax1.set_ylabel('Temperatura (°C)')
        ax1.set_title('Monitor de Extrusora PET')
        ax1.legend(loc='upper left')
        ax1.grid(True)
        
        # Gráfico 2: Velocidad Motor
        ax2.clear()
        ax2.plot(times, speeds, label='Velocidad Motor', color='#3498db', linewidth=2)
        ax2.set_ylabel('Frecuencia (Hz)')
        ax2.set_xlabel('Tiempo (segundos)')
        ax2.set_ylim(0, 2600) # Fijamos escala un poco mayor a tu MAX (2400)
        ax2.legend(loc='upper left')
        ax2.grid(True)

# Iniciamos la animación
# interval=100 significa que intentará refrescar la gráfica cada 100ms
ani = animation.FuncAnimation(fig, animate, interval=100)

print("Iniciando Dashboard... Cierra la ventana del gráfico para salir.")
plt.show()

# Al cerrar la ventana, cerramos el puerto limpiamente
ser.close()
print("Conexión cerrada.")