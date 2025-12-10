import serial
import time

# CAMBIA ESTO POR TU PUERTO
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200

print(f"Abriendo {SERIAL_PORT} a {BAUD_RATE}...")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # Esperar reinicio
    ser.reset_input_buffer()
    
    print("Escuchando... (Presiona Ctrl+C para salir)")
    while True:
        if ser.in_waiting > 0:
            # Leemos bytes y tratamos de decodificar
            raw_data = ser.readline()
            try:
                decoded_data = raw_data.decode('utf-8').strip()
                print(f"Recibido: {decoded_data}")
            except:
                print(f"Bytes raros: {raw_data}")
        else:
            # Pequeña pausa para no saturar CPU
            time.sleep(0.01)

except serial.SerialException:
    print("ERROR: No se pudo abrir el puerto. ¿Está ocupado?")
except KeyboardInterrupt:
    print("\nCerrando.")
    ser.close()