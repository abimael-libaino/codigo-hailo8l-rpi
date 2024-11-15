#!/usr/bin/python3

import time
from pymavlink import mavutil

# Conectar ao controlador de voo
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

# Esperar por um heartbeat
master.wait_heartbeat()

# Informar o usuário
print("Conectado ao sistema:", master.target_system, ", componente:", master.target_component)

while True:
    # Receber mensagens MAVLink
    msg = master.recv_match(blocking=False)
    
    if msg:
        # Verifica se a mensagem é do tipo 'GLOBAL_POSITION_INT'
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1E7  # Converte para graus
            lon = msg.lon / 1E7  # Converte para graus
            alt = msg.alt / 1000.0  # Converte para metros
            print(f"Coordenadas GPS - Latitude: {lat}, Longitude: {lon}, Altitude: {alt} m")
        
        # Verifica se a mensagem é do tipo 'CAMERA_TRIGGER'
        elif msg.get_type() == 'CAMERA_TRIGGER':
            print(f"Disparo da câmera detectado! {msg}")
        
        # Também pode monitorar 'STATUSTEXT' para mensagens relacionadas à câmera
        elif msg.get_type() == 'STATUSTEXT':
            status_text = msg.text.lower()
            if "camera" in status_text:
                print(f"Mensagem relacionada à câmera: {status_text}")
    
    # Aguarda um pouco antes de continuar
    time.sleep(0.05)
