import subprocess
from pymavlink import mavutil

# Conectar ao MAVLink
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()

# Informar o usurio
print("Conectado ao sistema:", master.target_system, ", componente:", master.target_component)

# Função para executar o script Python
def run_python_script():
    print("funfou, fdp")
    subprocess.run(['python3', '/home/airvision/hello_world.py'])

# Loop para monitorar os comandos MAVLink
while True:
    msg = master.recv_match(type='COMMAND_LONG', blocking=True)
    if msg:
        if msg.command == 31000:  # Comando personalizado
            if msg.param1 == 1:   # Verifica o valor do parâmetro 1
                run_python_script()
