#!/usr/bin/env python3

"""
Script para se conectar a um veículo via MAVLink e receber dados de posição global.
Utiliza a biblioteca pymavlink para estabelecer conexão serial com o veículo.
"""

from pymavlink import mavutil
import sys
import time

def conectar_veiculo(porta='/dev/ttyAMA0', baud=115200):
    """
    Estabelece conexão com o veículo via MAVLink.
    
    Args:
        porta (str): Porta serial do dispositivo
        baud (int): Taxa de transmissão em baud
        
    Returns:
        mavutil.mavlink_connection: Objeto de conexão com o veículo
    """
    try:
        print(f"Conectando ao veículo em {porta}...")
        return mavutil.mavlink_connection(porta, baud=baud)
    except Exception as e:
        print(f"Erro ao conectar: {e}")
        sys.exit(1)

def esperar_heartbeat(veiculo):
    """
    Aguarda receber o primeiro heartbeat do veículo.
    
    Args:
        veiculo: Objeto de conexão mavutil
    """
    print("Aguardando heartbeat...")
    try:
        veiculo.wait_heartbeat()
        print(f"Heartbeat recebido do sistema (sistema {veiculo.target_system} "
              f"componente {veiculo.target_component})")
    except Exception as e:
        print(f"Erro ao receber heartbeat: {e}")
        sys.exit(1)

def esperar_primeira_posicao(veiculo, timeout=30):
    """
    Aguarda receber a primeira mensagem de posição do veículo.
    
    Args:
        veiculo: Objeto de conexão mavutil
        timeout (int): Tempo máximo de espera em segundos
        
    Returns:
        dict: Primeira mensagem de posição recebida ou None se houver timeout
    """
    print("Aguardando primeira mensagem de posição...")
    tempo_inicio = time.time()
    
    while time.time() - tempo_inicio < timeout:
        try:
            msg = veiculo.recv_match(
                type='GLOBAL_POSITION_INT',
                blocking=True,
                timeout=1.0
            )
            
            if msg is not None:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.relative_alt / 1000.0
                
                print(f"Primeira posição recebida:")
                print(f"Latitude: {lat:.7f}°")
                print(f"Longitude: {lon:.7f}°")
                print(f"Altitude Relativa: {alt:.2f}m")
                return msg
                
        except Exception as e:
            print(f"Erro ao aguardar posição: {e}")
    
    print(f"Timeout após {timeout} segundos sem receber posição")
    return None

def monitorar_posicao(veiculo):
    """
    Monitora continuamente a posição global do veículo.
    
    Args:
        veiculo: Objeto de conexão mavutil
    """
    print("\nIniciando monitoramento contínuo de posição...")
    while True:
        try:
            msg = veiculo.recv_match(
                type='GLOBAL_POSITION_INT',
                blocking=True,
                timeout=1.0
            )
            
            if msg is not None:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.relative_alt / 1000.0
                
                print(f"Posição: Lat: {lat:.7f}°, Lon: {lon:.7f}°, Alt Relativa: {alt:.2f}m")
            
        except KeyboardInterrupt:
            print("\nMonitoramento interrompido pelo usuário")
            sys.exit(0)
        except Exception as e:
            print(f"Erro ao receber dados: {e}")
            time.sleep(1)

def main():
    """Função principal do script"""
    # Estabelece conexão
    veiculo = conectar_veiculo()
    
    # Aguarda heartbeat
    esperar_heartbeat(veiculo)
    
    # Aguarda primeira posição
    primeira_posicao = esperar_primeira_posicao(veiculo)
    if primeira_posicao is None:
        print("Não foi possível obter a posição inicial. Encerrando...")
        sys.exit(1)
    
    # Inicia monitoramento contínuo
    monitorar_posicao(veiculo)

if __name__ == "__main__":
    main()
