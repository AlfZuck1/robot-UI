import socket

# Configuración
ARDUINO_PORT = 8888             # Puerto donde Arduino envía
CONTAINER_IP = "172.19.209.239" # IP de Docker/WSL
CONTAINER_PORT = 8888

# Socket principal
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", ARDUINO_PORT))

# Diccionario para recordar desde qué IP:puerto se conectó Arduino
arduino_addr = None

print(f"Proxy UDP bidireccional iniciado: 0.0.0.0:{ARDUINO_PORT} <-> {CONTAINER_IP}:{CONTAINER_PORT}")

while True:
    data, addr = sock.recvfrom(4096)
    
    # Si viene de Arduino
    if addr != (CONTAINER_IP, CONTAINER_PORT):
        arduino_addr = addr
        sock.sendto(data, (CONTAINER_IP, CONTAINER_PORT))
    
    # Si viene del contenedor
    else:
        if arduino_addr is not None:
            sock.sendto(data, arduino_addr)
