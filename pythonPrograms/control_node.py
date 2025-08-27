# control_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from fastapi import FastAPI, HTTPException, Request
import uvicorn
import threading
import secrets
import sys
import logging
from fastapi.middleware.cors import CORSMiddleware

# Contraseña única al iniciar
PASSWORD = secrets.token_urlsafe(16)

origins = [
    "http://localhost:4200",
    "*"
]

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,  # Cambia esto a los orígenes permitidos
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ros_node = None  # Referencia al nodo ROS2
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class ControlNode(Node):
    def __init__(self):
        super().__init__('secure_joint_controller')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.joint6_pub = self.create_publisher(Float64, '/joint6_controller/commands', 10)
        self.lock = threading.Lock()
        logging.info("ControlNode initialized and ready to receive commands.")

    def publish_joint_state(self, name, position):
        with self.lock:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = name
            msg.position = [float(p) for p in position]
            self.publisher.publish(msg)
            
    def publish_joint6(self, value: float):
        with self.lock:
            msg = Float64()
            msg.data = float(value)
            self.joint6_pub.publish(msg)
            logging.info(f"Published joint6 command: {value}")

# API route segura
@app.post("/send_command")
async def send_command(request: Request):
    data = await request.json()
    if data.get("password") != PASSWORD:
        raise HTTPException(status_code=401, detail="Unauthorized")
    
    name = data.get("name", [])
    position = data.get("position", [])
    
    if not isinstance(name, list) or not isinstance(position, list):
        raise HTTPException(status_code=400, detail="Invalid input")
    
    if len(name) != len(position):
        raise HTTPException(status_code=400, detail="Length of 'name' and 'position' must be equal")

    ros_node.publish_joint_state(name, position)
    logging.info(f"Published joint states for joints {name} with positions {position}")
    return {"status": "ok"}

# Nueva ruta para el comando de la junta 6
@app.post("/send_joint6_command")
async def send_joint6_command(request: Request):
    data = await request.json()
    if data.get("password") != PASSWORD:
        raise HTTPException(status_code=401, detail="Unauthorized")

    value = data.get("value")
    if value is None:
        raise HTTPException(status_code=400, detail="Missing 'value' for joint6 command")

    ros_node.publish_joint6(value)
    return {"status": "ok", "joint6_value": value}

def ros_thread():
    global ros_node
    rclpy.init()
    ros_node = ControlNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

def main():
    print(f"Generated password for control access: {PASSWORD}", file=sys.stdout, flush=True)
    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == '__main__':
    main()
