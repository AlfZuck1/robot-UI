import { Injectable } from '@angular/core';
import ROSLIB from 'roslib';

@Injectable({
  providedIn: 'root'
})
export class RosService {
  private ros: ROSLIB.Ros;
  private jointStateTopic: ROSLIB.Topic;
  private readonly API_URL = 'http://localhost:8000/send_command';
  private readonly PASSWORD = 'E9wW8XulJJka9cyK1sCSrA';

  constructor() {
    this.ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    this.ros.on('connection', () => {
      console.log('Conectado a ROS.');
    });

    this.ros.on('error', (error) => {
      console.error('Error en la conexión ROS:', error);
    });

    this.ros.on('close', () => {
      console.log('Conexión ROS cerrada.');
    });

    this.jointStateTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/joint_states',
      messageType: 'sensor_msgs/JointState'
    });
  }

  public getUrdf() {
    return new ROSLIB.Param({
      ros: this.ros,
      name: '/robot_state_publisher:robot_description'
    });
  }

  subscribeToJointStates(callback: (message: ROSLIB.Message) => void) {
    this.jointStateTopic.subscribe((message: ROSLIB.Message) => {
      console.log('Received joint states:', message);
      callback(message);
    });
  }

  public publishExample() {
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/example_topic',
      messageType: 'std_msgs/String'
    });

    const message = new ROSLIB.Message({
      data: 'Hola desde Angular!'
    });

    topic.publish(message);
  }

  publishJointState(names: string[], positions: number[]) {
    const payload = {
      password: this.PASSWORD,
      name: names,
      position: positions
    };

    fetch(this.API_URL, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    })
      .then(async (response) => {
        if (!response.ok) {
          const err = await response.json();
          console.error('Error publicando estado:', err.detail || response.statusText);
          return;
        }
        console.log('Estado publicado con éxito');
      })
      .catch(err => {
        console.error('Error al conectar con API:', err);
      });
  }
}
