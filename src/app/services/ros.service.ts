import { Injectable } from '@angular/core';
import ROSLIB from 'roslib';

@Injectable({
  providedIn: 'root'
})
export class RosService {
  private ros: ROSLIB.Ros;

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
}
