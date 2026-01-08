import { Injectable } from '@angular/core';
import ROSLIB from 'roslib';
import { environment } from '../../environments/environment.development';
import { First_Pose, Pose, Trajectory, TrajectoryPoint } from '../components/models/trajectory';
import { HttpClient } from '@angular/common/http';
import { catchError, Observable, of, throwError } from 'rxjs';
import { AuthService } from './auth.service';

@Injectable({
  providedIn: 'root'
})
export class RosService {
  private ros: ROSLIB.Ros;
  private jointStateTopic: ROSLIB.Topic;
  private readonly API_URL = environment.API_URL;
  private readonly ROSBRIDGE_SERVER_URL = environment.ROSBRIDGE_SERVER_URL;

  constructor(private http: HttpClient,
    private authService: AuthService
  ) {
    this.ros = new ROSLIB.Ros({
      url: this.ROSBRIDGE_SERVER_URL
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

  private get password(): string {
    const pwd = this.authService.getPassword();
    if (!pwd) {
      throw new Error('Usuario no autenticado. La contraseña es requerida para las operaciones de ROS.');
    }
    return pwd;
  }

  public getUrdf() {
    return new ROSLIB.Param({
      ros: this.ros,
      name: '/robot_state_publisher:robot_description'
    });
  }

  subscribeToJointStates(callback: (message: ROSLIB.Message) => void) {
    this.jointStateTopic.subscribe((message: ROSLIB.Message) => {
      // console.log('Received joint states:', message);
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
      password: this.password,
      name: names,
      position: positions
    };

    fetch(this.API_URL + '/send_command', {
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

    /*this.jointStateTopic.publish(new ROSLIB.Message({
      name: names,
      position: positions
    }));
    console.log('Estado publicado con éxito', names, positions);*/
  }

  public planTrajectory(firstPose: First_Pose, targetPose: any[], cartesian: boolean, velocity: number): Observable<any> {
    const normalizedVelocity = Math.max(0, Math.min(1, velocity / 100));
    const payload = {
      password: this.password,
      first_pose: firstPose,
      poses: targetPose,
      cartesian: cartesian,
      velocity: normalizedVelocity
    };
    return this.http.post(`${this.API_URL}/plan_trajectory`, payload).pipe(
      catchError(err => {
        return throwError(() => new Error('Error planificando trayectoria'));
      })
    );
  }

  public executeTrajectory() {
    const payload = {
      password: this.password,
    }
    return this.http.post(`${this.API_URL}/execute_trajectory`, payload).pipe(
      catchError(err => {
        return throwError(() => new Error('Error ejecutando trayectoria'));
      })
    );
  }

  public executeCommand(cmd: string): Observable<any> {
    const payload = {
      password: this.password,
    };
    return this.http.post(`${this.API_URL}/send_command/${cmd}`, payload).pipe(
      catchError(err => {
        return throwError(() => new Error('Error ejecutando comando'));
      })
    );
  }

  // --- CRUD Trajectory
  createTrajectory(trajectory: Trajectory): Observable<any> {
    const payload = {
      password: this.password,
      name: trajectory.name,
      points: trajectory.points
    };
    return this.http.post(`${this.API_URL}/trajectories`, payload).pipe(
      catchError(err => {
        console.error('Error creando trayectoria:', err);
        return of(null);
      })
    );
  }

  updateTrajectory(id: number, trajectory: Trajectory): Observable<any> {
    const payload = {
      password: this.password,
      name: trajectory.name,
      points: trajectory.points
    };
    return this.http.put(`${this.API_URL}/trajectories/${id}`, payload).pipe(
      catchError(err => {
        console.error('Error actualizando trayectoria:', err);
        return of(null);
      })
    );
  }

  getTrajectories(): Observable<Trajectory[]> {
    return this.http.get<Trajectory[]>(this.API_URL + '/trajectories').pipe(
      catchError((err) => {
        console.error('Error leyendo trayectorias:', err);
        return of([]);
      })
    );
  }

  getTrajectory(id: number): Observable<Trajectory> {
    return this.http.get<Trajectory>(`${this.API_URL}/trajectories/${id}`).pipe(
      catchError((err) => {
        console.error('Error leyendo trayectoria:', err);
        return of({ id: 0, name: 'Error', points: [] });
      })
    );
  }

  deleteTrajectory(index: number): Observable<any> {
    return this.http.delete(`${this.API_URL}/trajectories/${index}`,
      { body: { password: this.password } }
    ).pipe(
      catchError(err => {
        console.error('Error eliminando trayectoria:', err);
        return of(null);
      })
    );
  }

  // --- Calibration API ---
  calibrate(
    joint: number,
    brake_release: number,
    move_to_position: number
  ): Observable<any> {

    const payload = {
      joint: joint,
      brake_release: brake_release,
      move_to_position: move_to_position,
      password: this.password
    };
    return this.http.post(
      `${this.API_URL}/calibrate`,
      payload
    ).pipe(
      catchError(err => {
        console.error('Error calibrando:', err);
        return throwError(() => err);
      })
    );
  }
}