import { Component, effect, HostBinding, OnInit, signal } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { CommonModule } from '@angular/common';
import { RosService } from '../../../services/ros.service';
import { Trajectory } from '../../models/trajectory';
import { UiStateService } from '../../../services/ui-state.service';
import { TrajectoryService } from '../../../services/trajectory.service';
import { RobotStateService, RobotStateServiceSimulated } from '../../../services/robot-state.service';
import { CdkDragDrop, DragDropModule, moveItemInArray } from '@angular/cdk/drag-drop';


@Component({
  selector: 'app-joints-menu',
  imports: [CommonModule, FormsModule, DragDropModule],
  templateUrl: './joints-menu.component.html',
  styleUrl: './joints-menu.component.scss'
})
export class JointsMenuComponent implements OnInit {

  menuOpen: boolean = true;
  activeSubmenu: 'controls' | 'trajectories' = 'controls';
  cartesianMode: boolean = false;
  errorMessage: string = '';
  isSmallScreen: boolean = false;

  // Articulaciones
  pointAngles: number[] = [0, 0, 0, 0, 0, 0];
  private trajectoryTimeouts: any[] = [];

  // Trayectorias
  trajectories: Trajectory[] = [];
  currentTrajectory: Trajectory | null = null;
  newTrajectory: Trajectory = {
    id: 0,
    name: 'Trayectoria 1',
    points: [],
  };

  // Pose actual para movimiento cartesiano
  currentPose = { x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 0.0 };
  currentSimulatedPose = { x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 0.0 };
  newPoint: { positions: number[]; time: number } = { positions: [0, 0, 0, 0, 0, 0], time: 0 };

  constructor(private rosService: RosService,
    private uiStateService: UiStateService,
    private trajectoryService: TrajectoryService,
    public robot: RobotStateService,
    public robotSimulated: RobotStateServiceSimulated) {
    this.uiStateService.menuOpen$.subscribe(open => this.menuOpen = open);
    this.uiStateService.isSmallScreen$.subscribe(small => this.isSmallScreen = small);
    this.trajectories = this.trajectoryService.getTrajectories();

    effect(() => {
      const currentPose = {
        x: this.robot.x(),
        y: this.robot.y(),
        z: this.robot.z(),
        qx: this.robot.qx(),
        qy: this.robot.qy(),
        qz: this.robot.qz(),
        qw: this.robot.qw()
      };

      const currentSimulatedPose = {
        x: this.robotSimulated.x(),
        y: this.robotSimulated.y(),
        z: this.robotSimulated.z(),
        qx: this.robotSimulated.qx(),
        qy: this.robotSimulated.qy(),
        qz: this.robotSimulated.qz(),
        qw: this.robotSimulated.qw()
      };

      this.currentPose = { ...currentPose };
      this.currentSimulatedPose = { ...currentSimulatedPose };
    });
  }

  ngOnInit(): void {
    this.loadTrajectories();
  }

  setActiveMenu(menu: 'controls' | 'trajectories') {
    this.activeSubmenu = menu;
  }

  toggleCartesianMode() {
    this.cartesianMode = !this.cartesianMode;
  }

  formatAngles(angles: number[]): string {
    if (!angles) return '';
    return angles.map(a => a !== undefined ? a.toFixed(1) : '').join(', ');
  }

  moveSimulatedJoint(joint: string, angle: number) {
    const jointMap: { [key: string]: number } = {
      'junta_0_1': this.pointAngles[0],
      'junta_1_2': this.pointAngles[1],
      'junta_2_3': this.pointAngles[2],
      'junta_3_4': this.pointAngles[3],
      'junta_4_5': this.pointAngles[4],
      'junta_5_6': this.pointAngles[5]
    };

    this.robotSimulated.angle1.set(this.pointAngles[0]);
    this.robotSimulated.angle2.set(this.pointAngles[1]);
    this.robotSimulated.angle3.set(this.pointAngles[2]);
    this.robotSimulated.angle4.set(this.pointAngles[3]);
    this.robotSimulated.angle5.set(this.pointAngles[4]);
    this.robotSimulated.angle6.set(this.pointAngles[5]);
  }

  // TODO: Delete later testing
  rotate6Joint(angle: number) {
    const num = angle / 360;
    this.rosService.publishJoint6Command(num);
  }

  // --- Funciones de trayectorias ---
  loadTrajectories() {
    this.rosService.getTrajectories().subscribe({
      next: (trajectories) => {
        this.trajectories = trajectories;
        console.log("Trajectories:", this.trajectories);
      },
      error: (err) => {
        console.error("Error loading trajectories:", err);
      }
    });
  }

  addPoint() {
    const positions = [parseFloat(this.robotSimulated.angle1().toFixed(2)), parseFloat(this.robotSimulated.angle2().toFixed(2)), parseFloat(this.robotSimulated.angle3().toFixed(2)), parseFloat(this.robotSimulated.angle4().toFixed(2)), parseFloat(this.robotSimulated.angle5().toFixed(2)), parseFloat(this.robotSimulated.angle6().toFixed(2))];
    this.newPoint = { positions, time: parseFloat(this.robotSimulated.time().toFixed(2)) };
    this.newTrajectory.points.push({ angles: [...this.newPoint.positions], time: this.newPoint.time });
  }

  dropPoint(event: CdkDragDrop<any[]>) {
    moveItemInArray(
      this.newTrajectory.points,
      event.previousIndex,
      event.currentIndex
    );
  }

  deletePoint(index: number) {
    this.newTrajectory.points.splice(index, 1);
  }

  updatePoint(index: number) {
    const positions = [this.robot.angle1(), this.robot.angle2(), this.robot.angle3(), this.robot.angle4(), this.robot.angle5(), this.robot.angle6()];
    this.newTrajectory.points[index] = { angles: [...positions], time: this.robot.time() };
  }

  addTrajectory() {
    if (!this.newTrajectory.name || this.newTrajectory.points.length === 0) {
      this.errorMessage = 'Por favor, ingrese un nombre y al menos un punto para la trayectoria.';
      return;
    }

    this.rosService.createTrajectory(this.newTrajectory).subscribe({
      next: (data) => {
        this.loadTrajectories();
      },
      error: (err) => console.error('Error guardando trayectoria:', err)
    });
    this.loadTrajectories();
  }

  updateTrajectory() {
    if (!this.newTrajectory) return;

    this.rosService.updateTrajectory(this.newTrajectory.id, this.newTrajectory).subscribe({
      next: (data) => {
        this.loadTrajectories();
      },
      error: (err) => console.error('Error actualizando trayectoria:', err)
    });
  }

  removeTrajectory() {
    if (!this.currentTrajectory) return;

    this.rosService.deleteTrajectory(this.currentTrajectory.id).subscribe({
      next: () => this.loadTrajectories(),
      error: (err) => console.error('Error eliminando trayectoria:', err)
    });
    this.currentTrajectory = null;
  }

  onTrajectoryChange() {
    if (!this.newTrajectory) return;

    this.rosService.getTrajectory(this.newTrajectory.id).subscribe({
      next: (trajectory) => {
        this.currentTrajectory = {
          id: trajectory.id,
          name: trajectory.name,
          points: trajectory.points
        };

        if (this.currentTrajectory && this.currentTrajectory.points.length > 0) {
          this.newTrajectory = this.currentTrajectory;
        }
      },
      error: (err) => {
        console.error("Error loading trajectory:", err);
      }
    });
  }

  moveToPoint(point: { angles: number[], time: number }) {
    this.pointAngles = [...point.angles]
    this.robotSimulated.angle1.set(point.angles[0]);
    this.robotSimulated.angle2.set(point.angles[1]);
    this.robotSimulated.angle3.set(point.angles[2]);
    this.robotSimulated.angle4.set(point.angles[3]);
    this.robotSimulated.angle5.set(point.angles[4]);
    this.robotSimulated.angle6.set(point.angles[5]);
    this.robotSimulated.time.set(point.time);
  }

  toggleMenu() {
    this.menuOpen = !this.menuOpen;
    this.uiStateService.setMenuOpen(this.menuOpen);
  }

  planMultipleTrajectory() {
    if (!this.newTrajectory || this.newTrajectory.points.length === 0) return;

    const first_pose = {
      angle1: this.robot.angle1() * Math.PI / 180,
      angle2: this.robot.angle2() * Math.PI / 180,
      angle3: this.robot.angle3() * Math.PI / 180,
      angle4: this.robot.angle4() * Math.PI / 180,
      angle5: this.robot.angle5() * Math.PI / 180,
      angle6: this.robot.angle6() * Math.PI / 180,
    }

    const poses = [first_pose];
    poses.push(
      ...this.newTrajectory.points.map(pt => ({
        angle1: pt.angles[0] * Math.PI / 180,
        angle2: pt.angles[1] * Math.PI / 180,
        angle3: pt.angles[2] * Math.PI / 180,
        angle4: pt.angles[3] * Math.PI / 180,
        angle5: pt.angles[4] * Math.PI / 180,
        angle6: pt.angles[5] * Math.PI / 180
      }))
    );

    this.rosService.planTrajectory(first_pose, poses, false).subscribe({
      next: (data: any) => {
        console.log('Planificación de trayectoria exitosa:', data);
        if (!data.trajectory || data.trajectory.length === 0) return;
        const allPoints: any[] = [];
        let totalTime = 0;
        data.trajectory.forEach((poseEntry: any) => {
          let prevTime = 0;
          poseEntry.points.forEach((pt: any) => {
            const delay = (pt.time_from_start - prevTime) * 1000;
            totalTime += delay;
            prevTime = pt.time_from_start;
            const point = {
              angles: pt.positions.map((angle: number) => angle * 180 / Math.PI),
              time: pt.time_from_start
            };
            setTimeout(() => {
              this.moveToPoint(point);
            }, totalTime);
            allPoints.push(point);
          });
        }
        );
      },
      error: (err) => {
        console.error('Error planificando trayectoria:', err);
      }
    });
  }

  // Movimiento cartesiano incremental
  moveCartesian(axis: 'x' | 'y' | 'z', direction: number) {
    // console.log("Current Pose before move:", this.currentPose);
    // Asignamos la pose inicial de las articulaciones del robot (actuales)
    const first_pose = {
      angle1: this.robotSimulated.angle1() * Math.PI / 180,
      angle2: this.robotSimulated.angle2() * Math.PI / 180,
      angle3: this.robotSimulated.angle3() * Math.PI / 180,
      angle4: this.robotSimulated.angle4() * Math.PI / 180,
      angle5: this.robotSimulated.angle5() * Math.PI / 180,
      angle6: this.robotSimulated.angle6() * Math.PI / 180
    };
    // Creamos la mini-trayectoria: primero la pose actual que servirá como fallback
    const poses = [{ ...this.currentSimulatedPose }];
    // Aumentamos/disminuimos distancia en el eje correspondiente
    this.currentSimulatedPose = {
      ...this.currentSimulatedPose,
      x: axis === 'x' ? this.currentSimulatedPose.x + direction * 0.01 : this.currentSimulatedPose.x,
      y: axis === 'y' ? this.currentSimulatedPose.y + direction * 0.01 : this.currentSimulatedPose.y,
      z: axis === 'z' ? this.currentSimulatedPose.z + direction * 0.01 : this.currentSimulatedPose.z,
    };
    poses.push({ ...this.currentSimulatedPose });

    this.rosService.planTrajectory(first_pose, poses, false).subscribe({
      next: (data: any) => {
        if (!data.trajectory || data.trajectory.length === 0) return;

        // Tomamos el último punto de la última pose
        const lastTrajectory = data.trajectory[data.trajectory.length - 1];
        const lastPoint = lastTrajectory.points[lastTrajectory.points.length - 1];
        const point = {
          angles: lastPoint.positions.map((angle: number) => angle * 180 / Math.PI),
          time: lastPoint.time_from_start
        };
        this.moveToPoint(point); // mueve al robot/UI al último punto de la trayectoria
        // console.log('Movimiento cartesiano incremental exitoso:', point);
      },
      error: (err) => {
        console.log("Reverting to previous pose due to error.", poses[0]);
        this.currentSimulatedPose = { ...poses[0] }; // revertir a la pose original en caso de error
        console.error('Error planificando movimiento cartesiano:', err);
      }
    });
  }

  // Movimiento de rotación incremental
  moveRotation(axis: 'qx' | 'qy' | 'qz', direction: number) {
    // console.log("Current Pose before rotation:", this.currentPose);

    // Asignamos la pose inicial de las articulaciones del robot (actuales)
    const poses = [{ ...this.currentSimulatedPose }];
    const first_pose = {
      angle1: this.robotSimulated.angle1() * Math.PI / 180,
      angle2: this.robotSimulated.angle2() * Math.PI / 180,
      angle3: this.robotSimulated.angle3() * Math.PI / 180,
      angle4: this.robotSimulated.angle4() * Math.PI / 180,
      angle5: this.robotSimulated.angle5() * Math.PI / 180,
      angle6: this.robotSimulated.angle6() * Math.PI / 180
    };

    // Ángulo incremental a rotar (por ejemplo, 5 grados convertido a radianes)
    const angleIncrementRad = direction * 5 * Math.PI / 180;
    // Normalizar el quaternion actual
    let q = [this.currentSimulatedPose.qx, this.currentSimulatedPose.qy, this.currentSimulatedPose.qz, this.currentSimulatedPose.qw] as [number, number, number, number];

    // Definir el eje de rotación
    let axisVec: [number, number, number];
    switch (axis) {
      case 'qx': axisVec = [1, 0, 0]; break;
      case 'qy': axisVec = [0, 1, 0]; break;
      case 'qz': axisVec = [0, 0, 1]; break;
      default: axisVec = [0, 0, 1];
    }

    // Crear cuaternión incremental
    const qIncrement = quatFromAxisAngle(axisVec, angleIncrementRad);

    // Multiplicar: nueva rotación = qIncrement * qActual
    const qNew = quatMultiply(qIncrement, q);

    // Actualizar la pose
    this.currentSimulatedPose = {
      ...this.currentSimulatedPose,
      qx: qNew[0],
      qy: qNew[1],
      qz: qNew[2],
      qw: qNew[3],
    };

    poses.push({ ...this.currentSimulatedPose });

    this.rosService.planTrajectory(first_pose, poses, false).subscribe({
      next: (data: any) => {
        if (!data.trajectory || data.trajectory.length === 0) return;

        const lastTrajectory = data.trajectory[data.trajectory.length - 1];
        const lastPoint = lastTrajectory.points[lastTrajectory.points.length - 1];

        const point = {
          angles: lastPoint.positions.map((angle: number) => angle * 180 / Math.PI),
          time: lastPoint.time_from_start
        };

        this.moveToPoint(point);
        console.log('Rotación incremental exitosa:', point);
      },
      error: (err) => {
        this.currentSimulatedPose = { ...poses[0] }; // revertir a la pose original en caso de error
        console.error('Error planificando rotación:', err);
      }
    });
  }

  private clearTrajectoryTimeouts() {
    this.trajectoryTimeouts.forEach(timeoutId => clearTimeout(timeoutId));
    this.trajectoryTimeouts = [];
  }

  planTrajectory() {
    // Cancelar cualquier animación anterior antes de iniciar una nueva
    this.clearTrajectoryTimeouts();

    const first_pose = {
      angle1: this.robot.angle1() * Math.PI / 180,
      angle2: this.robot.angle2() * Math.PI / 180,
      angle3: this.robot.angle3() * Math.PI / 180,
      angle4: this.robot.angle4() * Math.PI / 180,
      angle5: this.robot.angle5() * Math.PI / 180,
      angle6: this.robot.angle6() * Math.PI / 180,
    }

    const targetPose = {
      angle1: this.robotSimulated.angle1() * Math.PI / 180,
      angle2: this.robotSimulated.angle2() * Math.PI / 180,
      angle3: this.robotSimulated.angle3() * Math.PI / 180,
      angle4: this.robotSimulated.angle4() * Math.PI / 180,
      angle5: this.robotSimulated.angle5() * Math.PI / 180,
      angle6: this.robotSimulated.angle6() * Math.PI / 180,
    }

    console.log(targetPose);

    const poses = [
      first_pose,
      targetPose
    ]

    this.rosService.planTrajectory(first_pose, poses, this.cartesianMode).subscribe({
      next: (data: any) => {
        console.log('Planificación de trayectoria exitosa:', data);
        if (!data.trajectory || data.trajectory.length === 0) return;

        const allPoints: any[] = [];
        let totalTime = 0;
        data.trajectory.forEach((poseEntry: any) => {
          let prevTime = 0;
          poseEntry.points.forEach((pt: any) => {
            const delay = (pt.time_from_start - prevTime) * 1000;
            totalTime += delay;
            prevTime = pt.time_from_start;
            const point = {
              angles: pt.positions.map((angle: number) => angle * 180 / Math.PI),
              time: pt.time_from_start
            };
            const timeoutId = setTimeout(() => {
              this.moveToPoint(point);
            }, totalTime);
            this.trajectoryTimeouts.push(timeoutId);
            allPoints.push(point);
          });
        });
      },
      error: (err) => {
        console.error('Error planificando trayectoria:', err);
      }
    });
  }


  executePlannedTrajectory() {
    this.rosService.executeTrajectory().subscribe({
      next: (res) => {
        console.log('Ejecución de trayectoria iniciada:', res);
      },
      error: (err) => {
        console.error('Error ejecutando trayectoria:', err);
      }
    });
  }
}

// Crear un cuaternión a partir de un eje y un ángulo en radianes
function quatFromAxisAngle(axis: [number, number, number], angleRad: number): [number, number, number, number] {
  const halfAngle = angleRad / 2;
  const s = Math.sin(halfAngle);
  return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(halfAngle)];
}

// Multiplicar dos cuaterniones q1 * q2
function quatMultiply(q1: [number, number, number, number], q2: [number, number, number, number]): [number, number, number, number] {
  const [x1, y1, z1, w1] = q1;
  const [x2, y2, z2, w2] = q2;

  return [
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
  ];
}