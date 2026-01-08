import { CommonModule } from '@angular/common';
import { AfterViewInit, Component, effect, ElementRef, ViewChild } from '@angular/core';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import URDFLoader from 'urdf-loader';
import { RobotStateService, RobotStateServiceSimulated } from '../../../services/robot-state.service';
import { RosService } from '../../../services/ros.service';
import { UiStateService } from '../../../services/ui-state.service';
import { First_Pose, Pose } from '../../models/trajectory';

@Component({
  selector: 'app-robot-display',
  imports: [CommonModule],
  templateUrl: './robot-display.component.html',
  styleUrl: './robot-display.component.scss'
})
export class RobotDisplayComponent implements AfterViewInit {
  @ViewChild('canvasContainer', { static: true }) canvasContainer!: ElementRef;
  isSmallScreen: boolean = false;
  menuOpen: boolean = false;

  scene!: THREE.Scene;
  camera!: THREE.PerspectiveCamera;
  renderer!: THREE.WebGLRenderer;
  base!: THREE.Object3D;
  joint1!: THREE.Object3D;
  joint2!: THREE.Object3D;
  joint3!: THREE.Object3D;
  joint4!: THREE.Object3D;
  joint5!: THREE.Object3D;
  joint6!: THREE.Object3D;

  robotModel: any;
  robotModelSimulated: any;
  endEffectorAxisHelper!: THREE.AxesHelper;
  endEffectorAxisHelperSimulated!: THREE.AxesHelper;
  manualMode: boolean = false;
  controls!: OrbitControls;

  rotateControls!: TransformControls;
  translateControls!: TransformControls;
  draggingTarget = false;
  private ikThrottleTimer: any = null;
  private ikThrottleDelay = 200;

  constructor(private rosService: RosService,
    private uiStateService: UiStateService,
    public robot: RobotStateService,
    public robotSimulated: RobotStateServiceSimulated
  ) {
    this.uiStateService.isSmallScreen$.subscribe(isSmall => {
      this.isSmallScreen = isSmall;
      this.resetCameraPosition();
    });

    this.uiStateService.menuOpen$.subscribe(menuOpen => {
      this.menuOpen = menuOpen;
      setTimeout(() => {
        const w = this.canvasContainer.nativeElement.clientWidth;
        const h = this.canvasContainer.nativeElement.clientHeight;
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(w, h, false);
      }, 200);
    });

    window.addEventListener('resize', () => {
      const small = window.innerWidth < 768;
      this.uiStateService.setSmallScreen(small);
    });

    this.uiStateService.resetCamera$.subscribe(() => {
      this.resetCameraPosition();
    });

    this.uiStateService.manualMode$.subscribe(mode => { this.manualMode = mode; });

    effect(() => {
      const anglesRad = [
        this.robot.angle1() * Math.PI / 180,
        this.robot.angle2() * Math.PI / 180,
        this.robot.angle3() * Math.PI / 180,
        this.robot.angle4() * Math.PI / 180,
        this.robot.angle5() * Math.PI / 180,
        this.robot.angle6() * Math.PI / 180,
      ];

      const anglesRadSimulated = [
        this.robotSimulated.angle1() * Math.PI / 180,
        this.robotSimulated.angle2() * Math.PI / 180,
        this.robotSimulated.angle3() * Math.PI / 180,
        this.robotSimulated.angle4() * Math.PI / 180,
        this.robotSimulated.angle5() * Math.PI / 180,
        this.robotSimulated.angle6() * Math.PI / 180,
      ]

      if (this.robotModel) {
        const names = ['junta_0_1', 'junta_1_2', 'junta_2_3', 'junta_3_4', 'junta_4_5', 'junta_5_6'];
        for (let i = 0; i < names.length; i++) {
          const joint = this.robotModel.joints[names[i]];
          if (joint) {
            joint.setJointValue(anglesRad[i]);
          }
        }
      }

      if (this.robotModelSimulated) {
        const namesSim = ['junta_0_1', 'junta_1_2', 'junta_2_3', 'junta_3_4', 'junta_4_5', 'junta_5_6'];
        for (let i = 0; i < namesSim.length; i++) {
          const jointSim = this.robotModelSimulated.joints[namesSim[i]];
          if (jointSim) {
            jointSim.setJointValue(anglesRadSimulated[i]);
          }
        }
      }
    });
  }


  ngAfterViewInit(): void {
    this.initScene();
    this.animate();
    this.rosService.subscribeToJointStates((message) => {
      this.updateRobotMovement(message)
    });
  }

  initScene() {
    // Escena y fondo
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf0f0f0);

    // Renderizador
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0xf0f0f0);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    const width = this.canvasContainer.nativeElement.clientWidth;
    const height = this.canvasContainer.nativeElement.clientHeight;
    this.renderer.setSize(width, height, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.canvasContainer.nativeElement.appendChild(this.renderer.domElement);

    // Cámara
    this.camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 100);
    if (this.isSmallScreen) {
      this.camera.position.set(0, 5.5, -5.5);
    }
    else {
      this.camera.position.set(0, 3, -4);
    }
    this.camera.lookAt(0, 1.0, 0);

    // Luz ambiental
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    this.scene.add(ambientLight);

    // Luz direccional
    const dirLight = new THREE.DirectionalLight(0xffffff, 2);
    dirLight.position.set(-4, 10, -4);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 0.1;
    dirLight.shadow.camera.far = 30;
    this.scene.add(dirLight);

    // Carga de piso y área de seguridad
    this.loadFloor();

    // Cargar modelo URDF
    this.loadRobotModel();

    // Fondo global
    this.createGlobalBackground();

    // Ajuste de tamaño responsivo
    window.addEventListener('resize', () => {
      const w = this.canvasContainer.nativeElement.clientWidth;
      const h = this.canvasContainer.nativeElement.clientHeight;
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(w, h, false);
    });

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 1.0, 0);
    this.controls.update();
  }

  updateRobotMovement(message: any) {
    const jointPositionsMap: { [key: string]: number } = {};
    message.name.forEach((jointName: string, i: number) => {
      jointPositionsMap[jointName] = message.position[i];
    });

    this.robot.angle1.set(Math.round(jointPositionsMap['junta_0_1'] * 180 / Math.PI * 10) / 10);
    this.robot.angle2.set(Math.round(jointPositionsMap['junta_1_2'] * 180 / Math.PI * 10) / 10);
    this.robot.angle3.set(Math.round(jointPositionsMap['junta_2_3'] * 180 / Math.PI * 10) / 10);
    this.robot.angle4.set(Math.round(jointPositionsMap['junta_3_4'] * 180 / Math.PI * 10) / 10);
    this.robot.angle5.set(Math.round(jointPositionsMap['junta_4_5'] * 180 / Math.PI * 10) / 10);
    this.robot.angle6.set(Math.round(jointPositionsMap['junta_5_6'] * 180 / Math.PI * 10) / 10);

  }

  updateRobotSimulatedMovement(angles: number[]) {
    this.robotSimulated.angle1.set(angles[0]);
    this.robotSimulated.angle2.set(angles[1]);
    this.robotSimulated.angle3.set(angles[2]);
    this.robotSimulated.angle4.set(angles[3]);
    this.robotSimulated.angle5.set(angles[4]);
    this.robotSimulated.angle6.set(angles[5]);
  }

  animate() {
    requestAnimationFrame(() => this.animate());

    if (this.robotModel && this.endEffectorAxisHelper) {
      const endEffectorReal = this.robotModel.links["ee_link"];
      if (endEffectorReal) {
        const pos = new THREE.Vector3();
        const quat = new THREE.Quaternion();
        endEffectorReal.getWorldPosition(pos);
        endEffectorReal.getWorldQuaternion(quat);
        this.endEffectorAxisHelper.position.copy(pos);
        this.endEffectorAxisHelper.quaternion.copy(quat);
      }
    }
    if (!this.draggingTarget && this.robotModelSimulated && this.endEffectorAxisHelperSimulated) {
      const endEffectorSim = this.robotModelSimulated.links["ee_link"];
      if (endEffectorSim) {
        endEffectorSim.getWorldPosition(this.endEffectorAxisHelperSimulated.position);
        endEffectorSim.getWorldQuaternion(this.endEffectorAxisHelperSimulated.quaternion);
        this.translateControls.attach(this.endEffectorAxisHelperSimulated);
        this.rotateControls.attach(this.endEffectorAxisHelperSimulated);
      }
    }
    this.renderer.render(this.scene, this.camera);
    this.getEndEffectorPose(this.robotModel, this.robot);
    this.getEndEffectorPose(this.robotModelSimulated, this.robotSimulated);
  }

  loadRobotModel() {
    // Cargar URDF desde ROS
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    const param = this.rosService.getUrdf();

    param.get((value: string) => {
      if (!value) {
        console.error('No se pudo obtener el URDF desde el parámetro de ROS.');
        return;
      }
      try {
        // Cargar modelo URDF para el robot real y simulado
        this.robotModel = loader.parse(value);
        this.robotModel.rotation.x = -Math.PI / 2;
        this.robotModel.scale.set(3, 3, 3);
        this.scene.add(this.robotModel);

        this.robotModelSimulated = loader.parse(value);
        this.robotModelSimulated.rotation.x = -Math.PI / 2;
        this.robotModelSimulated.scale.set(3, 3, 3);
        this.scene.add(this.robotModelSimulated);

        // Ejes en el efector final
        this.endEffectorAxisHelper = new THREE.AxesHelper(0.2);
        this.scene.add(this.endEffectorAxisHelper);
        this.endEffectorAxisHelperSimulated = new THREE.AxesHelper(0.2);
        this.scene.add(this.endEffectorAxisHelperSimulated);

        // Manejar sombras y materiales después de un pequeño retraso para que se realice el renderizado
        setTimeout(() => {
          // Mejor manejo de sombras para todos los meshes
          this.robotModel.traverse((child: THREE.Object3D) => {
            if ((child as THREE.Mesh).isMesh) {
              (child as THREE.Mesh).castShadow = true;
              (child as THREE.Mesh).receiveShadow = true;
            }
          });

          this.robotModelSimulated.traverse((child: THREE.Object3D) => {
            if ((child as THREE.Mesh).isMesh) {
              const mesh = child as THREE.Mesh;
              mesh.receiveShadow = false;
              mesh.castShadow = false;
              mesh.material = new THREE.MeshBasicMaterial({
                color: 0x0000ff,
                opacity: 0.3,
                transparent: true,
                depthWrite: false,
              });
            }
          });
        }, 200);

      } catch (e) {
        console.error('Error al cargar el modelo URDF:', e);
      }
    });

    this.rotateControls = new TransformControls(this.camera, this.renderer.domElement);
    this.rotateControls.setMode('rotate');
    this.rotateControls.setSpace('world');
    this.rotateControls.setSize(0.2);
    this.scene.add(this.rotateControls.getHelper());
    this.rotateControls.enabled = true;
    this.rotateControls.attach(this.endEffectorAxisHelperSimulated);
    this.rotateControls.addEventListener('dragging-changed', (event) => {
      this.controls.enabled = !event.value;
      this.draggingTarget = event.value as boolean;
    });

    this.rotateControls.addEventListener('objectChange', () => {
      if (!this.draggingTarget) return;
      if (this.ikThrottleTimer) return;

      this.ikThrottleTimer = setTimeout(() => {
        const pos = this.endEffectorAxisHelperSimulated.position.clone();
        const quat = this.endEffectorAxisHelperSimulated.quaternion.clone();
        this.onTargetPoseChanged(pos, quat);
        this.ikThrottleTimer = null;
      }, this.ikThrottleDelay);
    });


    this.translateControls = new TransformControls(this.camera, this.renderer.domElement);
    this.translateControls.setMode('translate');
    this.translateControls.setSpace('world');
    this.translateControls.setSize(0.2);
    this.scene.add(this.translateControls.getHelper());
    this.translateControls.enabled = true;
    this.translateControls.attach(this.endEffectorAxisHelperSimulated);
    this.translateControls.addEventListener('dragging-changed', (event) => {
      this.controls.enabled = !event.value;
      this.draggingTarget = event.value as boolean;
    });

    this.translateControls.addEventListener('objectChange', () => {
      if (!this.draggingTarget) return;
      if (this.ikThrottleTimer) return;

      this.ikThrottleTimer = setTimeout(() => {
        const pos = this.endEffectorAxisHelperSimulated.position.clone();
        const quat = this.endEffectorAxisHelperSimulated.quaternion.clone();

        this.onTargetPoseChanged(pos, quat);
        this.ikThrottleTimer = null;
      }, this.ikThrottleDelay);
    });
  }

  onTargetPoseChanged(position: THREE.Vector3, quaternion: THREE.Quaternion) {

    const rosPosition = new THREE.Vector3(
      position.x / 3,
      -position.z / 3,
      position.y / 3
    );

    const modelInclination = new THREE.Quaternion()
      .setFromEuler(new THREE.Euler(Math.PI / 2, 0, 0));

    const rosQuaternion = quaternion.clone();
    rosQuaternion.premultiply(modelInclination);

    const firstPose: First_Pose = {
      angle1: this.robotSimulated.angle1() * Math.PI / 180,
      angle2: this.robotSimulated.angle2() * Math.PI / 180,
      angle3: this.robotSimulated.angle3() * Math.PI / 180,
      angle4: this.robotSimulated.angle4() * Math.PI / 180,
      angle5: this.robotSimulated.angle5() * Math.PI / 180,
      angle6: this.robotSimulated.angle6() * Math.PI / 180,
    };

    const pose: Pose = {
      x: rosPosition.x,
      y: rosPosition.y,
      z: rosPosition.z,
      qx: rosQuaternion.x,
      qy: rosQuaternion.y,
      qz: rosQuaternion.z,
      qw: rosQuaternion.w
    };

    console.log('📤 Pose enviada a IK:', pose);

    this.rosService.getIk(pose, firstPose).subscribe({
      next: (data) => {
        this.updateRobotSimulatedMovement([
          data.joint_positions[0] * 180 / Math.PI,
          data.joint_positions[1] * 180 / Math.PI,
          data.joint_positions[2] * 180 / Math.PI,
          data.joint_positions[3] * 180 / Math.PI,
          data.joint_positions[4] * 180 / Math.PI,
          data.joint_positions[5] * 180 / Math.PI,
        ]);
      },
      error: err => console.error('IK error', err)
    });
  }

  loadFloor() {
    // Piso con textura de tablero y área de seguridad
    const floorSize = 20;
    const checkerTexture = new THREE.TextureLoader().load('images/floor.jpeg');
    checkerTexture.wrapS = checkerTexture.wrapT = THREE.RepeatWrapping;
    checkerTexture.repeat.set(floorSize / 4, floorSize / 4);
    checkerTexture.anisotropy = 8;

    const floorGeometry = new THREE.PlaneGeometry(floorSize, floorSize);
    const floorMaterial = new THREE.MeshStandardMaterial({
      map: checkerTexture,
      side: THREE.DoubleSide,
      roughness: 0.7,
      metalness: 0.1
    });
    const floor = new THREE.Mesh(floorGeometry, floorMaterial);
    floor.receiveShadow = true;
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = 0;
    this.scene.add(floor);

    // Área de seguridad (círculo rojo semitransparente con borde)
    const safetyRadius = 2.1;
    const safetyGeometry = new THREE.CircleGeometry(safetyRadius, 128);
    const safetyMaterial = new THREE.MeshBasicMaterial({
      color: 0xff0000,
      opacity: 0.2,
      transparent: true,
      depthWrite: false
    });
    const safetyArea = new THREE.Mesh(safetyGeometry, safetyMaterial);
    safetyArea.rotation.x = -Math.PI / 2;
    safetyArea.position.y = 0.02;
    this.scene.add(safetyArea);

    // Borde del área de seguridad
    const edgeGeometry = new THREE.RingGeometry(safetyRadius - 0.03, safetyRadius, 128);
    const edgeMaterial = new THREE.MeshBasicMaterial({
      color: 0xff0000,
      opacity: 0.5,
      transparent: true,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    const edge = new THREE.Mesh(edgeGeometry, edgeMaterial);
    edge.rotation.x = -Math.PI / 2;
    edge.position.y = 0.025;
    this.scene.add(edge);
  }

  createGlobalBackground() {
    // Textura de gradiente para el fondo
    const canvas = document.createElement('canvas');
    canvas.width = 512;
    canvas.height = 512;
    const ctx = canvas.getContext('2d');
    if (ctx) {
      // Gradiente radial: centro claro, bordes azulados
      const gradient = ctx.createRadialGradient(
        256, 256, 50, // centro
        256, 256, 256 // radio
      );
      gradient.addColorStop(0, '#e0e7ef');
      gradient.addColorStop(0.7, '#b3c6e7');
      gradient.addColorStop(1, '#7fa1d6');
      ctx.fillStyle = gradient;
      ctx.fillRect(0, 0, 512, 512);
    }
    const texture = new THREE.CanvasTexture(canvas);
    this.scene.background = texture;

    // --- Agregar paredes alrededor del piso con textura ---
    const floorSize = 20; // Aumenta el tamaño del piso
    const wallHeight = 6; // Aumenta la altura de las paredes
    const wallThickness = 0.4; // Aumenta el grosor de las paredes

    // Cargar textura de pared
    const wallTexture = new THREE.TextureLoader().load('images/wall.jpg');
    wallTexture.wrapS = wallTexture.wrapT = THREE.RepeatWrapping;
    wallTexture.repeat.set(floorSize / 4, wallHeight / 2);

    const wallMaterial = new THREE.MeshStandardMaterial({
      map: wallTexture,
      color: 0xffffff,
      roughness: 0.8,
      metalness: 0.05,
      transparent: true,
      opacity: 0.97
    });

    // Pared trasera (eje Z+)
    const backWall = new THREE.Mesh(
      new THREE.BoxGeometry(floorSize, wallHeight, wallThickness),
      wallMaterial
    );
    backWall.position.set(0, wallHeight / 2, floorSize / 2);
    backWall.receiveShadow = true;
    this.scene.add(backWall);

    // Pared delantera (eje Z-)
    const frontWall = new THREE.Mesh(
      new THREE.BoxGeometry(floorSize, wallHeight, wallThickness),
      wallMaterial
    );
    frontWall.position.set(0, wallHeight / 2, -floorSize / 2);
    frontWall.receiveShadow = true;
    this.scene.add(frontWall);

    // Pared izquierda (eje X-)
    const leftWall = new THREE.Mesh(
      new THREE.BoxGeometry(wallThickness, wallHeight, floorSize),
      wallMaterial
    );
    leftWall.position.set(-floorSize / 2, wallHeight / 2, 0);
    leftWall.receiveShadow = true;
    this.scene.add(leftWall);

    // Pared derecha (eje X+)
    const rightWall = new THREE.Mesh(
      new THREE.BoxGeometry(wallThickness, wallHeight, floorSize),
      wallMaterial
    );
    rightWall.position.set(floorSize / 2, wallHeight / 2, 0);
    rightWall.receiveShadow = true;
    this.scene.add(rightWall);
  }

  resetCameraPosition() {
    if (!this.camera) return;

    if (this.isSmallScreen) {
      this.camera.position.set(0, 5.5, -5.5);
    } else {
      this.camera.position.set(0, 3, -4);
    }

    this.controls.target.set(0, 1.0, 0);
    this.controls.update();
  }

  getEndEffectorPose(robotModel: any, robot: any) {
    if (!robotModel) return;

    const endEffector = robotModel.links["ee_link"];
    if (!endEffector) return;

    const position = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();

    // Obtener posición y orientación mundial
    endEffector.getWorldPosition(position);
    endEffector.getWorldQuaternion(quaternion);

    // Ajuste de inclinación del modelo URDF (-π/2 en X)
    const modelInclination = new THREE.Quaternion().setFromEuler(new THREE.Euler(Math.PI / 2, 0, 0));
    quaternion.premultiply(modelInclination);

    // Ajuste para coincidir con convención ROS (X->X, Y->Z, Z->-Y)
    const rosPosition = new THREE.Vector3(
      position.x / 3,       // X
      -position.z / 3,       // Y
      position.y / 3        // Z
    );

    const rosQuaternion = new THREE.Quaternion(
      quaternion.x,   // X
      quaternion.y,   // Y
      quaternion.z,   // Z
      quaternion.w    // W
    );

    // Guardar en el robot state (opcional, para UI)
    robot.x.set(parseFloat(rosPosition.x.toFixed(3)));
    robot.y.set(parseFloat(rosPosition.y.toFixed(3)));
    robot.z.set(parseFloat(rosPosition.z.toFixed(3)));
    robot.qx.set(rosQuaternion.x);
    robot.qy.set(rosQuaternion.y);
    robot.qz.set(rosQuaternion.z);
    robot.qw.set(rosQuaternion.w);
  }

  stopMotion() {
    this.rosService.executeCommand('stop').subscribe(
      {
        next: (data) => {
          console.log("Movimiento detenido");
        },
        error: (error) => {
          console.error("Detención del robot érronea");
        }
      }
    );
  }

}
