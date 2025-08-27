import { Component, ElementRef, OnInit, ViewChild, AfterViewInit } from '@angular/core';
import { FormsModule } from '@angular/forms';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
import { RosService } from '../services/ros.service';
import { CommonModule } from '@angular/common';

@Component({
  standalone: true,
  selector: 'app-robot-arm',
  imports: [FormsModule, CommonModule],
  templateUrl: './robot-arm.component.html',
  styleUrl: './robot-arm.component.scss'
})
export class RobotArmComponent implements OnInit, AfterViewInit {
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

  angle1: number = 0;
  angle2: number = 0;
  angle3: number = 0;
  angle4: number = 0;
  angle5: number = 0;
  angle6: number = 0;

  robotModel: any;

  constructor(private rosService: RosService) {
    // Detectar si la pantalla es pequeña
    this.isSmallScreen = window.innerWidth < 768;

    // Escuchar cambios de tamaño de ventana
    window.addEventListener('resize', () => {
      this.isSmallScreen = window.innerWidth < 768;
      if (this.isSmallScreen) {
        this.resetCameraPosition();
      }
    });
   }

  sendMessage() {
    this.rosService.publishExample();
  }

  ngOnInit(): void { }

  ngAfterViewInit(): void {
    this.initScene();
    this.animate();
    this.rosService.subscribeToJointStates((message) => this.updateRobotMovement(message));
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
    this.renderer.setSize(width, height);
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

    // Controles de órbita
    const controls = new OrbitControls(this.camera, this.renderer.domElement);
    controls.target.set(0, 1.0, 0);
    controls.update();

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
      this.renderer.setSize(w, h);
    });
  }

  updateRobotMovement(message: any) {
    const jointNames = message.name;
    const jointPositions = message.position;

    for (let i = 0; i < jointNames.length; i++) {
      const jointName = jointNames[i];
      const position = jointPositions[i];

      const joint = this.robotModel.joints[jointName];
      if (joint) {
        joint.setJointValue(position);
      }
    }
    // Actualizar ángulos de las juntas y redondear a enteros
    this.angle1 = Math.round(jointPositions[0] * 180 / Math.PI);
    this.angle2 = Math.round(jointPositions[1] * 180 / Math.PI);
    this.angle3 = Math.round(jointPositions[2] * 180 / Math.PI);
    this.angle4 = Math.round(jointPositions[3] * 180 / Math.PI);
    this.angle5 = Math.round(jointPositions[4] * 180 / Math.PI);
    this.angle6 = Math.round(jointPositions[5] * 180 / Math.PI);

  }


  animate() {
    requestAnimationFrame(() => this.animate());
    this.renderer.render(this.scene, this.camera);
  }

  moveJoint(joint: string, angle: number) {
    const jointMap: { [key: string]: keyof RobotArmComponent } = {
      'junta_0_1': 'angle1',
      'junta_1_2': 'angle2',
      'junta_2_3': 'angle3',
      'junta_3_4': 'angle4',
      'junta_4_5': 'angle5',
      'junta_5_6': 'angle6'
    };

    if (jointMap[joint]) {
      (this as any)[jointMap[joint]] = angle;
    }

    const angles = [
      this.angle1, this.angle2, this.angle3,
      this.angle4, this.angle5, this.angle6
    ].map(deg => deg * Math.PI / 180);

    this.rosService.publishJointState(
      Object.keys(jointMap),
      angles
    );
  }

  rotate6Joint(angle: number) {
    const num = angle / 360;
    this.rosService.publishJoint6Command(num);
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
        this.robotModel = loader.parse(value);
        this.robotModel.rotation.x = -Math.PI / 2;
        setTimeout(() => {  
          // Mejor manejo de sombras para todos los meshes
          this.robotModel.traverse((child: THREE.Object3D) => {
            if ((child as THREE.Mesh).isMesh) {
              (child as THREE.Mesh).castShadow = true;
              (child as THREE.Mesh).receiveShadow = true;
            }
          });
        }, 200);

        this.robotModel.scale.set(3, 3, 3);
        this.scene.add(this.robotModel);
      } catch (e) {
        console.error('Error al cargar el modelo URDF:', e);
      }
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
    }
    else {
      this.camera.position.set(0, 3, -4);
    }
    this.camera.lookAt(0, 1.0, 0);
  }
}
