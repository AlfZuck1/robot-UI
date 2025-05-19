import { Component, ElementRef, OnInit, ViewChild, AfterViewInit } from '@angular/core';
import { FormsModule } from '@angular/forms';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
import { RosService } from '../services/ros.service';

@Component({
  selector: 'app-robot-arm',
  imports: [FormsModule],
  templateUrl: './robot-arm.component.html',
  styleUrl: './robot-arm.component.scss'
})
export class RobotArmComponent implements OnInit, AfterViewInit {
  @ViewChild('canvasContainer', { static: true }) canvasContainer!: ElementRef;

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
  angle3: number = 90;
  angle4: number = 0;
  angle5: number = 0;
  angle6: number = 0;

  robotModel: any;

  constructor(private rosService: RosService) { }

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
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf0f0f0);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.shadowMap.enabled = true;
    const width = this.canvasContainer.nativeElement.clientWidth;
    const height = this.canvasContainer.nativeElement.clientHeight;
    this.renderer.setSize(width, height);
    this.camera = new THREE.PerspectiveCamera(75, width / height, 1, 100);
    this.camera.position.set(0, 4, -4);
    this.canvasContainer.nativeElement.appendChild(this.renderer.domElement);
    const controls = new OrbitControls(this.camera, this.renderer.domElement)

    // Floor
    const floorGeometry = new THREE.PlaneGeometry(10, 10);
    const floorMaterial = new THREE.MeshStandardMaterial({ color: 0x888888, side: THREE.DoubleSide });
    const floor = new THREE.Mesh(floorGeometry, floorMaterial);
    floor.receiveShadow = true;
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = 0;
    this.scene.add(floor);

    const data = {
      color: 0x00ff00,
      lightColor: 0xffffff,
      shadowMapSizeWidth: 2048,
      shadowMapSizeHeight: 2048,
    }

    // Light
    const light = new THREE.DirectionalLight(data.lightColor, 2.5);
    light.position.set(2, 5, -5);
    light.castShadow = true;
    light.shadow.camera.near = 0.1;
    light.shadow.camera.far = 20;
    light.shadow.mapSize.width = data.shadowMapSizeWidth; // default
    light.shadow.mapSize.height = data.shadowMapSizeHeight; // default
    this.scene.add(light);

    // Cargar URDF desde el parámetro de ROS
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    const param = this.rosService.getUrdf();
    param.get((value) => {
      if (!value) {
        console.error('No se pudo obtener el URDF desde el parámetro de ROS.');
        return;
      }
      console.log('URDF recibido', value);
      // Cargar el modelo URDF
      this.robotModel = loader.parse(value);
      setTimeout(() => {
        this.robotModel.traverse((child: THREE.Object3D) => {
          if (child instanceof THREE.Mesh) {
            child.castShadow = true;
            child.receiveShadow = true;

          }
        });
      }, 200); // Espera 200 milisegundos a que los STL carguen
      this.robotModel.scale.set(3, 3, 3);

      this.scene.add(this.robotModel);
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
  }


  animate() {
    requestAnimationFrame(() => this.animate());
    this.renderer.render(this.scene, this.camera);
  }

  moveJoint(joint: string, angle: number) {
    switch (joint) {
      case 'junta_0_1':
        this.angle1 = angle;
        this.rosService.publishJointState(['junta_0_1'], [angle / 180 * Math.PI]); // Convertir a radianes
        break;
      case 'junta_1_2':
        this.angle2 = angle;
        this.rosService.publishJointState(['junta_1_2'], [angle / 180 * Math.PI]);
        break;
      case 'junta_2_3':
        this.angle3 = angle;
        this.rosService.publishJointState(['junta_2_3'], [angle / 180 * Math.PI]);
        break;
      case 'junta_3_4':
        this.angle4 = angle;
        this.rosService.publishJointState(['junta_3_4'], [angle / 180 * Math.PI]);
        break;
      case 'junta_4_5':
        this.angle5 = angle;
        this.rosService.publishJointState(['junta_4_5'], [angle / 180 * Math.PI]);
        break;
      case 'junta_5_6':
        this.angle6 = angle;
        this.rosService.publishJointState(['junta_5_6'], [angle / 180 * Math.PI]);
        break;
    }
  }
}
