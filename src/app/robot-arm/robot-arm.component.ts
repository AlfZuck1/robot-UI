import { Component, ElementRef, OnInit, ViewChild, AfterViewInit } from '@angular/core';
import { FormsModule } from '@angular/forms';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
import { RosService } from '../services/ros.service';
import ROSLIB from 'roslib';

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
    this.camera.position.set(0, 1.5, -1.2);

    this.canvasContainer.nativeElement.appendChild(this.renderer.domElement);

    const controls = new OrbitControls(this.camera, this.renderer.domElement)
    controls.enableDamping = true;

    // Floor
    const floorGeometry = new THREE.PlaneGeometry(3, 3);
    const floorMaterial = new THREE.MeshStandardMaterial({ color: 0x888888, side: THREE.DoubleSide });
    const floor = new THREE.Mesh(floorGeometry, floorMaterial);
    floor.receiveShadow = true;
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.01;
    this.scene.add(floor);

    const data = {
      color: 0x00ff00,
      lightColor: 0xffffff,
      shadowMapSizeWidth: 1024,
      shadowMapSizeHeight: 1024,
    }

    // Light
    const light = new THREE.DirectionalLight(data.lightColor, 2);
    light.position.set(1, 2, -1);
    light.castShadow = true;
    light.shadow.camera.near = 0.1;
    light.shadow.camera.far = 50;
    light.shadow.mapSize.width = data.shadowMapSizeWidth; // default
    light.shadow.mapSize.height = data.shadowMapSizeHeight; // default
    this.scene.add(light);

    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);

    // Cargar URDF desde el parámetro de ROS
    const param = this.rosService.getUrdf();

    param.get((value) => {
      if (!value) {
        console.error('No se pudo obtener el URDF desde el parámetro de ROS.');
        return;
      }

      console.log('URDF recibido:', value);

      const loader = new URDFLoader();
      this.robotModel = loader.parse(value);

      this.scene.add(this.robotModel);
      // Luego agrega cámara, luz y renderer con Three.js
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
      case 'joint1':
        this.angle1 = angle;
        this.rosService.publishJointState(['joint1'], [angle/180 * Math.PI]); // Convertir a radianes
        break;
      case 'joint2':
        this.angle2 = angle;
        this.rosService.publishJointState(['joint2'], [angle/180 * Math.PI]);
        break;
      case 'joint3':
        this.angle3 = angle;
        this.rosService.publishJointState(['joint3'], [angle/180 * Math.PI]);
        break;
      case 'joint4':
        this.angle4 = angle;
        this.rosService.publishJointState(['joint4'], [angle/180 * Math.PI]);
        break;
      case 'joint5':
        this.angle5 = angle;
        this.rosService.publishJointState(['joint5'], [angle/180 * Math.PI]);
        break;
      case 'joint6':
        this.angle6 = angle;
        this.rosService.publishJointState(['joint6'], [angle/180 * Math.PI]);
        break;
    }
  }
}
