import { Component, ElementRef, OnInit, ViewChild, AfterViewInit } from '@angular/core';
import { FormsModule } from '@angular/forms';
import * as THREE from 'three';
import { OrbitControls, ThreeMFLoader } from 'three/examples/jsm/Addons.js';
import GUI from 'three/examples/jsm/libs/lil-gui.module.min.js';

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

  ngOnInit(): void { }

  ngAfterViewInit(): void {
    this.initScene();
    this.animate();
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
    this.camera.position.set(5, 5, 0);

    this.canvasContainer.nativeElement.appendChild(this.renderer.domElement);

    const controls = new OrbitControls(this.camera, this.renderer.domElement)
    controls.enableDamping = true;

    // Floor
    const floorGeometry = new THREE.PlaneGeometry(10, 10);
    const floorMaterial = new THREE.MeshStandardMaterial({ color: 0x888888 });
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
    light.position.set(5, 15, -5);
    light.castShadow = true;
    light.shadow.camera.near = 0.1;
    light.shadow.camera.far = 50;
    light.shadow.mapSize.width = data.shadowMapSizeWidth; // default
    light.shadow.mapSize.height = data.shadowMapSizeHeight; // default
    this.scene.add(light);

    // Base
    this.base = new THREE.Object3D();
    const heightBase = 0.7;
    const baseMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.4, 0.7, heightBase), new THREE.MeshStandardMaterial({ color: "black" }));
    this.base.add(baseMesh);
    this.scene.add(this.base);
    baseMesh.position.y = heightBase / 2;
    baseMesh.castShadow = true;
    baseMesh.receiveShadow = true;

    // Joint 1
    this.joint1 = new THREE.Object3D();
    const heightArm1 = 0.6;
    const arm1 = new THREE.Mesh(new THREE.CylinderGeometry(0.4, 0.4, heightArm1, 14), new THREE.MeshStandardMaterial({ color: "black" }));
    arm1.position.y = heightArm1 / 2;
    this.joint1.add(arm1);
    this.joint1.position.y = heightBase / 2;
    this.base.add(this.joint1);
    arm1.castShadow = true;
    arm1.receiveShadow = true;
    
    // Joint 2
    this.joint2 = new THREE.Object3D();
    const heightArm2 = 1.1;
    const arm2 = new THREE.Mesh(new THREE.CapsuleGeometry(0.4, heightArm2, 20), new THREE.MeshStandardMaterial({ color: "gray" }));
    arm2.rotation.z = Math.PI / 2; // Rotación inicial para el segundo brazo
    this.joint2.add(arm2);
    this.joint2.position.y = heightArm1;
    this.joint1.add(this.joint2);
    arm2.castShadow = true;
    arm2.receiveShadow = true;

    const joint1extra = new THREE.Object3D();
    const heightArm1extra = 1.3;
    const arm1extra = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.4, heightArm1extra, 14), new THREE.MeshStandardMaterial({ color: "white" }));
    arm1extra.position.y = heightArm1extra / 2;
    joint1extra.add(arm1extra);
    this.joint2.add(joint1extra);
    arm1extra.castShadow = true;
    arm1extra.receiveShadow = true;

    // Joint 3
    this.joint3 = new THREE.Object3D();
    const heightArm3 = 0.4;
    const arm3 = new THREE.Mesh(new THREE.CapsuleGeometry(0.3, heightArm3, 14), new THREE.MeshStandardMaterial({ color: "white" }));
    arm3.rotation.z = Math.PI / 2; // Rotación inicial para el tercer brazo
    this.joint3.add(arm3);
    this.joint3.position.y = heightArm1extra;
    joint1extra.add(this.joint3);
    arm3.castShadow = true;
    arm3.receiveShadow = true;

    // Joint 4
    this.joint4 = new THREE.Object3D();
    const heightArm4 = 1.3;
    const arm4 = new THREE.Mesh(new THREE.CapsuleGeometry(0.25, heightArm4, 14), new THREE.MeshStandardMaterial({ color: "white" }));
    arm4.position.y = heightArm4 / 2;
    this.joint4.add(arm4);
    this.joint3.add(this.joint4);
    arm4.castShadow = true;
    arm4.receiveShadow = true;

    // Joint 5
    this.joint5 = new THREE.Object3D();
    const heightArm5 = 0.4;
    const arm5 = new THREE.Mesh(new THREE.CapsuleGeometry(0.2, heightArm5, 14), new THREE.MeshStandardMaterial({ color: "gray" }));
    arm5.rotation.z = Math.PI / 2; // Rotación inicial para el cuarto brazo
    this.joint5.add(arm5);
    this.joint5.position.y = heightArm4 + 0.15;
    this.joint4.add(this.joint5);
    arm5.castShadow = true;
    arm5.receiveShadow = true;

    // Joint 6
    this.joint6 = new THREE.Object3D();
    const heightArm6 = 0.3;
    const arm6 = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, heightArm6, 14), new THREE.MeshStandardMaterial({ color: "gray" }));
    arm6.position.y = heightArm6 / 2;
    this.joint6.add(arm6);
    this.joint5.add(this.joint6);
    arm6.castShadow = true;
    arm6.receiveShadow = true;

    // Arrows for Base
    const arrowX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 1.5, 0x00ff00, 0.3, 0.3); // Green for X
    const arrowY = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 1.5, 0x0000ff, 0.3, 0.3); // Blue for Y
    const arrowZ = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 1.5, 0xff0000, 0.3, 0.3); // Red for Z
    this.base.add(arrowX);
    this.base.add(arrowY);
    this.base.add(arrowZ);

    // Arrows for joint6
    const arrowX6 = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 1, 0x00ff00, 0.3, 0.2); // Green for X
    const arrowY6 = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 1, 0xff0000, 0.3, 0.2); // Red for Y
    const arrowZ6 = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 0, 0), 1, 0x0000ff, 0.3, 0.2); // Blue for Z
    this.joint6.add(arrowX6);
    this.joint6.add(arrowY6);
    this.joint6.add(arrowZ6);
  }


    animate() {
      requestAnimationFrame(() => this.animate());

      // Animaciones (puedes luego conectar esto con controles)
      this.joint1.rotation.y = THREE.MathUtils.degToRad(this.angle1);
      this.joint2.rotation.x = THREE.MathUtils.degToRad(this.angle2);
      this.joint3.rotation.x = THREE.MathUtils.degToRad(this.angle3);
      this.joint4.rotation.y = THREE.MathUtils.degToRad(this.angle4);
      this.joint5.rotation.x = THREE.MathUtils.degToRad(this.angle5);
      this.joint6.rotation.y = THREE.MathUtils.degToRad(this.angle6);

      this.renderer.render(this.scene, this.camera);
    }
  }
