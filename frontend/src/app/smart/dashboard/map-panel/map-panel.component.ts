import {
  Component,
  ElementRef,
  ViewChild,
  AfterViewInit,
  OnDestroy,
  NgZone,
} from '@angular/core';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

@Component({
  selector: 'app-map-panel',
  standalone: true,
  templateUrl: './map-panel.component.html',
  styleUrl: './map-panel.component.scss',
})
export class MapPanelComponent implements AfterViewInit, OnDestroy {
  @ViewChild('canvas', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;

  private renderer!: THREE.WebGLRenderer;
  private scene!: THREE.Scene;
  private camera!: THREE.PerspectiveCamera;
  private controls!: OrbitControls;
  private animationId = 0;
  private resizeObserver!: ResizeObserver;

  private devicePath: THREE.Vector3[] = [];
  private peerPath: THREE.Vector3[] = [];
  private deviceLine!: THREE.Line;
  private peerLine!: THREE.Line;
  private deviceStartDot!: THREE.Mesh;
  private deviceEndDot!: THREE.Mesh;
  private peerStartDot!: THREE.Mesh;
  private peerEndDot!: THREE.Mesh;
  private readonly GLOBE_RADIUS = 80;
  private readonly DOT_RADIUS = 1.2;

  constructor(private ngZone: NgZone) {}

  ngAfterViewInit(): void {
    this.initScene();
    this.ngZone.runOutsideAngular(() => this.animate());
  }

  ngOnDestroy(): void {
    cancelAnimationFrame(this.animationId);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.renderer.dispose();
  }

  /** Call this to append new positions from incoming comms data */
  addPoints(device: { x: number; y: number; z: number }, peer: { x: number; y: number; z: number }): void {
    const r = this.GLOBE_RADIUS;
    this.devicePath.push(new THREE.Vector3(device.x, device.z + r, device.y));
    this.peerPath.push(new THREE.Vector3(peer.x, peer.z + r, peer.y));
    this.updateLine(this.deviceLine, this.devicePath);
    this.updateLine(this.peerLine, this.peerPath);
    this.updateDots(this.deviceStartDot, this.deviceEndDot, this.devicePath);
    this.updateDots(this.peerStartDot, this.peerEndDot, this.peerPath);
  }

  clearPaths(): void {
    this.devicePath = [];
    this.peerPath = [];
    this.updateLine(this.deviceLine, this.devicePath);
    this.updateLine(this.peerLine, this.peerPath);
    this.updateDots(this.deviceStartDot, this.deviceEndDot, this.devicePath);
    this.updateDots(this.peerStartDot, this.peerEndDot, this.peerPath);
  }

  private cssColor(prop: string): number {
    const hex = getComputedStyle(document.documentElement).getPropertyValue(prop).trim();
    return new THREE.Color(hex).getHex();
  }

  private initScene(): void {
    const canvas = this.canvasRef.nativeElement;
    const container = canvas.parentElement!;

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(this.cssColor('--bg-base'));

    // Scene
    this.scene = new THREE.Scene();

    // Camera
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.1, 2000);
    this.camera.position.set(160, 100, 160);

    // Controls
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;

    // Globe
    const globeGeo = new THREE.SphereGeometry(this.GLOBE_RADIUS, 32, 32);
    const globeMat = new THREE.MeshBasicMaterial({
      color: this.cssColor('--globe-wire'),
      transparent: true,
      opacity: 0.08,
      wireframe: true,
    });
    this.scene.add(new THREE.Mesh(globeGeo, globeMat));

    // Path lines
    const deviceColor = this.cssColor('--accent-device');
    const peerColor = this.cssColor('--accent-peer');
    this.deviceLine = this.createLine(deviceColor);
    this.peerLine = this.createLine(peerColor);
    this.scene.add(this.deviceLine);
    this.scene.add(this.peerLine);

    // Endpoint dots
    this.deviceStartDot = this.createDot(deviceColor);
    this.deviceEndDot = this.createDot(deviceColor);
    this.peerStartDot = this.createDot(peerColor);
    this.peerEndDot = this.createDot(peerColor);
    this.scene.add(this.deviceStartDot, this.deviceEndDot, this.peerStartDot, this.peerEndDot);

    // Resize handling
    this.resizeObserver = new ResizeObserver(() => this.onResize(container));
    this.resizeObserver.observe(container);
    this.onResize(container);
  }

  private createLine(color: number): THREE.Line {
    const geo = new THREE.BufferGeometry();
    const mat = new THREE.LineBasicMaterial({ color });
    return new THREE.Line(geo, mat);
  }

  private createDot(color: number): THREE.Mesh {
    const geo = new THREE.SphereGeometry(this.DOT_RADIUS, 16, 16);
    const mat = new THREE.MeshBasicMaterial({ color });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.visible = false;
    return mesh;
  }

  private updateDots(startDot: THREE.Mesh, endDot: THREE.Mesh, points: THREE.Vector3[]): void {
    if (points.length === 0) {
      startDot.visible = false;
      endDot.visible = false;
      return;
    }
    startDot.position.copy(points[0]);
    startDot.visible = true;
    endDot.position.copy(points[points.length - 1]);
    endDot.visible = true;
  }

  private updateLine(line: THREE.Line, points: THREE.Vector3[]): void {
    line.geometry.dispose();
    line.geometry = new THREE.BufferGeometry().setFromPoints(points);
  }

  private onResize(container: HTMLElement): void {
    const { clientWidth: w, clientHeight: h } = container;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  private animate = (): void => {
    this.animationId = requestAnimationFrame(this.animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };
}
