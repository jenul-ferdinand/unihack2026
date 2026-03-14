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
    this.devicePath.push(new THREE.Vector3(device.x, device.y, device.z));
    this.peerPath.push(new THREE.Vector3(peer.x, peer.y, peer.z));
    this.updateLine(this.deviceLine, this.devicePath);
    this.updateLine(this.peerLine, this.peerPath);
  }

  clearPaths(): void {
    this.devicePath = [];
    this.peerPath = [];
    this.updateLine(this.deviceLine, this.devicePath);
    this.updateLine(this.peerLine, this.peerPath);
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
    this.camera.position.set(30, 20, 30);

    // Controls
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;

    // Globe
    const globeGeo = new THREE.SphereGeometry(15, 32, 32);
    const globeMat = new THREE.MeshBasicMaterial({
      color: this.cssColor('--globe-wire'),
      transparent: true,
      opacity: 0.08,
      wireframe: true,
    });
    this.scene.add(new THREE.Mesh(globeGeo, globeMat));

    // Path lines
    this.deviceLine = this.createLine(this.cssColor('--accent-device'));
    this.peerLine = this.createLine(this.cssColor('--accent-peer'));
    this.scene.add(this.deviceLine);
    this.scene.add(this.peerLine);

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
