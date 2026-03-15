import { Component, Input, Output, EventEmitter } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RunSummary, RunDetailResponse } from '../../services/runs.service';
import type { Vector3 } from '@unihack/types';

export interface RunStats {
  durationSec: number;
  deviceDistance: number;
  peerDistance: number;
  maxDepth: number;
  avgSeparation: number;
  maxSeparation: number;
  avgSpeed: number;
}

@Component({
  selector: 'app-info-detail-panel',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './info-detail-panel.component.html',
  styleUrl: './info-detail-panel.component.scss',
})
export class InfoDetailPanelComponent {
  @Input() run: RunSummary | null = null;
  @Input() detail: RunDetailResponse | null = null;
  @Output() deleteRequested = new EventEmitter<string>();

  confirmingDelete = false;

  get stats(): RunStats | null {
    if (!this.detail || !this.run || this.detail.path.length < 2) return null;

    const path = this.detail.path;

    const durationSec = this.run.stopped_at
      ? (new Date(this.run.stopped_at).getTime() - new Date(this.run.created_at).getTime()) / 1000
      : 0;

    let deviceDistance = 0;
    let peerDistance = 0;
    const startZ = path[0].device_pos.z;
    let minZ = startZ;
    let totalSeparation = 0;
    let maxSeparation = 0;

    for (let i = 0; i < path.length; i++) {
      const p = path[i];

      if (i > 0) {
        const prev = path[i - 1];
        deviceDistance += this.dist(prev.device_pos, p.device_pos);
        peerDistance += this.dist(prev.peer_pos, p.peer_pos);
      }

      if (p.device_pos.z < minZ) minZ = p.device_pos.z;

      const sep = this.dist(p.device_pos, p.peer_pos);
      totalSeparation += sep;
      if (sep > maxSeparation) maxSeparation = sep;
    }

    const maxDepth = startZ - minZ;
    const avgSeparation = totalSeparation / path.length;
    const avgSpeed = durationSec > 0 ? deviceDistance / durationSec : 0;

    return { durationSec, deviceDistance, peerDistance, maxDepth, avgSeparation, maxSeparation, avgSpeed };
  }

  formatDate(iso: string): string {
    return new Date(iso).toLocaleString();
  }

  formatDuration(seconds: number): string {
    const m = Math.floor(seconds / 60);
    const s = Math.floor(seconds % 60);
    return m > 0 ? `${m}m ${s}s` : `${s}s`;
  }

  formatDistance(meters: number): string {
    return meters >= 1000 ? `${(meters / 1000).toFixed(2)} km` : `${meters.toFixed(2)} m`;
  }

  formatSpeed(mps: number): string {
    return `${mps.toFixed(2)} m/s`;
  }

  onDelete(): void {
    if (!this.confirmingDelete) {
      this.confirmingDelete = true;
      return;
    }
    if (this.run) {
      this.deleteRequested.emit(this.run.run_id);
    }
    this.confirmingDelete = false;
  }

  cancelDelete(): void {
    this.confirmingDelete = false;
  }

  onDownloadCsv(): void {
    if (!this.detail || !this.run) return;

    const header = 'timestamp,device_x,device_y,device_z,peer_x,peer_y,peer_z';
    const rows = this.detail.path.map((p) =>
      `${p.timestamp},${p.device_pos.x},${p.device_pos.y},${p.device_pos.z},${p.peer_pos.x},${p.peer_pos.y},${p.peer_pos.z}`
    );
    const csv = [header, ...rows].join('\n');

    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `run-${this.run.run_id}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  }

  private dist(a: Vector3, b: Vector3): number {
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const dz = b.z - a.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }
}
