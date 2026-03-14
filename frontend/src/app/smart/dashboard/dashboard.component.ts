import { Component, OnInit, ViewChild } from '@angular/core';
import { NavbarComponent } from '../../dumb/navbar/navbar.component';
import { InfoPanelComponent } from './info-panel/info-panel.component';
import { MapPanelComponent } from './map-panel/map-panel.component';
import { CommsService, RunSummary } from '../../services/comms.service';

@Component({
  selector: 'app-dashboard',
  standalone: true,
  imports: [NavbarComponent, InfoPanelComponent, MapPanelComponent],
  templateUrl: './dashboard.component.html',
  styleUrl: './dashboard.component.scss',
})
export class DashboardComponent implements OnInit {
  @ViewChild(MapPanelComponent) mapPanel!: MapPanelComponent;

  runs: RunSummary[] = [];
  selectedRunId: string | null = null;
  loading = false;

  constructor(private commsService: CommsService) {}

  ngOnInit(): void {
    this.loadRuns();
  }

  loadRuns(): void {
    this.commsService.getRuns().subscribe({
      next: (res) => (this.runs = res.runs),
      error: (err) => console.error('Failed to load runs:', err),
    });
  }

  onRunSelected(runId: string): void {
    this.selectedRunId = runId;
    this.loading = true;
    this.mapPanel.clearPaths();

    this.commsService.getRunDetail(runId).subscribe({
      next: (detail) => {
        for (const point of detail.path) {
          this.mapPanel.addPoints(point.device_pos, point.peer_pos);
        }
        this.loading = false;
      },
      error: (err) => {
        console.error('Failed to load run detail:', err);
        this.loading = false;
      },
    });
  }
}
