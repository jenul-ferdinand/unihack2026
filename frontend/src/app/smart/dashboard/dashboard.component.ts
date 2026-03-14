import { Component, OnInit, ViewChild } from '@angular/core';
import { NavbarComponent, NavTab } from '../../dumb/navbar/navbar.component';
import { InfoPanelComponent } from './info-panel/info-panel.component';
import { InfoDetailPanelComponent } from '../../dumb/info-detail-panel/info-detail-panel.component';
import { MapPanelComponent } from './map-panel/map-panel.component';
import { PairingComponent } from '../pairing/pairing.component';
import { CommsService, RunSummary } from '../../services/comms.service';

@Component({
  selector: 'app-dashboard',
  standalone: true,
  imports: [NavbarComponent, InfoPanelComponent, InfoDetailPanelComponent, MapPanelComponent, PairingComponent],
  templateUrl: './dashboard.component.html',
  styleUrl: './dashboard.component.scss',
})
export class DashboardComponent implements OnInit {
  @ViewChild(MapPanelComponent) mapPanel!: MapPanelComponent;

  activeTab: NavTab = 'pairing';
  runs: RunSummary[] = [];
  selectedRunId: string | null = null;
  loading = false;
  demoLoading = false;

  constructor(private commsService: CommsService) {}

  ngOnInit(): void {
    this.loadRuns();
  }

  onTabChanged(tab: NavTab): void {
    this.activeTab = tab;
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

  onDemoRequested(): void {
    this.demoLoading = true;
    this.commsService.startDemo().subscribe({
      next: (res) => {
        this.demoLoading = false;
        this.loadRuns();
        this.onRunSelected(res.run_id);
      },
      error: (err) => {
        console.error('Failed to start demo:', err);
        this.demoLoading = false;
      },
    });
  }
}
