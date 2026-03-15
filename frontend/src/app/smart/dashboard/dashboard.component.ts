import { Component, OnInit, ViewChild } from '@angular/core';
import { NavbarComponent, NavTab } from '../../dumb/navbar/navbar.component';
import { InfoPanelComponent } from './info-panel/info-panel.component';
import { InfoDetailPanelComponent } from '../../dumb/info-detail-panel/info-detail-panel.component';
import { MapPanelComponent } from './map-panel/map-panel.component';
import { PairingComponent } from '../pairing/pairing.component';
import { HelpPageComponent } from '../../dumb/help-page/help-page.component';
import { RunsService, RunSummary, RunDetailResponse } from '../../services/runs.service';

@Component({
  selector: 'app-dashboard',
  standalone: true,
  imports: [NavbarComponent, InfoPanelComponent, InfoDetailPanelComponent, MapPanelComponent, PairingComponent, HelpPageComponent],
  templateUrl: './dashboard.component.html',
  styleUrl: './dashboard.component.scss',
})
export class DashboardComponent implements OnInit {
  @ViewChild(MapPanelComponent) mapPanel!: MapPanelComponent;

  activeTab: NavTab = 'pairing';
  runs: RunSummary[] = [];
  selectedRunId: string | null = null;
  selectedRun: RunSummary | null = null;
  selectedRunDetail: RunDetailResponse | null = null;
  loading = false;
  demoLoading = false;

  constructor(private runsService: RunsService) {}

  ngOnInit(): void {
    this.loadRuns();
  }

  onTabChanged(tab: NavTab): void {
    this.activeTab = tab;
  }

  loadRuns(): void {
    this.runsService.getRuns().subscribe({
      next: (res) => (this.runs = res.runs),
      error: (err) => console.error('Failed to load runs:', err),
    });
  }

  onRunSelected(runId: string): void {
    this.selectedRunId = runId;
    this.selectedRun = this.runs.find((r) => r.run_id === runId) ?? null;
    this.loading = true;
    this.mapPanel.clearPaths();

    this.runsService.getRunDetail(runId).subscribe({
      next: (detail) => {
        if (this.selectedRunId !== runId) return;
        this.selectedRunDetail = detail;
        for (const point of detail.path) {
          this.mapPanel.addPoints(point.device_pos, point.peer_pos);
        }
        this.loading = false;
      },
      error: (err) => {
        if (this.selectedRunId !== runId) return;
        console.error('Failed to load run detail:', err);
        this.loading = false;
      },
    });
  }

  onDeleteRun(runId: string): void {
    this.selectedRunId = null;
    this.selectedRun = null;
    this.selectedRunDetail = null;
    this.activeTab = 'runs';
    this.mapPanel?.clearPaths();

    this.runsService.deleteRun(runId).subscribe({
      next: () => this.loadRuns(),
      error: (err) => console.error('Failed to delete run:', err),
    });
  }

  onDemoRequested(): void {
    this.demoLoading = true;
    this.runsService.startDemo().subscribe({
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
