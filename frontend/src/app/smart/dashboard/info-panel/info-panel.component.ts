import { Component, Input, Output, EventEmitter } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RunSummary } from '../../../services/comms.service';

@Component({
  selector: 'app-info-panel',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './info-panel.component.html',
  styleUrl: './info-panel.component.scss',
})
export class InfoPanelComponent {
  @Input() runs: RunSummary[] = [];
  @Input() selectedRunId: string | null = null;
  @Input() demoLoading = false;
  @Output() runSelected = new EventEmitter<string>();
  @Output() refreshRequested = new EventEmitter<void>();
  @Output() demoRequested = new EventEmitter<void>();

  selectRun(id: string): void {
    this.runSelected.emit(id);
  }

  refresh(): void {
    this.refreshRequested.emit();
  }

  demo(): void {
    this.demoRequested.emit();
  }

  formatDate(iso: string): string {
    return new Date(iso).toLocaleString();
  }
}
