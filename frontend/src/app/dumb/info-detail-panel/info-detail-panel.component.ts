import { Component, Input, Output, EventEmitter } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RunSummary } from '../../services/comms.service';

@Component({
  selector: 'app-info-detail-panel',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './info-detail-panel.component.html',
  styleUrl: './info-detail-panel.component.scss',
})
export class InfoDetailPanelComponent {
  @Input() run: RunSummary | null = null;
  @Output() deleteRequested = new EventEmitter<string>();

  confirmingDelete = false;

  formatDate(iso: string): string {
    return new Date(iso).toLocaleString();
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
}
