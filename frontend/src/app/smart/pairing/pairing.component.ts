import { Component, OnInit, OnDestroy } from '@angular/core';
import { Subscription, interval, switchMap, startWith } from 'rxjs';
import { PairService } from '../../services/pair.service';

@Component({
  selector: 'app-pairing',
  standalone: true,
  templateUrl: './pairing.component.html',
  styleUrl: './pairing.component.scss',
})
export class PairingComponent implements OnInit, OnDestroy {
  deviceConnected = false;
  peerConnected = false;
  deviceIp: string | null = null;
  peerIp: string | null = null;

  private pollSub?: Subscription;

  constructor(private pairService: PairService) {}

  ngOnInit(): void {
    this.pollSub = interval(2000)
      .pipe(
        startWith(0),
        switchMap(() => this.pairService.getStatus()),
      )
      .subscribe({
        next: (status) => {
          if (status.paired) {
            this.deviceConnected = true;
            this.peerConnected = true;
            this.deviceIp = status.paired.device_ip;
            this.peerIp = status.paired.peer_ip;
          } else {
            this.deviceConnected = status.pending_count > 0;
            this.peerConnected = false;
            this.deviceIp = null;
            this.peerIp = null;
          }
        },
        error: (err) => console.error('Failed to poll pair status:', err),
      });
  }

  ngOnDestroy(): void {
    this.pollSub?.unsubscribe();
  }
}
