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
  paired = false;
  device1Connected = false;
  device2Connected = false;
  device1Ip: string | null = null;
  device2Ip: string | null = null;

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
            this.paired = true;
            this.device1Connected = true;
            this.device2Connected = true;
            this.device1Ip = status.paired.device_ip;
            this.device2Ip = status.paired.peer_ip;
          } else {
            this.paired = false;
            this.device1Connected = status.pending_count > 0;
            this.device2Connected = false;
            this.device1Ip = null;
            this.device2Ip = null;
          }
        },
        error: (err) => console.error('Failed to poll pair status:', err),
      });
  }

  ngOnDestroy(): void {
    this.pollSub?.unsubscribe();
  }
}
