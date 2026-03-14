import { Component } from '@angular/core';

@Component({
  selector: 'app-pairing',
  standalone: true,
  templateUrl: './pairing.component.html',
  styleUrl: './pairing.component.scss',
})
export class PairingComponent {
  deviceConnected = false;
  peerConnected = false;
}
