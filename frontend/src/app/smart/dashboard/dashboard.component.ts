import { Component } from '@angular/core';
import { NavbarComponent } from '../../dumb/navbar/navbar.component';
import { InfoPanelComponent } from './info-panel/info-panel.component';
import { MapPanelComponent } from './map-panel/map-panel.component';

@Component({
  selector: 'app-dashboard',
  standalone: true,
  imports: [NavbarComponent, InfoPanelComponent, MapPanelComponent],
  templateUrl: './dashboard.component.html',
  styleUrl: './dashboard.component.scss',
})
export class DashboardComponent {}
