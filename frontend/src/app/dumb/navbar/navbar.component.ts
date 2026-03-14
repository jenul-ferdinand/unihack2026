import { Component, Input, Output, EventEmitter } from '@angular/core';

export type NavTab = 'runs' | 'info' | 'pairing';

@Component({
  selector: 'app-navbar',
  standalone: true,
  templateUrl: './navbar.component.html',
  styleUrl: './navbar.component.scss',
})
export class NavbarComponent {
  @Input() activeTab: NavTab = 'pairing';
  @Output() tabChanged = new EventEmitter<NavTab>();

  selectTab(tab: NavTab): void {
    this.tabChanged.emit(tab);
  }
}
