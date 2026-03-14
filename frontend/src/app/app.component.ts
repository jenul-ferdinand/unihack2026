import { Component, HostListener, OnInit } from '@angular/core';
import { RouterOutlet } from '@angular/router';

@Component({
  selector: 'app-root',
  standalone: true,
  imports: [RouterOutlet],
  templateUrl: './app.component.html',
  styleUrl: './app.component.scss'
})
export class AppComponent implements OnInit {
  private static readonly MIN_DESKTOP_WIDTH = 1100;

  isDesktopViewport = true;

  ngOnInit(): void {
    this.updateViewportState();
  }

  @HostListener('window:resize')
  onWindowResize(): void {
    this.updateViewportState();
  }

  private updateViewportState(): void {
    this.isDesktopViewport = window.innerWidth >= AppComponent.MIN_DESKTOP_WIDTH;
  }
}
