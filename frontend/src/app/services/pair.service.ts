import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';

export interface PairStatus {
  pending_count: number;
  paired: {
    device_ip: string;
    peer_ip: string;
  } | null;
}

@Injectable({ providedIn: 'root' })
export class PairService {
  private base = '/api/pair';

  constructor(private http: HttpClient) {}

  getStatus(): Observable<PairStatus> {
    return this.http.get<PairStatus>(`${this.base}/status`);
  }
}
