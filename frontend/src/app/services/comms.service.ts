import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import { environment } from '../../environments/environment';

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export interface PathPoint {
  device_pos: Vec3;
  peer_pos: Vec3;
  timestamp: string;
}

export interface RunSummary {
  run_id: string;
  status: 'active' | 'completed';
  created_at: string;
  stopped_at: string | null;
  point_count: number;
}

export interface RunsListResponse {
  runs: RunSummary[];
}

export interface RunDetailResponse {
  run_id: string;
  status: 'active' | 'completed';
  created_at: string;
  stopped_at: string | null;
  path: PathPoint[];
}

@Injectable({ providedIn: 'root' })
export class CommsService {
  private runsBase = `${environment.apiBase}/api/runs`;

  constructor(private http: HttpClient) {}

  getRuns(): Observable<RunsListResponse> {
    return this.http.get<RunsListResponse>(this.runsBase);
  }

  getRunDetail(id: string): Observable<RunDetailResponse> {
    return this.http.get<RunDetailResponse>(`${this.runsBase}/${id}`);
  }

  startDemo(): Observable<{ run_id: string }> {
    return this.http.post<{ run_id: string }>(`${this.runsBase}/demo`, {});
  }
}
