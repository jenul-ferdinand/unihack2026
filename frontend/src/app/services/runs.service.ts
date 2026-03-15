import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import type { RunSummary, RunsListResponse, RunDetailResponse } from '@unihack/types';

export type { RunSummary, RunsListResponse, RunDetailResponse };

@Injectable({ providedIn: 'root' })
export class RunsService {
  private base = '/api/runs';

  constructor(private http: HttpClient) {}

  getRuns(): Observable<RunsListResponse> {
    return this.http.get<RunsListResponse>(this.base);
  }

  getRunDetail(id: string): Observable<RunDetailResponse> {
    return this.http.get<RunDetailResponse>(`${this.base}/${id}`);
  }

  startDemo(): Observable<{ run_id: string }> {
    return this.http.post<{ run_id: string }>(`${this.base}/demo`, {});
  }

  deleteRun(id: string): Observable<{ success: boolean }> {
    return this.http.delete<{ success: boolean }>(`${this.base}/${id}`);
  }
}
