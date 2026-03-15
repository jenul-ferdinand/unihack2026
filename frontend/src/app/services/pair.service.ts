import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import type { PairStatusResponse } from '@unihack/types';

export type { PairStatusResponse };

@Injectable({ providedIn: 'root' })
export class PairService {
  private base = '/api/pair';

  constructor(private http: HttpClient) {}

  getStatus(): Observable<PairStatusResponse> {
    return this.http.get<PairStatusResponse>(`${this.base}/status`);
  }
}
