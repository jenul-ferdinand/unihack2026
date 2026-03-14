import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';

@Injectable({ providedIn: 'root' })
export class CommsService {
  private commsBase = '/api/comms';

  constructor(private http: HttpClient) {}
}
