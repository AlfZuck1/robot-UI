import { Injectable, signal } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class RobotStateService {
  angle1 = signal(0);
  angle2 = signal(0);
  angle3 = signal(0);
  angle4 = signal(0);
  angle5 = signal(0);
  angle6 = signal(0);
  time = signal(0);
}
