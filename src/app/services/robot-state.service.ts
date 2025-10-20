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
  x = signal(0);
  y = signal(0);
  z = signal(0);
  qx = signal(0);
  qy = signal(0);
  qz = signal(0);
  qw = signal(0);
}

@Injectable({
  providedIn: 'root'
})
export class RobotStateServiceSimulated extends RobotStateService {
  // Es un servicio que simula el estado del robot
}
