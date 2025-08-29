import { Injectable } from '@angular/core';
import { BehaviorSubject } from 'rxjs';

@Injectable({ providedIn: 'root' })
export class UiStateService {
  private smallScreenSubject = new BehaviorSubject<boolean>(window.innerWidth < 768);
  isSmallScreen$ = this.smallScreenSubject.asObservable();

  private menuOpenSubject = new BehaviorSubject<boolean>(false);
  menuOpen$ = this.menuOpenSubject.asObservable();

  private manualMode = new BehaviorSubject<boolean>(false);
  manualMode$ = this.manualMode.asObservable();

  setMenuOpen(value: boolean) {
    this.menuOpenSubject.next(value);
  }

  setSmallScreen(value: boolean) {
    this.smallScreenSubject.next(value);
  }

  setManualMode(value: boolean) {
    this.manualMode.next(value);
  }

  private resetCameraSubject = new BehaviorSubject<void>(undefined);
  resetCamera$ = this.resetCameraSubject.asObservable();

  triggerResetCamera() {
    this.resetCameraSubject.next();
  }
}