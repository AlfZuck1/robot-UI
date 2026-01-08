import { Injectable } from '@angular/core';
import { BehaviorSubject } from 'rxjs';


export interface AppError {
  status?: number;
  title: string;
  message: string;
}


@Injectable({ providedIn: 'root' })
export class ErrorService {


  private errorSubject = new BehaviorSubject<AppError | null>(null);
  error$ = this.errorSubject.asObservable();


  showError(error: AppError) {
    this.errorSubject.next(error);
  }


  clear() {
    this.errorSubject.next(null);
  }
}