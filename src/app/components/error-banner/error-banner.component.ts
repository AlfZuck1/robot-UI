import { Component, OnDestroy } from '@angular/core';
import { ErrorService } from '../../services/error.service';
import { CommonModule } from '@angular/common';
import { Observable, Subscription, timer } from 'rxjs';

@Component({
  selector: 'app-error-banner',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './error-banner.component.html',
  styleUrls: ['./error-banner.component.scss']
})
export class ErrorBannerComponent implements OnDestroy {

  error$: Observable<AppError | null>;
  private timeoutSub?: Subscription;
  private readonly AUTO_CLOSE_TIME = 2000;

  constructor(private errorService: ErrorService) {
    this.error$ = this.errorService.error$;

    this.error$.subscribe(error => {
      if (error) {
        this.startTimeout();
      }
    });
  }

  private startTimeout() {
    this.timeoutSub?.unsubscribe();

    this.timeoutSub = timer(this.AUTO_CLOSE_TIME).subscribe(() => {
      this.close();
    });
  }

  close() {
    this.timeoutSub?.unsubscribe();
    this.errorService.clear();
  }

  ngOnDestroy() {
    this.timeoutSub?.unsubscribe();
  }
}

export interface AppError {
  title: string;
  message: string;
}
