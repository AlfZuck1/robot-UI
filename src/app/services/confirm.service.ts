import { Injectable, signal } from '@angular/core';

export interface ConfirmOptions {
  title?: string;
  message?: string;
}

@Injectable({ providedIn: 'root' })
export class ConfirmService {
  visible = signal(false);
  title = signal('Confirmar acción');
  message = signal('¿Deseas continuar?');

  private resolver?: () => void;
  private rejecter?: () => void;

  confirm(options: ConfirmOptions = {}): Promise<void> {
    this.title.set(options.title ?? 'Confirmar acción');
    this.message.set(options.message ?? '¿Deseas continuar?');
    this.visible.set(true);

    return new Promise<void>((resolve, reject) => {
      this.resolver = resolve;
      this.rejecter = reject;
    });
  }

  accept() {
    this.visible.set(false);
    this.resolver?.();
  }

  cancel() {
    this.visible.set(false);
    this.rejecter?.();
  }
}
