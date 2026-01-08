import { inject } from '@angular/core';
import {
  HttpInterceptorFn,
  HttpErrorResponse
} from '@angular/common/http';
import { catchError, throwError } from 'rxjs';

import { ErrorService } from '../services/error.service';
import { AuthService } from '../services/auth.service';

export const errorInterceptor: HttpInterceptorFn = (req, next) => {

  const errorService = inject(ErrorService);
  const authService = inject(AuthService);

  return next(req).pipe(
    catchError((error: HttpErrorResponse) => {

      if (error.status === 401) {
        errorService.showError({
          status: 401,
          title: 'Unauthorized',
          message: 'Contraseña incorrecta o sesión expirada.'
        });
        authService.logout();
      }

      else if (error.status === 403) {
        errorService.showError({
          status: 403,
          title: 'Acceso denegado',
          message: 'No tienes permisos para realizar esta acción.'
        });
      }

      else {
        errorService.showError({
          status: error.status,
          title: 'Error del sistema',
          message: error.message || 'Ha ocurrido un error inesperado.'
        });
      }

      // 🔥 MUY IMPORTANTE: volver a lanzar el error original
      return throwError(() => error);
    })
  );
};
