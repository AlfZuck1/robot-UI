import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class AuthService {
  private password: string | null = null;

  setPassword(newPassword: string): void {
    this.password = newPassword;
  }

  getPassword(): string | null {
    return this.password;
  }

  isLoggedIn(): boolean {
    return this.password !== null && this.password.length > 0;
  }

  clearPassword(): void {
    this.password = null;
  }
}
