import { Injectable } from '@angular/core';

const AUTH_KEY = 'robot_auth_password';

@Injectable({
  providedIn: 'root'
})
export class AuthService {
  private password: string | null = null;

  constructor() {
    this.password = localStorage.getItem(AUTH_KEY);
  }

  setPassword(newPassword: string): void {
    this.password = newPassword;
    localStorage.setItem(AUTH_KEY, newPassword);
  }

  getPassword(): string | null {
    return this.password;
  }

  isLoggedIn(): boolean {
    return !!this.password;
  }

  logout(): void {
    this.password = null;
    localStorage.removeItem(AUTH_KEY);
  }
}