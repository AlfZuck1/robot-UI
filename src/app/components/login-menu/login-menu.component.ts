import { Component } from '@angular/core';
import { AuthService } from '../../services/auth.service';
import { FormsModule } from '@angular/forms';
import { CommonModule } from '@angular/common';


@Component({
  selector: 'app-login-menu',
  imports: [FormsModule, CommonModule],
  templateUrl: './login-menu.component.html',
  styleUrl: './login-menu.component.scss'
})
export class LoginMenuComponent {

  open = false;
  password = '';

  constructor(public auth: AuthService) { }

  toggle() {
    this.open = !this.open;
  }

  login() {
    if (this.password.trim().length > 0) {
      this.auth.setPassword(this.password);
      this.password = '';
      this.open = false;
    }
  }

  logout() {
    this.auth.logout();
    this.open = false;
  }
}