import { Component } from '@angular/core';
import { AuthService } from '../../services/auth.service';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';

@Component({
  selector: 'app-login-dialog',
  imports: [CommonModule, FormsModule],
  templateUrl: './login-dialog.component.html',
  styleUrl: './login-dialog.component.scss'
})
export class LoginDialogComponent {
  password: string = '';

  constructor(private authService: AuthService) {}

  login() {
    if (this.password.trim().length === 0) {
      alert('Por favor, ingresa una contraseña válida.');
      return;
    }
    this.authService.setPassword(this.password);
  }
}
