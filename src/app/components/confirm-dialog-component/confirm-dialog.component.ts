import { Component, EventEmitter, Input, Output } from '@angular/core';
import { ConfirmService } from '../../services/confirm.service';
import { CommonModule } from '@angular/common';

@Component({
  selector: 'app-confirm-dialog-component',
  imports: [CommonModule],
  templateUrl: './confirm-dialog.component.html',
  styleUrl: './confirm-dialog.component.scss'
})
export class ConfirmDialogComponent {
  constructor(public confirm: ConfirmService) {}
}

