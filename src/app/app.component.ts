import { Component } from '@angular/core';
import { RobotDisplayComponent } from './components/robot-arm/robot-display/robot-display.component';
import { JointsMenuComponent } from "./components/robot-arm/joints-menu/joints-menu.component";
import { UiStateService } from './services/ui-state.service';
import { CommonModule } from '@angular/common';


@Component({
  selector: 'app-root',
  standalone: true,
  imports: [RobotDisplayComponent, JointsMenuComponent, CommonModule],
  templateUrl: './app.component.html',
  styleUrl: './app.component.scss'
})
export class AppComponent {
  title = 'robot-UI';
  isManualMode = false;
  constructor(private uiStateService: UiStateService) {
    this.uiStateService.manualMode$.subscribe((isManual) => {
      this.isManualMode = isManual;
    });
  }
}
