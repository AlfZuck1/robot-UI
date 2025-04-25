import { Component } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { RobotArmComponent } from "./robot-arm/robot-arm.component";

@Component({
  selector: 'app-root',
  imports: [RobotArmComponent],
  templateUrl: './app.component.html',
  styleUrl: './app.component.scss'
})
export class AppComponent {
  title = 'robot-UI';
}
