// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2026;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.team9140.frc2026.subsystems.Turret;

public class RobotContainer {

  private Turret turret = Turret.getInstance();
  private final CommandXboxController driverController = new CommandXboxController(0);

  private Joystick testJoystick = new Joystick(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    this.driverController.rightBumper().onTrue(turret.movePitchToPosition(Math.PI));
    this.driverController.rightTrigger().onTrue(turret.movePitchToPosition(0));
    
    this.driverController.leftBumper().onTrue(turret.moveYawToPosition(Math.PI));
    this.driverController.leftTrigger().onTrue(turret.moveYawToPosition(0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
