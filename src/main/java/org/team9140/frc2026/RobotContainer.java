// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2026;

import java.util.function.Supplier;

import org.team9140.frc2026.generated.TunerConstants;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.getDrivetrain();
  private final SwerveTelemetry logger = new SwerveTelemetry(Constants.Drive.MAX_TELEOP_VELOCITY);
  private Supplier<Pose2d> poseSup = ()-> drivetrain.getState().Pose;
  private final Shooter shooter = Shooter.getInstance(poseSup);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.teleopDrive(controller::getLeftX, controller::getLeftY,
            controller::getRightX).ignoringDisable(true));
    
    this.drivetrain.registerTelemetry(logger::telemeterize);

    controller.a().whileTrue(shooter.aimAtPosition(Constants.Turret.HOOP_POSITION.getTranslation()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}