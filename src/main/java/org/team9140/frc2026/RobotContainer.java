// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2026;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.team9140.frc2026.generated.TunerConstants;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Turret;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import org.team9140.frc2026.Constants;


public class RobotContainer {

  private Turret turret = Turret.getInstance();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final SwerveTelemetry logger = new SwerveTelemetry(Constants.Drive.MAX_TELEOP_VELOCITY);

private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MAX_TELEOP_VELOCITY * 0.1).withRotationalDeadband(Constants.Drive.MAX_TELEOP_ROTATION * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
     drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * Constants.Drive.MAX_TELEOP_VELOCITY) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * Constants.Drive.MAX_TELEOP_VELOCITY) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * Constants.Drive.MAX_TELEOP_ROTATION) // Drive counterclockwise with negative X (left)
            )
        );


    this.driverController.rightBumper().onTrue(turret.movePitchToPosition(Math.PI));
    this.driverController.rightTrigger().onTrue(turret.movePitchToPosition(0));
    
    this.driverController.leftBumper().onTrue(turret.moveYawToPosition(Math.PI));
    this.driverController.leftTrigger().onTrue(turret.moveYawToPosition(0));

    this.drivetrain.registerTelemetry(logger::telemeterize);

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
