// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2026;

import org.team9140.frc2026.commands.AutonomousRoutines;
import org.team9140.frc2026.generated.TunerConstants;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Hopper;
import org.team9140.frc2026.subsystems.Intake;
import org.team9140.frc2026.subsystems.Shooter;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  // private final Climber climber = Climber.getInstance();
  private final Hopper hopper = Hopper.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Shooter shooter = Shooter.getInstance();

  private final CommandXboxController controller = new CommandXboxController(0);
  private final SwerveTelemetry logger = new SwerveTelemetry(drivetrain, Constants.Drive.MAX_VELOCITY);
  private final AutonomousRoutines autoRoutines;
  private final Command driveCommand;

  private final Vision limeA = new Vision(Constants.Vision.CAMERA_NAMES[0], this.drivetrain::acceptVisionMeasurement,
      Constants.Vision.ROBOT_TO_CAM[0]);
  private final Vision limeB = new Vision(Constants.Vision.CAMERA_NAMES[1], this.drivetrain::acceptVisionMeasurement,
      Constants.Vision.ROBOT_TO_CAM[1]);
  private final Vision limeC = new Vision(Constants.Vision.CAMERA_NAMES[2], this.drivetrain::acceptVisionMeasurement, 
      Constants.Vision.ROBOT_TO_CAM[2]);

  public RobotContainer() {
    limeA.setIMUMode(1);
    limeB.setIMUMode(1);
    limeC.setIMUMode(1);
    driveCommand = drivetrain.teleopDrive(controller::getLeftX, controller::getLeftY,
        controller::getRightX);

    configureBindings();

    limeA.start();
    limeB.start();
    limeC.start();

    autoRoutines = AutonomousRoutines.getInstance(drivetrain);

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
  }

  private void configureBindings() {
    this.shooter.setPoseSupplier(() -> this.drivetrain.getCachedState().Pose);
    SmartDashboard.putNumber("tuning RPM", 2500);
    SmartDashboard.putNumber("tuning Angle", 24.0);

    // this.controller.rightBumper()
    //     .onTrue(this.intake.intake())
    //     .onFalse(this.intake.off());

    this.controller.leftBumper()
        .onTrue(this.intake.armIn().andThen(this.hopper.unjam()))
        .onFalse(this.intake.off().alongWith(this.hopper.off()));

    Trigger wantAim = this.controller.rightTrigger(0.3).debounce(Constants.Turret.TURN_SHOOTER_OFF_TIME, DebounceType.kFalling);
    Trigger wantShoot = this.controller.rightTrigger(0.9);

    wantShoot.and(shooter.readyToShoot).whileTrue(hopper.feed()).onFalse(hopper.reverseAndOff());
    wantAim.whileTrue(this.shooter.aim(this.drivetrain::getCachedState));

    Trigger wantIntake = this.controller.rightBumper();

    wantIntake.whileTrue(this.intake.intake());

    // wantShoot.and(shooter.readyToShoot).and(wantIntake.negate()).debounce(1.0).onTrue(this.intake.squeeze());

    this.controller.y().toggleOnTrue(this.shooter.tuning(this.drivetrain::getCachedState, () -> SmartDashboard.getNumber("tuning RPM", 2500), () -> SmartDashboard.getNumber("tuning Angle", 24.0)));
    // this.controller.a().onTrue(this.shooter.aim(this.drivetrain::getCachedState));
    // this.controller.x().onTrue(this.shooter.off());
    this.controller.leftTrigger().onTrue(hopper.feed()).onFalse(hopper.reverseAndOff());

    this.controller.x().onTrue(this.intake.squeeze());

    // this.controller.back().whileTrue(this.shooter.manualLeft());
    // this.controller.start().whileTrue(this.shooter.manualRight());
    // this.controller.y().whileTrue(this.drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // this.controller.a().whileTrue(this.drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // this.controller.b().whileTrue(this.drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // this.controller.x().whileTrue(this.drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    drivetrain.setDefaultCommand(driveCommand);

    this.drivetrain.registerTelemetry(logger::telemeterize);
  }

  Command autonomousRoutine = null;

  public Command getAutonomousCommand() {
    Command path = autoRoutines.getCommand();

    if (path != null) {
      this.autonomousRoutine = path.finallyDo((interrupted) -> {
        if (interrupted)
          CommandScheduler.getInstance().schedule(hopper.off().andThen(shooter.off()));
      });
    }

    return autonomousRoutine;
  }
}