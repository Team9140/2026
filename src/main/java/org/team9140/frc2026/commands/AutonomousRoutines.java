package org.team9140.frc2026.commands;

import java.util.Optional;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.helpers.AimAlign;
import org.team9140.frc2026.subsystems.Climber;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Hopper;
import org.team9140.frc2026.subsystems.Intake;
import org.team9140.frc2026.subsystems.Shooter;
import org.team9140.lib.FollowPath;
import org.team9140.lib.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousRoutines {
    private final CommandSwerveDrivetrain drivetrain;

    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Climber climber = Climber.getInstance();
    private final Hopper hopper = Hopper.getInstance();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private static AutonomousRoutines instance;

    public static AutonomousRoutines getInstance(CommandSwerveDrivetrain drivetrain) {
        return (instance == null) ? instance = new AutonomousRoutines(drivetrain) : instance;
    }

    private AutonomousRoutines(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        autoChooser.setDefaultOption("Do Nothing", doNothing());
        autoChooser.addOption("Shoot Preload", shootPreload(3));
        autoChooser.addOption("Climb Left", climb(true));
        autoChooser.addOption("Climb Right", climb(false));
        autoChooser.addOption("Sweep Middle From Left", sweepMiddleFromLeft());
        // autoChooser.addOption("Shoot Fuel from Middle Left", shootFuelFromMiddleLeft());
        // autoChooser.addOption("Shoot Fuel from Middle Right", shootFuelFromMiddleRight());
        SmartDashboard.putData(autoChooser);
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    public Command doNothing() {
        return new PrintCommand("Doing Nothing");
    }

    public Command shootPreload(double seconds) {
        return Commands.deadline(
            new WaitCommand(seconds),
            shooter.aim(
                () -> AimAlign.getZone(this.drivetrain.getState().Pose),
                () -> this.drivetrain.getState()
            ),
            hopper.feed() 
        );
    }

    public Command climb(boolean left) {
        Pose2d goalPos;
        if(left) {
            if(Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
                goalPos = Constants.Positions.CLIMB_LEFT_BLUE;
            } else {
                goalPos = Constants.Positions.CLIMB_LEFT_RED;
            }
        } else {
            if(Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
                goalPos = Constants.Positions.CLIMB_RIGHT_BLUE;
            } else {
                goalPos = Constants.Positions.CLIMB_RIGHT_RED;
            }
        }
        return this.drivetrain.goToPose(() -> goalPos)
            .until(this.drivetrain.reachedPose)
            .andThen(climber.extend()).andThen(climber.retract());
    }

    public Command sweepMiddleFromLeft() {
        FollowPath path = new FollowPath("crossandsweep_Blue_Right", () -> this.drivetrain.getState().Pose,
                this.drivetrain::followSample, Util.getAlliance().get(), drivetrain);
        return path.gimmeCommand();
    }
    
    // public Command shootFuelFromMiddleLeft() {
    //     Pose2d goalPos;
    //     Pose2d returnPos;
    //     if(Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
    //         goalPos = new Pose2d(Constants.Positions.X_CENTER, 5.189, new Rotation2d());
    //         returnPos = new Pose2d(2.9, 5.189, new Rotation2d());
    //     } else {
    //         goalPos = new Pose2d(Constants.Positions.X_CENTER, 2.879, new Rotation2d());
    //         returnPos = new Pose2d(13.7, 2.879, new Rotation2d());
    //     }
    //     return intake.intake().raceWith(this.drivetrain.goToPose(() -> goalPos)
    //         .until(this.drivetrain.reachedPose)
    //         .andThen(this.drivetrain.goToPose(() -> returnPos))
    //         .until(this.drivetrain.reachedPose)).andThen(shootPreload(3));
    // }

    // public Command shootFuelFromMiddleRight() {
    //     Pose2d goalPos;
    //     Pose2d returnPos;
    //     if(Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
    //         goalPos = new Pose2d(Constants.Positions.X_CENTER, 2.879, new Rotation2d());
    //         returnPos = new Pose2d(2.9, 2.879, new Rotation2d());
    //     } else {
    //         goalPos = new Pose2d(Constants.Positions.X_CENTER, 5.189, new Rotation2d());
    //         returnPos = new Pose2d(13.7, 5.189, new Rotation2d());
    //     }
    //     return intake.intake().raceWith(this.drivetrain.goToPose(() -> goalPos)
    //         .until(this.drivetrain.reachedPose)
    //         .andThen(this.drivetrain.goToPose(() -> returnPos))
    //         .until(this.drivetrain.reachedPose)).andThen(shootPreload(3));
    // }
}