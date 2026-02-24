package org.team9140.frc2026.commands;

import java.util.Optional;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.helpers.AimAlign;
import org.team9140.frc2026.subsystems.Climber;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Hopper;
import org.team9140.frc2026.subsystems.Intake;
import org.team9140.frc2026.subsystems.Shooter;
import org.team9140.lib.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousRoutines {
    CommandSwerveDrivetrain drivetrain;

    Shooter shooter = Shooter.getInstance();
    Intake intake = Intake.getInstance();
    Climber climber = Climber.getInstance();
    Hopper hopper = Hopper.getInstance();

    public AutonomousRoutines(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }    

    public Command shootPreload(double seconds) {
        Pose2d currPos = drivetrain.getState().Pose;
        ChassisSpeeds currSpeed = drivetrain.getState().Speeds;
        Translation2d goalPos = AimAlign.getZone(currPos).getTranslation();
        return shooter.setYawAngle(AimAlign.yawAngleToPos(
            currPos, 
            goalPos
            )).andThen(Commands.deadline(
                new WaitCommand(seconds),
                new PrintCommand("Start Hopper Feed"), 
                shooter.setSpeed(AimAlign.getRequiredSpeed(currPos, goalPos, currSpeed)
                )));
    }

    public Command climbLeft() {
        Pose2d goalPos;
        if(Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
            goalPos = Constants.Positions.CLIMB_LEFT_BLUE;
        } else {
            goalPos = Constants.Positions.CLIMB_LEFT_RED;
        }
        return this.drivetrain.goToPose(() -> goalPos)
            .until(this.drivetrain.reachedPose)
            .andThen(climber.extend()).andThen(climber.retract());
    }

    public Command climbRight() {
        Pose2d goalPos;
        if(Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
            goalPos = Constants.Positions.CLIMB_RIGHT_BLUE;
        } else {
            goalPos = Constants.Positions.CLIMB_RIGHT_RED;
        }
        return this.drivetrain.goToPose(() -> goalPos)
            .until(this.drivetrain.reachedPose)
            .andThen(climber.extend()).andThen(climber.retract());
    }
    
    public Command shootFuelFromMiddleLeft() {
        Pose2d goalPos;
        Pose2d returnPos;
        if(Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
            goalPos = new Pose2d(Constants.Positions.X_CENTER, 5.189, new Rotation2d());
            returnPos = new Pose2d(2.9, 5.189, new Rotation2d());
        } else {
            goalPos = new Pose2d(Constants.Positions.X_CENTER, 2.879, new Rotation2d());
            returnPos = new Pose2d(13.7, 2.879, new Rotation2d());
        }
        return intake.intake().raceWith(this.drivetrain.goToPose(() -> goalPos)
            .until(this.drivetrain.reachedPose)
            .andThen(this.drivetrain.goToPose(() -> returnPos))
            .until(this.drivetrain.reachedPose)).andThen(shootPreload(3));
    }

    public Command shootFuelFromMiddleRight() {
        Pose2d goalPos;
        Pose2d returnPos;
        if(Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
            goalPos = new Pose2d(Constants.Positions.X_CENTER, 2.879, new Rotation2d());
            returnPos = new Pose2d(2.9, 2.879, new Rotation2d());
        } else {
            goalPos = new Pose2d(Constants.Positions.X_CENTER, 5.189, new Rotation2d());
            returnPos = new Pose2d(13.7, 5.189, new Rotation2d());
        }
        return intake.intake().raceWith(this.drivetrain.goToPose(() -> goalPos)
            .until(this.drivetrain.reachedPose)
            .andThen(this.drivetrain.goToPose(() -> returnPos))
            .until(this.drivetrain.reachedPose)).andThen(shootPreload(3));
    }
}