package org.team9140.frc2026.commands;

import org.team9140.frc2026.helpers.AimAlign;
import org.team9140.frc2026.subsystems.Climber;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Intake;
import org.team9140.frc2026.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousRoutines {
    CommandSwerveDrivetrain drivetrain;

    Shooter shooter = Shooter.getInstance();
    Intake intake = Intake.getInstance();
    Climber climber = Climber.getInstance();

    public AutonomousRoutines(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }    

    public Command shootPreload(double seconds) {
        Pose2d currPos = drivetrain.getState().Pose;
        Translation2d goalPos = AimAlign.getZone(currPos).getTranslation();
        return shooter.setYawAngle(AimAlign.yawAngleToPos(
            currPos, 
            goalPos
            )).andThen(shooter.setSpeed(AimAlign.getRequiredSpeed(
                currPos, 
                goalPos, 
                drivetrain.getState().Speeds
            ))).andThen(new WaitCommand(seconds)
            ).andThen(shooter.off());
    }
    
}