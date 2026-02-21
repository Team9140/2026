package org.team9140.frc2026.commands;

import org.team9140.frc2026.subsystems.Climber;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Intake;
import org.team9140.frc2026.subsystems.Shooter;

public class AutonomousRoutines {
    CommandSwerveDrivetrain drivetrain;

    Shooter shooter = Shooter.getInstance();
    Intake intake = Intake.getInstance();
    Climber climber = Climber.getInstance();

    public AutonomousRoutines(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }    
}