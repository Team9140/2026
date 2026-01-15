package org.team9140.frc2026.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
    private final TalonFX motor;
    private static Intake instance;

    private Intake(){
        this.motor = new TalonFX(0);
    }

    public static Intake getInstance(){
        return (instance == null) ? instance = new Intake() : instance;
    }
}