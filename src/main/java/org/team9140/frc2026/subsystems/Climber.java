package org.team9140.frc2026.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;


public class Climber extends SubsystemBase {
    private static Climber instance;
    private final TalonFX motor;

    private Climber() {
        this.motor = new TalonFX(0);
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }
}