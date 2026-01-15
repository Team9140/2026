package org.team9140.frc2026.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final TalonFX motor;

    private static Shooter instance;

    public Shooter getInstance() {
        return (instance == null) ? instance = new Shooter() : instance;
    }

    private Shooter() {
        this.motor = new TalonFX(0);
    }
}
