package org.team9140.frc2026.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final TalonFX motor;

    private static Turret instance;

    public Turret getInstance() {
        return (instance == null) ? instance = new Turret() : instance;
    }

    private Turret() {
        this.motor = new TalonFX(0);
    }
}