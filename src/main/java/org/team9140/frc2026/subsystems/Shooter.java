package org.team9140.frc2026.subsystems;

import org.team9140.frc2026.Constants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final TalonFX motor;

    private static Shooter instance;

    private MotionMagicVelocityTorqueCurrentFOC shooterMM = new MotionMagicVelocityTorqueCurrentFOC(0);

    public static Shooter getInstance() {
        return (instance == null) ? instance = new Shooter() : instance;
    }

    private Shooter() {
        this.motor = new TalonFX(0);

        MotionMagicConfigs shooterMMConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(Constants.Shooter.SHOOTER_ACCELERATION)
            .withMotionMagicCruiseVelocity(Constants.Shooter.SHOOTER_CRUISE_VELOCITY);

        Slot0Configs shooterSlot0Configs = new Slot0Configs()
            .withKS(Constants.Shooter.SHOOTER_KS)
            .withKV(Constants.Shooter.SHOOTER_KV)
            .withKA(Constants.Shooter.SHOOTER_KA)
            .withKP(Constants.Shooter.SHOOTER_KP)
            .withKI(Constants.Shooter.SHOOTER_KI)
            .withKD(Constants.Shooter.SHOOTER_KD);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
            .withMotionMagic(shooterMMConfigs)
            .withSlot0(shooterSlot0Configs);

        shooterMM = new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
        motor.getConfigurator().apply(shooterConfig);
        motor.setControl(shooterMM.withVelocity(0));
    }

    public Command setShooterReleaseSpeed(double releaseSpeed) {
        return this.runOnce(() -> {
            motor.setControl(shooterMM.withVelocity(releaseSpeed * Constants.Shooter.SHOOT_VELOCITY_RATIO / 2.0 / Math.PI));
        });
    }

    public Command shoot(double velocity, double time) {
        return this.runOnce(() -> {
            setShooterReleaseSpeed(velocity).andThen(Commands.waitSeconds(time).andThen(setShooterReleaseSpeed(0)));
        });
    }

    public Command shootAtEndPosistion(Pose2d turretPos, Translation3d endPos, double time, double robotSpeed) {
        return this.runOnce(() -> 
            Turret.getInstance().aimAtPosition(turretPos, endPos).andThen(shoot(AimAlign.speedToPosition(turretPos, endPos, robotSpeed), time))
        );
    }

}
