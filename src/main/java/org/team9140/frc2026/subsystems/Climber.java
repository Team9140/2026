package org.team9140.frc2026.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team9140.frc2026.Constants;

public class Climber extends SubsystemBase {
    private static Climber instance;
    private final TalonFX motor;

    private Climber() {
        this.motor = new TalonFX(0);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Climber.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.Climber.SUPPLY_CURRENT_LIMIT);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Climber.GEAR_RATIO);

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Constants.Climber.FORWARD_SOFT_LIMIT_THRESHOLD)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.Climber.REVERSE_SOFT_LIMIT_THRESHOLD)
                .withReverseSoftLimitEnable(true);

        TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentLimits)
                .withMotorOutput(motorOutputConfigs)
                .withFeedback(feedbackConfigs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

        this.motor.getConfigurator().apply(motorConfigs);
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public Distance getPosition() {
        return Constants.Climber.SPOOL_RADIUS.times(2*Math.PI*this.motor.getPosition().getValueAsDouble());
    }

    public Command extend() {
        return this.runOnce(() -> this.motor.setControl(new VoltageOut(Constants.Climber.EXTENSION_VOLTAGE)))
                .andThen(new WaitUntilCommand(() -> this.getPosition().gt(Constants.Climber.EXTEND_POSITION)))
                .andThen(this.runOnce(() -> this.motor.setControl(new StaticBrake())));
    }

    public Command retract() {
        return this.runOnce(() -> this.motor.setControl(new VoltageOut(-Constants.Climber.EXTENSION_VOLTAGE)))
                .andThen(new WaitUntilCommand(() -> this.getPosition().lte(Constants.Climber.EXTEND_POSITION)))
                .andThen(this.runOnce(() -> this.motor.setControl(new StaticBrake())));
    }
}