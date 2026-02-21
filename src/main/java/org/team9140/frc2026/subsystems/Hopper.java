package org.team9140.frc2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.team9140.frc2026.Constants;
import org.team9140.lib.Util;

public class Hopper extends SubsystemBase{
    private final TalonFX spinnerMotor; // This is the spinny thingy
    private final TalonFX outakeMotor; // This feeds to shooter
    private static Hopper instance;

    private Hopper() {
        this.spinnerMotor = new TalonFX(Constants.Ports.HOPPER_SPINNER_MOTOR);
        this.outakeMotor = new TalonFX(Constants.Ports.HOPPER_OUTAKE_MOTOR);
        
        CurrentLimitsConfigs currentSpinnerLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Hopper.SPINNER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

        CurrentLimitsConfigs currentOutakeLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Hopper.OUTAKE_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        
        MotorOutputConfigs spinnerMotorOutputConfig = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

        MotorOutputConfigs outakeMotorOutputConfig = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

        TalonFXConfiguration spinnerMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentSpinnerLimits)
                .withMotorOutput(spinnerMotorOutputConfig);

        TalonFXConfiguration outakeMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentOutakeLimits)
                .withMotorOutput(outakeMotorOutputConfig);

        this.spinnerMotor.getConfigurator().apply(spinnerMotorConfigs);
        this.outakeMotor.getConfigurator().apply(outakeMotorConfigs);
    }

    public Hopper getInstance() {
        return (instance == null) ? instance = new Hopper() : instance;
    }

    public Command startSpinner() {
        return this.runOnce(() -> spinnerMotor.setVoltage(Constants.Hopper.SPINNER_VOLTAGE));
    }

    public Command stopSpinner() {
        return this.runOnce(() -> spinnerMotor.setVoltage(0));
    }

    public Command startOutake() {
        return this.runOnce(() -> outakeMotor.setVoltage(Constants.Hopper.OUTAKE_VOLTAGE));
    }

    public Command stopOutake() {
        return this.runOnce(() -> outakeMotor.setVoltage(0));
    }
}
