package org.team9140.frc2026.subsystems;

import org.team9140.frc2026.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase{
    private final TalonFX spinnerMotor; // This is the spinny thingy
    private final TalonFX feederMotor; // This feeds to shooter
    private static Hopper instance;

    private Hopper() {
        this.spinnerMotor = new TalonFX(Constants.Ports.HOPPER_SPINNER_MOTOR, Constants.Ports.CANIVORE);
        this.feederMotor = new TalonFX(Constants.Ports.HOPPER_FEEDER_MOTOR, Constants.Ports.CANIVORE);
        
        CurrentLimitsConfigs currentSpinnerLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Hopper.SPINNER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

        CurrentLimitsConfigs currentFeederLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Hopper.FEEDER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        
        MotorOutputConfigs spinnerMotorOutputConfig = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

        MotorOutputConfigs feederMotorOutputConfig = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

        TalonFXConfiguration spinnerMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentSpinnerLimits)
                .withMotorOutput(spinnerMotorOutputConfig);

        TalonFXConfiguration feederMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentFeederLimits)
                .withMotorOutput(feederMotorOutputConfig);

        this.spinnerMotor.getConfigurator().apply(spinnerMotorConfigs);
        this.feederMotor.getConfigurator().apply(feederMotorConfigs);
    }

    public static Hopper getInstance() {
        return (instance == null) ? instance = new Hopper() : instance;
    }

    public Command startSpinner() {
        return this.runOnce(() -> spinnerMotor.setVoltage(Constants.Hopper.SPINNER_VOLTAGE));
    }

    public Command stopSpinner() {
        return this.runOnce(() -> spinnerMotor.setVoltage(0));
    }

    public Command startFeeder() {
        return this.runOnce(() -> feederMotor.setVoltage(Constants.Hopper.FEEDER_VOLTAGE));
    }

    public Command stopFeeder() {
        return this.runOnce(() -> feederMotor.setVoltage(0));
    }
}
