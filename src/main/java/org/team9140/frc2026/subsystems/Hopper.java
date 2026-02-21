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
    private final TalonFX outakeMotor; // This feeds to shooter
    private static Hopper instance;

    private Hopper() {
        this.spinnerMotor = new TalonFX(Constants.Ports.HOPPER_SPINNER_MOTOR, Constants.Ports.CANIVORE);
        this.outakeMotor = new TalonFX(Constants.Ports.HOPPER_OUTAKE_MOTOR, Constants.Ports.CANIVORE);
        
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

    public static Hopper getInstance() {
        return (instance == null) ? instance = new Hopper() : instance;
    }

    public Command generateSetVoltageCommand(double volts, TalonFX motor) {
        return this.runOnce(() -> motor.setVoltage(volts));
    }

    public Command forward() {
        return this.runOnce(() -> generateSetVoltageCommand(Constants.Hopper.SPINNER_VOLTAGE, spinnerMotor))
                .alongWith(generateSetVoltageCommand(Constants.Hopper.OUTAKE_VOLTAGE, outakeMotor));
    }

    public Command backward() {
        return this.runOnce(() -> generateSetVoltageCommand(Constants.Hopper.SPINNER_VOLTAGE, spinnerMotor))
                .alongWith(generateSetVoltageCommand(Constants.Hopper.OUTAKE_VOLTAGE, outakeMotor));
    }

    public Command hopperOff() {
        return this.runOnce(() -> generateSetVoltageCommand(0, spinnerMotor))
                .alongWith(generateSetVoltageCommand(0, outakeMotor));
    }
}
