package org.team9140.frc2026.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.team9140.frc2026.Constants;
import org.team9140.lib.Util;

public class Intake extends SubsystemBase {
    private final TalonFX spinMotor;
    private final TalonFX extendMotor;
    private static Intake instance;
    private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0);

    private Intake() {
        this.spinMotor = new TalonFX(0); // move magic numbers to constants
        this.extendMotor = new TalonFX(1);

        // separate extend and spin current limits
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Intake.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

        // separate extend and spin
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.Intake.MOTION_MAGIC_CRUISE_VELOCITY)
                .withMotionMagicAcceleration(Constants.Intake.MOTION_MAGIC_ACCELERATION);

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Constants.Intake.FORWARD_SOFT_LIMIT_THRESHOLD)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.Intake.REVERSE_SOFT_LIMIT_THRESHOLD)
                .withReverseSoftLimitEnable(true);

        TalonFXConfiguration spinMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentLimits)
                .withMotorOutput(motorOutputConfigs);

        TalonFXConfiguration extendMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentLimits)
                .withMotorOutput(motorOutputConfigs)
                .withMotionMagic(motionMagicConfigs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

        // set extension sensor to mechanism ratio

        this.spinMotor.getConfigurator().apply(spinMotorConfigs);
        this.extendMotor.getConfigurator().apply(extendMotorConfigs);
    }

    public static Intake getInstance() {
        return (instance == null) ? instance = new Intake() : instance;
    }

    @Override
    public void periodic() {
        this.extendMotor.getPosition().refresh();
    }

    // javadoc comment every method that uses position to specify units
    double targetPosition = 0;

    /**
     * 
     * @return how many meters extended out is the thing
     */
    public double getPosition() {
        return Constants.Intake.PINION_CIRCUMFERENCE * this.extendMotor.getPosition(false).getValueAsDouble();
    }

    /**
     * 
     * @param position Target extension in meters
     * @return
     */
    public Command setPosition(double position) {
        return this.runOnce(() -> {
            this.targetPosition = position;
            this.extendMotor.setControl(this.motionMagic.withPosition(this.targetPosition));
        }).andThen(new WaitUntilCommand(atPosition));
    }

    public final Trigger atPosition = new Trigger(
            () -> Util.epsilonEquals(getPosition(), this.targetPosition, Units.inchesToMeters(0.5))); // move 0.5 inch tolerance to a Constant

    // convertion of motor rotations to length extended
    public Command armIn() {
        return this.setPosition(in_position);
    }

    public Command armOut() {
        return this.setPosition(out_position);
    }

    public Command setRollerSpeed(double speed) {
        return null; // this.runOnce, then make three methods below compose with this method
    }

    public Command off() {
        return this.armIn().andThen(this.runOnce(() -> {
            this.spinMotor.setVoltage(Constants.Intake.OFF);
        }));
    }

    public Command intake() {
        return this.armOut().andThen(this.runOnce(() -> {
            this.spinMotor.setVoltage(Constants.Intake.INTAKE_VOLTAGE);
        }));
    }

    public Command reverse() {
        return this.armOut().andThen(this.runOnce(() -> {
            this.spinMotor.setVoltage(Constants.Intake.REVERSE_INTAKE);
        }));
    }
}