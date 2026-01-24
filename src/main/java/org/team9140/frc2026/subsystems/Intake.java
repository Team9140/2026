package org.team9140.frc2026.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.team9140.frc2026.Constants;

public class Intake extends SubsystemBase {
    private final TalonFX spinMotor;
    private final TalonFX extendMotor;
    private static Intake instance;
    private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0);


    private Intake(){
        this.spinMotor = new TalonFX(0);
        this.extendMotor = new TalonFX(1);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Intake.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

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
                .withMotorOutput(motorOutputConfigs)
                .withMotionMagic(motionMagicConfigs);

        TalonFXConfiguration extendMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentLimits)
                .withMotorOutput(motorOutputConfigs)
                .withMotionMagic(motionMagicConfigs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);


        this.spinMotor.getConfigurator().apply(spinMotorConfigs);
        this.extendMotor.getConfigurator().apply(extendMotorConfigs);
    }

    public static Intake getInstance(){
        return (instance == null) ? instance = new Intake() : instance;
    }

    public Distance getPosition(boolean refresh) {
        return Constants.Elevator.SPOOL_CIRCUMFERENCE
                .times(this.rightMotor.getPosition(false).getValue().in(Rotations));
    }

    public Command setPosition(Distance position) {
        return this.runOnce(() -> {
            this.targetPosition = position;
            this.extendMotor.setControl(this.motionMagic.withPosition(0));
        }).andThen(new WaitUntilCommand(atPosition));
    }

    public final Trigger atPosition = new Trigger(() -> this.getPosition(true).isNear(this.targetPosition, Constants.Elevator.POSITION_epsilon));

    
    //convertion of motor rotations to length extended
    public Command armIn(){
        return this.runOnce(() -> {
            extendMotor.setControl(motionMagic.withPosition(Constants.Intake.ARM_IN_POSITION));
        }).andThen(new WaitUntilCommand(() -> extendMotor.getPosition().getValueAsDouble() == Constants.Intake.ARM_IN_POSITION));
    }

    public Command armOut(){
        return this.runOnce(() -> {
            extendMotor.setControl(motionMagic.withPosition(Constants.Intake.ARM_OUT_POSITION));
        }).andThen(new WaitUntilCommand(() -> extendMotor.getPosition().getValueAsDouble() == Constants.Intake.ARM_OUT_POSITION));
    }

    public Command off(){
        return this.armIn().andThen(this.runOnce(() -> {
            this.spinMotor.setVoltage(Constants.Intake.OFF);
            this.extendMotor.setVoltage(Constants.Intake.OFF);
        }));
    }

    public Command intake(){
        return this.armOut().andThen(this.runOnce(() -> {
            this.spinMotor.setVoltage(Constants.Intake.INTAKE_VOLTAGE);
            this.extendMotor.setVoltage(Constants.Intake.INTAKE_VOLTAGE);
        }));
    }

    public Command reverse(){
        return  this.armOut().andThen(this.runOnce(() -> {
            this.spinMotor.setVoltage(Constants.Intake.REVERSE_INTAKE);
        }));
    }
}