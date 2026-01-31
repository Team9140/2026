package org.team9140.frc2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
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
        this.spinMotor = new TalonFX(Constants.Ports.INTAKE_SPIN_MOTOR);
        this.extendMotor = new TalonFX(Constants.Ports.INTAKE_EXTEND_MOTOR);

        CurrentLimitsConfigs currentSpinLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Intake.SPIN_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

        CurrentLimitsConfigs currentExtendLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Intake.EXTEND_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

        MotorOutputConfigs spinMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);

        MotorOutputConfigs extendMotorOutputConfigs = new MotorOutputConfigs()
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
                .withCurrentLimits(currentSpinLimits)
                .withMotorOutput(spinMotorOutputConfigs);

        TalonFXConfiguration extendMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentExtendLimits)
                .withMotorOutput(extendMotorOutputConfigs)
                .withMotionMagic(motionMagicConfigs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

        extendMotorConfigs.Feedback.SensorToMechanismRatio = Constants.Intake.EXTENSION_GEAR_RATIO;

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

    double targetPosition = 0;

    /**
     * @return how many meters extended out is the thing
     */
    public double getPosition() {
        return Constants.Intake.PINION_CIRCUMFERENCE * this.extendMotor.getPosition(false).getValueAsDouble();
    }

    /**
     * @param position Target extension in meters
     * @return command that moves the arm to the given position
     */
    public Command setPosition(double position) {
        return this.runOnce(() -> {
            this.targetPosition = position;
            this.extendMotor.setControl(this.motionMagic.withPosition(this.targetPosition));
        }).andThen(new WaitUntilCommand(atPosition));
    }

    public final Trigger atPosition = new Trigger(
            () -> Util.epsilonEquals(getPosition(), this.targetPosition, Units.inchesToMeters(Constants.Intake.TOLERANCE))); // move 0.5 inch tolerance to a Constant

    /**
     * @return command that moves the arm to the "in" position (meters)
     */
    public Command armIn() {
        return this.setPosition(Constants.Intake.ARM_IN_POSITION);
    }

    /**
     * @return command that moves the arm to the "out" position (meters)
     */
    public Command armOut() {
        return this.setPosition(Constants.Intake.ARM_OUT_POSITION);
    }

    public Command setRollerSpeed(double speed) {
        return this.runOnce(()-> spinMotor.set(speed));
    }

    public Command off() {
        return this.armIn().andThen(this.runOnce(() -> {
            setRollerSpeed(Constants.Intake.INTAKE_OFF);
        }));
    }

    public Command intake() {
        return this.armOut().andThen(this.runOnce(() -> {
            setRollerSpeed(Constants.Intake.INTAKE_VOLTAGE);
        }));
    }

    public Command reverse() {
        return this.armOut().andThen(this.runOnce(() -> {
            setRollerSpeed(-Constants.Intake.INTAKE_VOLTAGE);
        }));
    }
}