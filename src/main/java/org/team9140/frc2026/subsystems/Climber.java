package org.team9140.frc2026.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team9140.frc2026.Constants;

public class Climber extends SubsystemBase {
    private static Climber instance;
    private final TalonFX motor;

    // TODO: what are these numbers mean
    // TODO: read this https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
    //pretend its meters
    Mechanism2d climberMechanism = new Mechanism2d(Constants.Climber.MECHANISM_WIDTH, Constants.Climber.MECHANISM_HEIGHT);
    MechanismRoot2d root = climberMechanism.getRoot("root", 32, 14);
    MechanismLigament2d telescope = root.append(new MechanismLigament2d("telescope", 20, 0));

    private Climber() {
        this.motor = new TalonFX(Constants.Ports.CLIMBER_MOTOR, Constants.Ports.CANIVORE);

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

    public double getPosition() {
        return Constants.Climber.SPOOL_RADIUS*2*Math.PI*this.motor.getPosition().getValueAsDouble();
    }

    public Command extend() {
        return this.runOnce(() -> this.motor.setControl(new VoltageOut(Constants.Climber.EXTENSION_VOLTAGE)))
                .andThen(new WaitUntilCommand(() -> this.getPosition() > Constants.Climber.EXTEND_POSITION))
                .andThen(this.runOnce(() -> this.motor.setControl(new StaticBrake())));
    }

    public Command retract() {
        return this.runOnce(() -> this.motor.setControl(new VoltageOut(-Constants.Climber.EXTENSION_VOLTAGE)))
                .andThen(new WaitUntilCommand(() -> this.getPosition() <= Constants.Climber.EXTEND_POSITION))
                .andThen(this.runOnce(() -> this.motor.setControl(new StaticBrake())));
    }

    private static final double kSimLoopPeriod = Constants.Climber.SIM_PERIOD; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    ElevatorSim extensionSim = new ElevatorSim(DCMotor.getKrakenX44(1),
            Constants.Climber.GEAR_RATIO,
            1,
            Constants.Climber.SPOOL_CIRCUMFERENCE / Math.PI / 2.0,
            Constants.Climber.MIN_HEIGHT,
            Constants.Climber.MAX_HEIGHT,
            false,
            0);

    private void updateSimState(double t, double volts) {
        double extendVolts = this.motor.getSimState().getMotorVoltage();
        this.extensionSim.setInputVoltage(extendVolts);
        this.extensionSim.update(t);

        double pos = this.extensionSim.getPositionMeters();
        double vel = this.extensionSim.getVelocityMetersPerSecond();

        this.motor.getSimState().setRawRotorPosition(pos * Constants.Climber.SPOOL_CIRCUMFERENCE * Constants.Climber.GEAR_RATIO);
        this.motor.getSimState().setRotorVelocity(vel * Constants.Climber.SPOOL_CIRCUMFERENCE * Constants.Climber.GEAR_RATIO);

        telescope.setLength(pos);
    }
}