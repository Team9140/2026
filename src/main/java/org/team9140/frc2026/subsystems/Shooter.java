package org.team9140.frc2026.subsystems;

import java.util.function.Supplier;

import org.team9140.frc2026.Constants;
import org.team9140.lib.Util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase {
        private final TalonFX yawMotor = new TalonFX(Constants.Ports.YAW_MOTOR);
        private final TalonFX shooterMotor = new TalonFX(Constants.Ports.SHOOTER_MOTOR);

        private double yawTargetPosition = 0;
        private double shooterTargetVelocity = 0;

        private final MotionMagicTorqueCurrentFOC yawMotorControl = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
        private final VelocityTorqueCurrentFOC shooterSpeedControl = new VelocityTorqueCurrentFOC(0).withSlot(0);
        private final VoltageOut shooterIdleControl = new VoltageOut(0);
        private final CoastOut shooterOffControl = new CoastOut();

        private final SingleJointedArmSim yawMotorSim = new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(1),
                        60,
                        1,
                        0.2,
                        -Math.PI,
                        Math.PI,
                        false,
                        0);
        private final MechanismLigament2d yawArmLigament;

        private TalonFXSimState shooterMotorSimState;
        private final FlywheelSim shooterMotorSim = new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(2), 0.2, 60),
                        DCMotor.getKrakenX60Foc(2));

        private final StructPublisher<Pose3d> publisher1 = NetworkTableInstance.getDefault()
                        .getStructTopic("shooter", Pose3d.struct).publish();
        private Pose3d shooterPos = new Pose3d();

        private static Shooter instance;

        public static Shooter getInstance() {
                return (instance == null) ? instance = new Shooter() : instance;
        }

        private Shooter() {
                MotionMagicConfigs yawMMConfigs = new MotionMagicConfigs()
                                .withMotionMagicAcceleration(Constants.Shooter.YAW_ACCELERATION)
                                .withMotionMagicCruiseVelocity(Constants.Shooter.YAW_CRUISE_VELOCITY);

                Slot0Configs yawSlot0Configs = new Slot0Configs()
                                .withKS(Constants.Shooter.YAW_KS)
                                .withKV(Constants.Shooter.YAW_KV)
                                .withKA(Constants.Shooter.YAW_KA)
                                .withKP(Constants.Shooter.YAW_KP)
                                .withKI(Constants.Shooter.YAW_KI)
                                .withKD(Constants.Shooter.YAW_KD);
                
                MotorOutputConfigs yawMotorOutputConfigs = new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive);

                Slot0Configs shooterSlot0Configs = new Slot0Configs()
                                .withKS(Constants.Shooter.SHOOTER_KS)
                                .withKV(Constants.Shooter.SHOOTER_KV)
                                .withKA(Constants.Shooter.SHOOTER_KA)
                                .withKP(Constants.Shooter.SHOOTER_KP)
                                .withKI(Constants.Shooter.SHOOTER_KI)
                                .withKD(Constants.Shooter.SHOOTER_KD);

                MotorOutputConfigs shooterMotorOutputConfigs = new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive);

                TorqueCurrentConfigs shooterTorqueCurrentConfigs = new TorqueCurrentConfigs()
                                .withPeakForwardTorqueCurrent(Constants.Shooter.PEAK_FORWARD_TORQUE)
                                .withPeakReverseTorqueCurrent(0.0);

                TalonFXConfiguration yawConfig = new TalonFXConfiguration()
                                .withMotionMagic(yawMMConfigs)
                                .withSlot0(yawSlot0Configs)
                                .withMotorOutput(yawMotorOutputConfigs);

                TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
                                .withSlot0(shooterSlot0Configs)
                                .withMotorOutput(shooterMotorOutputConfigs)
                                .withTorqueCurrent(shooterTorqueCurrentConfigs);

                yawMotor.getConfigurator().apply(yawConfig);
                shooterMotor.getConfigurator().apply(shooterConfig);

                yawMotor.setControl(yawMotorControl.withPosition(0));
                shooterMotor.setControl(shooterOffControl);

                Mechanism2d yawMech = new Mechanism2d(1, 1);
                MechanismRoot2d yawRoot = yawMech.getRoot("yawArm Root", 1.5, 0.5);
                yawArmLigament = yawRoot.append(new MechanismLigament2d(
                                "yawArm",
                                0.3,
                                0,
                                6,
                                new Color8Bit(Color.kYellow)));

                SmartDashboard.putData("YAW ARM MECHANISM", yawMech);
        }

        public Command setYawAngle(double pos) {
                return this.runOnce(() -> {
                        this.yawTargetPosition = pos - Math.floor(pos / 2.0 / Math.PI) * 2.0 * Math.PI - Math.PI;
                        yawMotor.setControl(yawMotorControl.withPosition(yawTargetPosition / 2.0 / Math.PI));
                });
        }

        public Command setYawAngle(Supplier<Double> pos) {
                return this.runOnce(() -> {
                        this.yawTargetPosition = pos.get() - Math.floor(pos.get() / 2.0 / Math.PI) * 2.0 * Math.PI
                                        - Math.PI;
                        yawMotor.setControl(yawMotorControl.withPosition(yawTargetPosition / 2.0 / Math.PI));
                });
        }

        public Command setSpeed(double angularVelocity) {
                return this.runOnce(() -> {
                        this.shooterTargetVelocity = angularVelocity;
                        shooterMotor.setControl(
                                        shooterSpeedControl.withVelocity(shooterTargetVelocity / 2.0 / Math.PI));
                });
        }

        public Command setSpeed(Supplier<Double> angularVelocity) {
                return this.runOnce(() -> {
                        this.shooterTargetVelocity = angularVelocity.get();
                        shooterMotor.setControl(
                                        shooterSpeedControl.withVelocity(shooterTargetVelocity / 2.0 / Math.PI));
                });
        }

        public Command idle() {
                return this.runOnce(() -> {
                        this.shooterTargetVelocity = Constants.Shooter.SPEED_AT_IDLE;
                        shooterMotor.setControl(shooterIdleControl.withOutput(Constants.Shooter.IDLE_VOLTAGE));
                });
        }

        public Command off() {
                return this.runOnce(() -> {
                        this.shooterTargetVelocity = 0;
                        shooterMotor.setControl(shooterOffControl);
                });
        }

        @Override
        public void periodic() {
                // all these are in rotations per second
                SmartDashboard.putNumber("Yaw Angle", yawMotor.getPosition(true).getValueAsDouble());
                SmartDashboard.putNumber("Yaw Target Position", this.yawTargetPosition / Math.PI / 2.0);
                SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getVelocity(true).getValueAsDouble());
                SmartDashboard.putNumber("Shooter Target Velocity", this.shooterTargetVelocity / Math.PI / 2.0);
        }

        @Override
        public void simulationPeriodic() {
                updateSimState(0.02);
        }

        public void updateSimState(double deltatime) {
                TalonFXSimState yawMotorSimState = yawMotor.getSimState();
                double yawSimVolts = yawMotorSimState.getMotorVoltage();
                yawMotorSim.setInputVoltage(yawSimVolts);
                yawMotorSim.update(0.02);

                yawMotor.getPosition().refresh();
                yawArmLigament.setAngle(yawMotor.getPosition().getValueAsDouble() * 360);// convert rot to deg

                yawMotorSimState.setRawRotorPosition(yawMotorSim.getAngleRads() / 2.0 / Math.PI);
                yawMotorSimState.setRotorVelocity(yawMotorSim.getVelocityRadPerSec() / 2.0 / Math.PI);

                shooterPos = new Pose3d(0, 0, 0,
                                new Rotation3d(0, 0, yawMotor.getPosition().getValueAsDouble() * 2 * Math.PI));
                publisher1.set(shooterPos);

                shooterMotorSimState = shooterMotor.getSimState();
                double shooterSimVolts = shooterMotorSimState.getMotorVoltage();
                shooterMotorSim.setInputVoltage(shooterSimVolts);

                shooterMotorSim.update(0.02);

                shooterMotorSimState.setRotorVelocity(shooterMotorSim.getAngularVelocityRPM() / 60.0);
                shooterMotorSimState.setRotorAcceleration(
                                shooterMotorSim.getAngularAccelerationRadPerSecSq() / 2.0 / Math.PI);
        }

        public final Trigger yawIsAtPosition = new Trigger(
                        () -> Util.epsilonEquals(this.yawMotor.getPosition(false).getValueAsDouble(),
                                        this.yawTargetPosition));

        public final Trigger shooterIsAtVelocity = new Trigger(
                        () -> Util.epsilonEquals(this.shooterMotor.getVelocity(false).getValueAsDouble(),
                                        this.shooterTargetVelocity));
}