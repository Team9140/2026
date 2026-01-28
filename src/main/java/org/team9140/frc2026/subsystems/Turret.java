package org.team9140.frc2026.subsystems;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.helpers.AimAlign;
import org.team9140.lib.Util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Turret extends SubsystemBase {
        private final TalonFX yawMotor = new TalonFX(Constants.Ports.YAW_MOTOR);
        private final TalonFX pitchMotor = new TalonFX(Constants.Ports.PITCH_MOTOR);
        private final TalonFX shooterMotor = new TalonFX(Constants.Ports.SHOOTER_MOTOR);

        private double yawTargetPosition;
        private double pitchTargetPosition;

        private MotionMagicTorqueCurrentFOC yawMM = new MotionMagicTorqueCurrentFOC(0);
        private MotionMagicTorqueCurrentFOC pitchMM = new MotionMagicTorqueCurrentFOC(0);
        private MotionMagicVelocityTorqueCurrentFOC shooterMM = new MotionMagicVelocityTorqueCurrentFOC(0);

        private TalonFXSimState yawMotorSimState;
        private final SingleJointedArmSim yawMotorSim = new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(1),
                        60,
                        1,
                        0.2,
                        0,
                        Math.PI,
                        false,
                        0);
        Mechanism2d yawMech = new Mechanism2d(1, 1);
        MechanismRoot2d yawRoot = yawMech.getRoot("yawArm Root", 1.5, 0.5);
        MechanismLigament2d yawArmLigament;
        double ARM_LENGTH = 1.0;

        private TalonFXSimState pitchMotorSimState;
        private final SingleJointedArmSim pitchMotorSim = new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(1),
                        60,
                        1,
                        0.2,
                        0,
                        Math.PI,
                        false,
                        0);

        private static Turret instance;

        public static Turret getInstance() {
                return (instance == null) ? instance = new Turret() : instance;
        }

        private Turret() {
                MotionMagicConfigs yawMMConfigs = new MotionMagicConfigs()
                                .withMotionMagicAcceleration(Constants.Turret.YAW_ACCELERATION)
                                .withMotionMagicCruiseVelocity(Constants.Turret.YAW_CRUISE_VELOCITY);

                Slot0Configs yawSlot0Configs = new Slot0Configs()
                                .withKS(Constants.Turret.YAW_KS)
                                .withKV(Constants.Turret.YAW_KV)
                                .withKA(Constants.Turret.YAW_KA)
                                .withKP(Constants.Turret.YAW_KP)
                                .withKI(Constants.Turret.YAW_KI)
                                .withKD(Constants.Turret.YAW_KD);

                MotionMagicConfigs pitchMMConfigs = new MotionMagicConfigs()
                                .withMotionMagicAcceleration(Constants.Turret.PITCH_ACCELERATION)
                                .withMotionMagicCruiseVelocity(Constants.Turret.PITCH_CRUISE_VELOCITY);

                Slot0Configs pitchSlot0Configs = new Slot0Configs()
                                .withKS(Constants.Turret.PITCH_KS)
                                .withKV(Constants.Turret.PITCH_KV)
                                .withKA(Constants.Turret.PITCH_KA)
                                .withKP(Constants.Turret.PITCH_KP)
                                .withKI(Constants.Turret.PITCH_KI)
                                .withKD(Constants.Turret.PITCH_KD);

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

                TalonFXConfiguration yawConfig = new TalonFXConfiguration()
                                .withMotionMagic(yawMMConfigs)
                                .withSlot0(yawSlot0Configs);

                TalonFXConfiguration pitchConfig = new TalonFXConfiguration()
                                .withMotionMagic(pitchMMConfigs)
                                .withSlot0(pitchSlot0Configs);

                TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
                                .withMotionMagic(shooterMMConfigs)
                                .withSlot0(shooterSlot0Configs);

                yawMM = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
                pitchMM = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
                shooterMM = new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);

                yawMotor.getConfigurator().apply(yawConfig);
                pitchMotor.getConfigurator().apply(pitchConfig);
                shooterMotor.getConfigurator().apply(shooterConfig);

                yawMotor.setControl(yawMM.withPosition(0));
                pitchMotor.setControl(pitchMM.withPosition(0));
                shooterMotor.setControl(shooterMM.withVelocity(0));

                yawArmLigament = yawRoot.append(new MechanismLigament2d(
                                "yawArm",
                                ARM_LENGTH,
                                0,
                                6,
                                new Color8Bit(Color.kYellow)));

                SmartDashboard.putData("YAW ARM MECHANISM", yawMech);

        }

        public Command moveYawToPosition(double pos) {
                return this.runOnce(() -> {
                        this.yawTargetPosition = pos;
                        yawMotor.setControl(yawMM.withPosition(yawTargetPosition / 2.0 / Math.PI));
                }).andThen(new WaitUntilCommand(yawIsAtPosition));
        }

        public Command movePitchToPosition(double pos) {
                return this.runOnce(() -> {
                        this.pitchTargetPosition = pos;
                        pitchMotor.setControl(pitchMM.withPosition(pitchTargetPosition / 2.0 / Math.PI));
                }).andThen(new WaitUntilCommand(pitchIsAtPosition));
        }

        public Command aimAtPosition(Pose2d turretPos, Translation3d endPos) {
                return this.runOnce(() -> moveYawToPosition(AimAlign.yawAngleToPosition(turretPos, endPos))
                                .andThen(movePitchToPosition(AimAlign.pitchAngleToPosition(turretPos, endPos))));
        }

        public final Trigger yawIsAtPosition = new Trigger(
                        () -> Util.epsilonEquals(this.yawMotor.getPosition(false).getValueAsDouble(),
                                        this.yawTargetPosition));

        public final Trigger pitchIsAtPosition = new Trigger(
                        () -> Util.epsilonEquals(this.pitchMotor.getPosition(false).getValueAsDouble(),
                                        this.pitchTargetPosition));

        public Command setShooterReleaseSpeed(double releaseSpeed) {
                return this.runOnce(() -> {
                        shooterMotor.setControl(shooterMM.withVelocity(
                                        releaseSpeed * Constants.Shooter.SHOOT_VELOCITY_RATIO / 2.0 / Math.PI));
                });
        }

        public Command shoot(double velocity, double time) {
                return this.runOnce(() -> {
                        setShooterReleaseSpeed(velocity)
                                        .andThen(Commands.waitSeconds(time).andThen(setShooterReleaseSpeed(0)));
                });
        }

        public Command shootAtEndPosistion(Pose2d turretPos, Translation3d endPos, double time, double robotSpeed) {
                return this.runOnce(() -> Turret.getInstance().aimAtPosition(turretPos, endPos)
                                .andThen(shoot(AimAlign.speedToPosition(turretPos, endPos, robotSpeed), time)));
        }

        @Override
        public void periodic() {
                yawMotorSimState = yawMotor.getSimState();
                double yawSimVolts = yawMotorSimState.getMotorVoltage();
                yawMotorSim.setInputVoltage(yawSimVolts);
                yawMotorSim.update(0.02);

                pitchMotorSimState = pitchMotor.getSimState();
                double pitchSimVolts = pitchMotorSimState.getMotorVoltage();
                pitchMotorSim.setInputVoltage(pitchSimVolts);
                pitchMotorSim.update(0.02);

                SmartDashboard.putNumber("yawAngle", yawMotor.getPosition().getValueAsDouble());
                yawMotor.getPosition().refresh();
                yawArmLigament.setAngle(yawMotor.getPosition().getValueAsDouble() * 360);// convert rot to deg

                SmartDashboard.putNumber("pitchAngle", pitchMotor.getPosition().getValueAsDouble());
                pitchMotor.getPosition().refresh();

                yawMotorSimState.setRawRotorPosition(yawMotorSim.getAngleRads() / 2.0 / Math.PI);
                yawMotorSimState.setRotorVelocity(yawMotorSim.getVelocityRadPerSec() / 2.0 / Math.PI);

                pitchMotorSimState.setRawRotorPosition(pitchMotorSim.getAngleRads() / 2.0 / Math.PI);
                pitchMotorSimState.setRotorVelocity(pitchMotorSim.getVelocityRadPerSec() / 2.0 / Math.PI);
        }
}