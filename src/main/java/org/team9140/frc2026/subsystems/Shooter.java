package org.team9140.frc2026.subsystems;

import java.util.function.Supplier;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase {
        private final TalonFX yawMotor = new TalonFX(Constants.Ports.YAW_MOTOR);
        private final TalonFX shooterMotor = new TalonFX(Constants.Ports.SHOOTER_MOTOR);

        private double yawTargetPosition = 0;

        private MotionMagicTorqueCurrentFOC yawMM = new MotionMagicTorqueCurrentFOC(0);
        private MotionMagicVelocityTorqueCurrentFOC shooterMM = new MotionMagicVelocityTorqueCurrentFOC(0);

        private TalonFXSimState yawMotorSimState;
        private final SingleJointedArmSim yawMotorSim = new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(1),
                        60,
                        1,
                        0.2,
                        -Math.PI,
                        Math.PI,
                        false,
                        0);
        private Mechanism2d yawMech = new Mechanism2d(1, 1);
        private MechanismRoot2d yawRoot = yawMech.getRoot("yawArm Root", 1.5, 0.5);
        private MechanismLigament2d yawArmLigament;
        private double ARM_LENGTH = 1.0;

        private static Shooter instance;

        public static Shooter getInstance(Supplier<Pose2d> rPos, Supplier<ChassisSpeeds> speeds) {
                return (instance == null) ? instance = new Shooter(rPos, speeds) : instance;
        }

        private Supplier<Pose2d> robotPose;
        private Supplier<ChassisSpeeds> robotSpeeds;

        private Shooter(Supplier<Pose2d> rPos, Supplier<ChassisSpeeds> speeds) {
                this.robotPose = rPos;
                this.robotSpeeds= speeds;
                
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

                TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
                                .withMotionMagic(shooterMMConfigs)
                                .withSlot0(shooterSlot0Configs);

                yawMM = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
                shooterMM = new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);

                yawMotor.getConfigurator().apply(yawConfig);
                shooterMotor.getConfigurator().apply(shooterConfig);

                yawMotor.setControl(yawMM.withPosition(0));
                shooterMotor.setControl(shooterMM.withVelocity(0));

                yawArmLigament = yawRoot.append(new MechanismLigament2d(
                                "yawArm",
                                ARM_LENGTH,
                                0,
                                6,
                                new Color8Bit(Color.kYellow)));

                SmartDashboard.putData("YAW ARM MECHANISM", yawMech);

        }

        public Command moveYawToPos(double pos) {
                return this.runOnce(() -> {
                        this.yawTargetPosition = pos % (2*Math.PI) - Math.PI;
                        yawMotor.setControl(yawMM.withPosition(yawTargetPosition / 2.0 / Math.PI));
                }).andThen(new WaitUntilCommand(yawIsAtPosition));
        }

        public final Trigger yawIsAtPosition = new Trigger(
                        () -> Util.epsilonEquals(this.yawMotor.getPosition(false).getValueAsDouble(),
                                        this.yawTargetPosition));

        public Command runShooterMotor(double angularVelocity) {
                return this.runOnce(() -> {
                        shooterMotor.setControl(shooterMM.withVelocity(angularVelocity / 2.0 / Math.PI));
                });
        }

        public Command stopShooterMotor() {
                return this.runOnce(() -> {
                        shooterMotor.setControl(shooterMM.withVelocity(0));
                });
        }

        public Command aimTowardsPos(Translation2d goalPos) {
                return moveYawToPos(AimAlign.yawAngleToPos(this.robotPose.get(), goalPos));
        }

        public Command shootAtTarget(Translation2d goalPos, ChassisSpeeds robotSpeed) {
                return runShooterMotor(AimAlign.getRequiredSpeed(robotPose.get(), goalPos, robotSpeed));
        }

        public Command shootAtGoal() {
                return this.aimTowardsPos(Constants.Turret.HOOP_POSITION.getTranslation()).andThen(
                                this.shootAtTarget(Constants.Turret.HOOP_POSITION.getTranslation(), robotSpeeds.get()));
        }

        @Override
        public void periodic() {
                yawMotorSimState = yawMotor.getSimState();
                double yawSimVolts = yawMotorSimState.getMotorVoltage();
                yawMotorSim.setInputVoltage(yawSimVolts);
                yawMotorSim.update(0.02);

                SmartDashboard.putNumber("Yaw Angle", yawMotor.getPosition().getValueAsDouble()); //yaw motor position in rotations
                SmartDashboard.putNumber("Yaw Target Position", this.yawTargetPosition / Math.PI / 2); //target position in rotations
                yawMotor.getPosition().refresh();
                yawArmLigament.setAngle(yawMotor.getPosition().getValueAsDouble() * 360);// convert rot to deg

                yawMotorSimState.setRawRotorPosition(yawMotorSim.getAngleRads() / 2.0 / Math.PI);
                yawMotorSimState.setRotorVelocity(yawMotorSim.getVelocityRadPerSec() / 2.0 / Math.PI);
        }
}