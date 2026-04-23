package org.team9140.frc2026.commands;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.Robot;
import org.team9140.frc2026.helpers.AimAlign;
import org.team9140.frc2026.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.Hopper;
import org.team9140.frc2026.subsystems.Intake;
import org.team9140.frc2026.subsystems.Shooter;
import org.team9140.lib.FollowPath;
import org.team9140.lib.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutonomousRoutines {
    private final CommandSwerveDrivetrain drivetrain;

    private final Shooter shooter = Shooter.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final Intake intake = Intake.getInstance();

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private static AutonomousRoutines instance;

    public static AutonomousRoutines getInstance(CommandSwerveDrivetrain drivetrain) {
        return (instance == null) ? instance = new AutonomousRoutines(drivetrain) : instance;
    }

    private AutonomousRoutines(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        autoChooser.setDefaultOption("Do Nothing", "nothing");
        autoChooser.addOption("Shoot Preload", "preload");
        autoChooser.addOption("Score from Depot starting Middle", "score_from_depot");
        autoChooser.addOption("Score from Depot starting Left", "depot_shoot_left");
        autoChooser.addOption("1 Pass From Left", "one_pass_depot");
        autoChooser.addOption("1 Pass From Left then Score from Depot", "one_pass_depot_then_depot_shoot");
        autoChooser.addOption("1 Pass From Right", "one_pass_outpost");
        autoChooser.addOption("2 Passes from Right", "two_pass_outpost");
        autoChooser.addOption("2 Passes from Left", "two_pass_depot");

        autoChooser.addOption("1 Pass Over Bump From Left", "one_pass_depot_bump");
        autoChooser.addOption("1 Pass Over Bump From Right", "one_pass_outpost_bump");
        autoChooser.addOption("2 Passes Over Bump from Right", "two_pass_outpost_bump");
        autoChooser.addOption("2 Passes Over Bump from Left", "two_pass_depot_bump");

        autoChooser.addOption("2 Passes Over Bump from Right (Move and Shoot)", "two_pass_outpost_bump_move");
        autoChooser.addOption("2 Passes Over Bump from Left (Move and Shoot)", "two_pass_depot_bump_move");
        SmartDashboard.putData(autoChooser);
    }

    private Command getShootCommand() {
        return shooter.aim(this.drivetrain::getCachedState, () -> AimAlign.getHub().getTranslation())
                .alongWith(new WaitUntilCommand(shooter.readyToShoot)
                        .andThen(new WaitCommand(1.0))
                        .andThen(hopper.feed().alongWith(
                            new WaitCommand(2.0).andThen(intake.squeeze()))));
    }

    private DriverStation.Alliance lastAlliance = Alliance.Red;
    private String lastFetchedAuto = "";

    public Command getCommand() {
        if (Util.getAlliance().isPresent() && (!Util.getAlliance().get().equals(lastAlliance)
                || (lastFetchedAuto != (lastFetchedAuto = autoChooser.getSelected())))) {
            lastAlliance = Util.getAlliance().get();
            switch (lastFetchedAuto) {
                case "preload":
                    return this.intake.armOut().andThen(new WaitCommand(2.0)).andThen(getShootCommand());
                case "depot_shoot_left":
                    return intake.intake()
                            .alongWith(runChoreoAuto("depotShoot_Left", true, true,
                                    Constants.Shooter.AUTO_IDLE_TIMESTAMP, shooter.idle()).andThen(drivetrain.stop())
                                    .andThen(this.getShootCommand().asProxy()));
                case "score_from_depot":
                    return runChoreoAuto("depotShoot", false, true, 0.0, Commands.none()).alongWith(this.intake.intake()).withTimeout(6.0)
                            .andThen(this.drivetrain.stop())
                            .andThen(new WaitCommand(0.5))
                            .andThen(this.getShootCommand());
                case "one_pass_outpost":
                    return intake.intake()
                            .alongWith(runChoreoAuto("Trench_Outpost_Deep", true, true,
                                    Constants.Shooter.AUTO_IDLE_TIMESTAMP, shooter.idle()).andThen(drivetrain.stop())
                                    .andThen(this.getShootCommand().asProxy()));
                case "one_pass_depot":
                    return intake.intake()
                            .alongWith(runChoreoAuto("Trench_Depot_Deep", true, true,
                                    Constants.Shooter.AUTO_IDLE_TIMESTAMP, shooter.idle()).andThen(drivetrain.stop())
                                    .andThen(this.getShootCommand().asProxy()));
                case "two_pass_outpost":
                    return runRepeat("Trench_Outpost_Deep", "Trench_Reset_Outpost", "Trench_Outpost_Shallow");
                case "two_pass_depot":
                    return runRepeat("Trench_Depot_Deep", "Trench_Reset_Depot", "Trench_Depot_Shallow");
                case "one_pass_outpost_bump":
                    return intake.intake()
                            .alongWith(runChoreoAuto("Bump_Outpost_Deep", true, true,
                                    Constants.Shooter.AUTO_IDLE_TIMESTAMP, shooter.idle()).andThen(drivetrain.stop())
                                    .andThen(this.getShootCommand().asProxy()));
                case "one_pass_depot_bump":
                    return intake.intake()
                            .alongWith(runChoreoAuto("Bump_Depot_Deep", true, true,
                                    Constants.Shooter.AUTO_IDLE_TIMESTAMP, shooter.idle()).andThen(drivetrain.stop())
                                    .andThen(this.getShootCommand().asProxy()));
                case "two_pass_outpost_bump":
                    return runRepeat("Bump_Outpost_Deep", "Bump_Reset_Outpost", "Bump_Outpost_Shallow");
                case "two_pass_depot_bump":
                    return runRepeat("Bump_Depot_Deep", "Bump_Reset_Depot", "Bump_Depot_Shallow");
                case "two_pass_outpost_bump_move":
                    return runRepeatMoveAndShoot("Bump_Outpost_Deep", "Bump_Reset_Outpost_Shoot",
                            "Bump_Outpost_Shallow");
                case "two_pass_depot_bump_move":
                    return runRepeatMoveAndShoot("Bump_Depot_Deep", "Bump_Reset_Depot_Shoot", "Bump_Depot_Shallow");
                case "one_pass_depot_then_depot_shoot":
                    return intake.intake()
                            .alongWith(runChoreoAuto("Trench_Depot_Deep", false, true,
                                    Constants.Shooter.AUTO_IDLE_TIMESTAMP, shooter.idle())
                                    .andThen(this.getShootCommand().withTimeout(5).asProxy())
                                    .andThen(runChoreoAuto("depotShoot_Left", true, false,
                                            Constants.Shooter.AUTO_IDLE_TIMESTAMP, shooter.idle()))
                                    .andThen(this.getShootCommand().asProxy()));
                default:
                    return doNothing();
            }
        }

        return null;
    }

    public Command doNothing() {
        return new PrintCommand("Doing Nothing");
    }

    private final StructPublisher<Pose2d> initialPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Auto Path Initial Pose", Pose2d.struct).publish();

    public Command runChoreoAuto(String pathame, boolean waitUntilAtFinalTarget, boolean reset, double timeBeforeEnd,
            Command pathCommand) {
        FollowPath path = new FollowPath(pathame, () -> this.drivetrain.getState().Pose,
                this.drivetrain::followSample, Util.getAlliance().get(), drivetrain);
        if (Robot.isSimulation() && reset)
            drivetrain.resetPose(path.getInitialPose());
        if (pathCommand != null) {
            path.atTimeFromEnd(timeBeforeEnd).onTrue(pathCommand);
        }
        initialPosePublisher.set(path.getInitialPose());
        return path.gimmeCommand(waitUntilAtFinalTarget);
    }

    public Command runChoreoAuto(String pathame) {
        return this.runChoreoAuto(pathame, true, true, 0, null);
    }

    public Command runRepeat(String pathname1, String pathname2, String pathname3) {
        return intake.intake()
                .alongWith(runChoreoAuto(pathname1, false, true, Constants.Shooter.AUTO_IDLE_TIMESTAMP, shooter.idle())
                        .andThen(drivetrain.stop())
                        .andThen(this.getShootCommand().withTimeout(5).asProxy())
                        .andThen(runChoreoAuto(pathname2, false, false, 0, null))
                        .andThen(runChoreoAuto(pathname3, true, false, Constants.Shooter.AUTO_IDLE_TIMESTAMP,
                                shooter.idle()))
                        .andThen(drivetrain.stop())
                        .andThen(this.getShootCommand().asProxy()));
    }

    public Command runRepeatMoveAndShoot(String pathname1, String pathname2, String pathname3) {
        return intake.intake()
                .alongWith(runChoreoAuto(pathname1, false, true, Constants.Shooter.AUTO_IDLE_TIMESTAMP, shooter.idle())
                        .andThen(runChoreoAuto(pathname2, false, false, 0, null)
                                .alongWith(this.getShootCommand().withTimeout(5).asProxy()))
                        .andThen(runChoreoAuto(pathname3, true, false, Constants.Shooter.AUTO_IDLE_TIMESTAMP,
                                shooter.idle()))
                        .andThen(drivetrain.stop())
                        .andThen(this.getShootCommand().asProxy()));
    }
}