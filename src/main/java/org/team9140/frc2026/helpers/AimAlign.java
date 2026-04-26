package org.team9140.frc2026.helpers;

import java.util.Optional;

import org.team9140.frc2026.FieldConstants;
import org.team9140.frc2026.SwerveTelemetry;
import org.team9140.frc2026.Constants.Turret;
import org.team9140.lib.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AimAlign {
    private static InterpolatingDoubleTreeMap motorSpeedFromDistanceShooting = new InterpolatingDoubleTreeMap(); //in rotatations per second
    private static InterpolatingDoubleTreeMap hoodAngleFromDistanceShooting = new InterpolatingDoubleTreeMap(); //in degrees
    private static InterpolatingDoubleTreeMap airtimeFromDistanceShooting = new InterpolatingDoubleTreeMap(); //in seconds

    private static InterpolatingDoubleTreeMap motorSpeedFromDistancePassing = new InterpolatingDoubleTreeMap(); // in rotations per second
    private static InterpolatingDoubleTreeMap hoodAngleFromDistancePassing = new InterpolatingDoubleTreeMap(); // in degrees
    private static InterpolatingDoubleTreeMap airtimeFromDistancePassing = new InterpolatingDoubleTreeMap(); //in seconds

    static {
        airtimeFromDistanceShooting.put(1.0, 1.0);
        airtimeFromDistancePassing.put(1.0, 1.0);

        motorSpeedFromDistanceShooting.put(1.68, 1725.0 / 60.0);
        motorSpeedFromDistanceShooting.put(2.31, 1800.0 / 60.0);
        motorSpeedFromDistanceShooting.put(2.92, 2000.0 / 60.0);
        motorSpeedFromDistanceShooting.put(3.73, 2150.0 / 60.0);
        motorSpeedFromDistanceShooting.put(4.04, 2200.0 / 60.0);
        motorSpeedFromDistanceShooting.put(4.34, 2250.0 / 60.0);
        motorSpeedFromDistanceShooting.put(4.67, 2550.0 / 60.0);

        hoodAngleFromDistanceShooting.put(1.68, 20.0);
        hoodAngleFromDistanceShooting.put(2.01, 21.0);
        hoodAngleFromDistanceShooting.put(2.31, 24.0);
        hoodAngleFromDistanceShooting.put(2.45, 25.0);
        hoodAngleFromDistanceShooting.put(2.92, 28.0);
        hoodAngleFromDistanceShooting.put(3.30, 31.5);
        hoodAngleFromDistanceShooting.put(3.73, 33.0);
        hoodAngleFromDistanceShooting.put(4.04, 34.0);
        hoodAngleFromDistanceShooting.put(4.34, 35.0);
        hoodAngleFromDistanceShooting.put(4.67, 38.0);
        hoodAngleFromDistanceShooting.put(4.98, 38.5);

        motorSpeedFromDistancePassing.put(1.0, 3500.0 / 60.0);
        hoodAngleFromDistancePassing.put(1.0, 42.0);
    }

    static StructPublisher<Pose2d> effectivePosePublisher = SwerveTelemetry.getFieldTable().getStructTopic("Effective Pose", Pose2d.struct).publish();

    public static Translation2d getEffectivePose(Pose2d robotPose, Translation2d goalPose, ChassisSpeeds robotSpeed, boolean isPassing) {
        robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeed, robotPose.getRotation());
        Translation2d robotVelocity = new Translation2d(
            Turret.POSITION_TO_ROBOT.getX(),
            Turret.POSITION_TO_ROBOT.getY()
        );
        robotVelocity = robotVelocity.rotateBy(robotPose.getRotation().plus(new Rotation2d(Math.PI/2))).times(-robotSpeed.omegaRadiansPerSecond);
        robotVelocity = robotVelocity.plus(new Translation2d(
                robotSpeed.vxMetersPerSecond,
                robotSpeed.vyMetersPerSecond));
        
        InterpolatingDoubleTreeMap airtimeLookup = isPassing ? airtimeFromDistancePassing : airtimeFromDistanceShooting;

        double distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(goalPose).getNorm();
        double airtime = airtimeLookup.get(distance);
        Translation2d newPose = goalPose.minus(robotVelocity.times(airtime));

        int iterations = 0;
        do {
            iterations++;
            distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(newPose).getNorm();
            airtime = airtimeLookup.get(distance);
            SmartDashboard.putNumber("Estimated Airtime", airtime);
            effectivePosePublisher.set(new Pose2d(newPose, new Rotation2d()));
        } while (newPose.minus(newPose = goalPose.minus(robotVelocity.times(airtime))).getNorm() > 0.05 && iterations < 5);

        SmartDashboard.putNumber("NumIterations", iterations);

        return newPose;
    }

    public static double getRequiredSpeed(Pose2d robotPose, Translation2d effectivePose, boolean isPassing) {
        double distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(effectivePose).getNorm();
        return isPassing ? motorSpeedFromDistancePassing.get(distance) : motorSpeedFromDistanceShooting.get(distance);
    }

    public static double getRequiredHoodAngle(Pose2d robotPose, Translation2d effectivePose, boolean isPassing) {
        double distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(effectivePose).getNorm();
        return isPassing ? hoodAngleFromDistancePassing.get(distance) : hoodAngleFromDistanceShooting.get(distance);
    }

    public static double yawAngleToPos(Pose2d robotPose, Translation2d endPose) {
        endPose = (new Pose2d(endPose, Util.NOROTATION).relativeTo(robotPose)).getTranslation();
        return MathUtil.angleModulus(Math.atan2(
                (endPose.getY() - Turret.POSITION_TO_ROBOT.getY()),
                (endPose.getX() - Turret.POSITION_TO_ROBOT.getX()))
                - Turret.POSITION_TO_ROBOT.getRotation().getRadians());
    }

    public static Pose2d getZone(Pose2d robotPose) {
        double rx = robotPose.getX();
        double ry = robotPose.getY();
        Pose2d position;
        if (Optional.of(DriverStation.Alliance.Red).equals(Util.getAlliance())
                && rx > FieldConstants.Lines.RED_ALLIANCE_ZONE) {
            position = FieldConstants.Hub.RED_CENTER_POINT;
        } else if (Optional.of(DriverStation.Alliance.Blue).equals(Util.getAlliance())
                && rx < FieldConstants.Lines.BLUE_ALLIANCE_ZONE) {
            position = FieldConstants.Hub.CENTER_POINT;
        } else if (Optional.of(DriverStation.Alliance.Red).equals(Util.getAlliance())) {
            if (ry < FieldConstants.FIELD_WIDTH / 2) {
                position = FieldConstants.FeedingPositions.FEEDING_POS_LOWER_RED;
            } else {
                position = FieldConstants.FeedingPositions.FEEDING_POS_UPPER_RED;
            }
        } else {
            if (ry < FieldConstants.FIELD_WIDTH / 2) {
                position = FieldConstants.FeedingPositions.FEEDING_POS_LOWER;
            } else {
                position = FieldConstants.FeedingPositions.FEEDING_POS_UPPER;
            }
        }
        return position;
    }

    public static Pose2d getHub() {
        if (Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Red))) {
            return FieldConstants.Hub.RED_CENTER_POINT;
        }

        return FieldConstants.Hub.CENTER_POINT;
    }
}
