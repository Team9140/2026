package org.team9140.frc2026.helpers;

import java.util.Optional;

import org.team9140.frc2026.FieldConstants;
import org.team9140.frc2026.Constants.Turret;
import org.team9140.lib.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AimAlign {
    private static InterpolatingDoubleTreeMap lookupMotorSpeedFromDistance = new InterpolatingDoubleTreeMap(); //in rotatations per second
    private static InterpolatingDoubleTreeMap lookupHoodAngleFromDistance = new InterpolatingDoubleTreeMap(); //in rotations
    private static InterpolatingDoubleTreeMap lookupAirtimeFromDistance = new InterpolatingDoubleTreeMap(); //in seconds

    static {
        lookupAirtimeFromDistance.put(1.0, 1.0);

        lookupMotorSpeedFromDistance.put(1.84, 1600.0 / 60.0);
        lookupMotorSpeedFromDistance.put(2.02, 1700.0 / 60.0);
        lookupMotorSpeedFromDistance.put(2.63, 1800.0 / 60.0);
        lookupMotorSpeedFromDistance.put(2.75, 1800.0 / 60.0);
        lookupMotorSpeedFromDistance.put(2.86, 1900.0 / 60.0);
        lookupMotorSpeedFromDistance.put(3.29, 2000.0 / 60.0);
        lookupMotorSpeedFromDistance.put(3.48, 2000.0 / 60.0);
        lookupMotorSpeedFromDistance.put(4.32, 2200.0 / 60.0);
        lookupMotorSpeedFromDistance.put(5.10, 2350.0 / 60.0);
        lookupMotorSpeedFromDistance.put(6.50, 2500.0 / 60.0);
        // lookupMotorSpeedFromDistance.put(5.0, 2200.0 / 60.0);
        lookupMotorSpeedFromDistance.put(7.0, 3000.0 / 60.0);
        // lookupMotorSpeedFromDistance.put(8.0, 6000.0 / 60.0);

        lookupHoodAngleFromDistance.put(2.02, 20.0);
        lookupHoodAngleFromDistance.put(2.63, 21.0);
        lookupHoodAngleFromDistance.put(2.75, 23.0);
        lookupHoodAngleFromDistance.put(2.86, 24.0);
        lookupHoodAngleFromDistance.put(3.29, 26.0); 
        lookupHoodAngleFromDistance.put(3.48, 27.0);
        lookupHoodAngleFromDistance.put(3.84, 28.0);
        lookupHoodAngleFromDistance.put(4.32, 29.0);
        lookupHoodAngleFromDistance.put(5.10, 35.0);
        // lookupHoodAngleFromDistance.put(5.0, 23.5);
        lookupHoodAngleFromDistance.put(6.50, 38.0);
        lookupHoodAngleFromDistance.put(7.0, 42.0);
        // lookupHoodAngleFromDistance.put(8.0, 47.0);
    }

    static StructPublisher<Pose2d> effectivePosePublisher = NetworkTableInstance.getDefault().getStructTopic("Effective Pose", Pose2d.struct).publish();

    public static Translation2d getEffectivePose(Pose2d robotPose, Translation2d goalPose, ChassisSpeeds robotSpeed) {
        robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeed, robotPose.getRotation());
        Translation2d robotVelocity = new Translation2d(
            Turret.POSITION_TO_ROBOT.getX(),
            Turret.POSITION_TO_ROBOT.getY()
        );
        robotVelocity = robotVelocity.rotateBy(robotPose.getRotation().plus(new Rotation2d(Math.PI/2))).times(-robotSpeed.omegaRadiansPerSecond);
        robotVelocity = robotVelocity.plus(new Translation2d(
                robotSpeed.vxMetersPerSecond,
                robotSpeed.vyMetersPerSecond));

        double distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(goalPose).getNorm(); 
        double airtime = lookupAirtimeFromDistance.get(distance);
        Translation2d newPose = goalPose.minus(robotVelocity.times(airtime));

        int iterations = 0;
        do {
            iterations++;
            distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(newPose).getNorm();
            airtime = lookupAirtimeFromDistance.get(distance);
            SmartDashboard.putNumber("Estimated Airtime", airtime);
            effectivePosePublisher.set(new Pose2d(newPose, new Rotation2d()));
        } while (newPose.minus(newPose = goalPose.minus(robotVelocity.times(airtime))).getNorm() > 0.05 && iterations < 5);

        SmartDashboard.putNumber("NumIterations", iterations);

        return newPose;
    }

    public static double getRequiredSpeed(Pose2d robotPose, Translation2d effectivePose) {
        double distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(effectivePose).getNorm();
        return lookupMotorSpeedFromDistance.get(distance);
    }

    public static double getRequiredHoodAngle(Pose2d robotPose, Translation2d effectivePose) {
        double distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(effectivePose).getNorm();
        return lookupHoodAngleFromDistance.get(distance);
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
