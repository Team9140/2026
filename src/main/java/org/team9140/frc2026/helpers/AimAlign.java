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
        lookupAirtimeFromDistance.put(2.0245, 0.8266666666666666667);
        lookupAirtimeFromDistance.put(2.3213, 1.0);
        lookupAirtimeFromDistance.put(2.66, 1.06333333333333333);
        lookupAirtimeFromDistance.put(2.91, 1.1666666666666667);
        lookupAirtimeFromDistance.put(3.19, 1.29);
        lookupAirtimeFromDistance.put(3.49, 1.41);
        lookupAirtimeFromDistance.put(3.81, 1.4316666666666667);
        lookupAirtimeFromDistance.put(4.17, 1.47333333333333);
        lookupAirtimeFromDistance.put(4.45, 1.6166666666666667);

        lookupMotorSpeedFromDistance.put(2.93, 2000.0 / 60.0);
        lookupMotorSpeedFromDistance.put(3.37, 2000.0 / 60.0);
        lookupMotorSpeedFromDistance.put(3.50, 2000.0 / 60.0);
        lookupMotorSpeedFromDistance.put(3.55, 2100.0 / 60.0);
        lookupMotorSpeedFromDistance.put(3.92, 2100.0 / 60.0);
        lookupMotorSpeedFromDistance.put(4.28, 2200.0 / 60.0);
        lookupMotorSpeedFromDistance.put(4.35, 2200.0 / 60.0);
        lookupMotorSpeedFromDistance.put(4.67, 2250.0 / 60.0);
        lookupMotorSpeedFromDistance.put(6.0, 2500.0 / 60.0);

        lookupHoodAngleFromDistance.put(2.93, 22.5);
        lookupHoodAngleFromDistance.put(3.37, 23.5);
        lookupHoodAngleFromDistance.put(3.50, 25.0);
        lookupHoodAngleFromDistance.put(3.55, 26.0);
        lookupHoodAngleFromDistance.put(3.92, 27.0);
        lookupHoodAngleFromDistance.put(4.28, 28.0);
        lookupHoodAngleFromDistance.put(4.35, 29.0);
        lookupHoodAngleFromDistance.put(4.67, 30.0);
        lookupHoodAngleFromDistance.put(6.0, 45.0);
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
