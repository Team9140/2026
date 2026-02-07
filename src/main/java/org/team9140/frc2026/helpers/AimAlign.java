package org.team9140.frc2026.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AimAlign {
    private static InterpolatingDoubleTreeMap lookupSpeedFromDistance =  new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap lookupMotorFromSpeed =  new InterpolatingDoubleTreeMap();

    private static double latency = 0;
    private static double pitchAngle = 46;

    static {
        lookupSpeedFromDistance.put(Double.valueOf(14.85), Double.valueOf(-0.11));
        lookupSpeedFromDistance.put(Double.valueOf(9.74), Double.valueOf(-0.119));
       
        lookupMotorFromSpeed.put(Double.valueOf(14.85), Double.valueOf(-0.11));
        lookupMotorFromSpeed.put(Double.valueOf(9.74), Double.valueOf(-0.119));
    }

    public static double getRequiredSpeed(Pose2d robotPose, Translation2d goalPose, ChassisSpeeds robotSpeed) {
        Translation2d futurePos = robotPose.getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
        );
        Translation2d targetTranslation = goalPose.minus(futurePos);
        double idealSpeed = lookupSpeedFromDistance.get(targetTranslation.getNorm());

        Translation2d shotVector = new Translation2d(idealSpeed * Math.cos(pitchAngle), targetTranslation.getAngle());
        shotVector = shotVector.minus(new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond));

        double requiredSpeed = Math.hypot(shotVector.getNorm(), idealSpeed * Math.sin(pitchAngle));

        return lookupMotorFromSpeed.get(requiredSpeed);
    }
   
    public static double yawAngleToPos(Pose2d turretPos, Translation2d endPos) {
        return Math.atan(
            (endPos.getY() - turretPos.getY())/(endPos.getX() - turretPos.getX())
            ) - turretPos.getRotation().getRadians();
    }
}
