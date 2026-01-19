package org.team9140.frc2026.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class AimAlign {
    private static InterpolatingDoubleTreeMap lookupPitch =  new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap lookupSpeed =  new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap lookupAirTime =  new InterpolatingDoubleTreeMap();

    static {
        lookupPitch.put(Double.valueOf(14.85), Double.valueOf(-0.11));
        lookupPitch.put(Double.valueOf(9.74), Double.valueOf(-0.119));
        
        lookupSpeed.put(Double.valueOf(14.85), Double.valueOf(-0.11));
        lookupSpeed.put(Double.valueOf(9.74), Double.valueOf(-0.119));
        
        lookupAirTime.put(Double.valueOf(14.85), Double.valueOf(-0.11));
        lookupAirTime.put(Double.valueOf(9.74), Double.valueOf(-0.119));
    }

    public static double yawAngleToPosition(Pose2d turretPos, Translation3d endPos) {
        return Math.atan(
            (endPos.getY() - turretPos.getY())/(endPos.getX() - turretPos.getX())
            ) - turretPos.getRotation().getRadians();
    }

    public static double distanceToPosition(Pose2d turretPos, Translation3d endPos) {
        return Math.sqrt(
            Math.pow(turretPos.getX() - endPos.getX(), 2) + 
            Math.pow(turretPos.getY() - endPos.getY(), 2)
            );
    }

    public static double pitchAngleToPosition(Pose2d turretPos, Translation3d endPos) {
        return lookupPitch.get(distanceToPosition(turretPos, endPos));
    }

    public static double speedToPosition(Pose2d turretPos, Translation3d endPos, double robotSpeed) {
        double dist = distanceToPosition(turretPos, endPos);
        do {
            dist -= lookupAirTime.get(dist) * (robotSpeed + lookupSpeed.get(dist));
        } while(dist > 2);
        return lookupSpeed.get(dist);
    }

    
}
