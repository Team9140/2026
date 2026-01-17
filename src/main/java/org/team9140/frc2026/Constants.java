package org.team9140.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static class Ports {
        public static final String CANIVORE_NAME = "buzz"; // Buzz Aldrin
    }

    public static class Cantdle {}

    public static class Climber {}

    public static class Intake {}

    public static class Shooter {}

    public static class Turret {
        public static final double YAW_ACCELERATION = 1;
        public static final double YAW_CRUISE_VELOCITY = 1;

        public static final double YAW_KS = 0;
        public static final double YAW_KV = 0;
        public static final double YAW_KA = 0;
        public static final double YAW_KP = 150.0;
        public static final double YAW_KI = 0;
        public static final double YAW_KD = 9.6;

        public static final double PITCH_ACCELERATION = 1;
        public static final double PITCH_CRUISE_VELOCITY = 1;

        public static final double PITCH_KS = 0;
        public static final double PITCH_KV = 0;
        public static final double PITCH_KA = 0;
        public static final double PITCH_KP = 150.0;
        public static final double PITCH_KI = 0;
        public static final double PITCH_KD = 9.6;

        public static final Pose2d HOOP_POSITION = new Pose2d(4.62534, 4.03479, new Rotation2d());
    }
}