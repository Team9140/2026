package org.team9140.frc2026;

import edu.wpi.first.math.util.Units;

import org.team9140.frc2026.generated.TunerConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static class Ports {
        public static final String CANIVORE_NAME = "buzz"; // Buzz Aldrin
        public static final int YAW_MOTOR = 9;
        public static final int PITCH_MOTOR = 98;
        public static final int SHOOTER_MOTOR = 97;
        public static final int INTAKE_SPIN_MOTOR = 96;
        public static final int INTAKE_EXTEND_MOTOR = 95;
        public static final int CLIMBER_MOTOR = 94;
        public static final int HOPPER_SPINNER_MOTOR = 6;
        public static final int HOPPER_OUTAKE_MOTOR = 7;
    }

    public static class Cantdle {
    }

    public static class Drive {
        public static double MAX_TELEOP_VELOCITY = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
        public static double MAX_TELEOP_ROTATION = 0.75 * 2 * Math.PI;

    }

    public static class Shooter {
        public static final double SHOOTER_ACCELERATION = 1;
        public static final double SHOOTER_CRUISE_VELOCITY = 1;

        public static final double SHOOTER_KS = 0;
        public static final double SHOOTER_KV = 0;
        public static final double SHOOTER_KA = 0;
        public static final double SHOOTER_KP = 50.0;
        public static final double SHOOTER_KI = 0;
        public static final double SHOOTER_KD = 9.6;

        public static final double SHOOT_VELOCITY_RATIO = 1;
    }
    public static class Climber {
        public static final double GEAR_RATIO = 25;
        public static final double SPOOL_RADIUS = 1.0;

        public static final double STATOR_CURRENT_LIMIT = 80;
        public static final double SUPPLY_CURRENT_LIMIT = 1.0;

        public static final double FORWARD_SOFT_LIMIT_THRESHOLD = 0;
        public static final double REVERSE_SOFT_LIMIT_THRESHOLD = 0;

        public static final double EXTENSION_VOLTAGE = 12.0;
        public static final double EXTEND_POSITION = 1.0;
    }

    public static class Intake {
        public static final double INTAKE_VOLTAGE = 0.7;
        public static final double INTAKE_OFF = 0;
        public static final double SPIN_STATOR_CURRENT_LIMIT = 0;
        public static final double EXTEND_STATOR_CURRENT_LIMIT = 0;
        public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0;
        public static final double MOTION_MAGIC_ACCELERATION = 0;
        public static final double FORWARD_SOFT_LIMIT_THRESHOLD = 0;
        public static final double REVERSE_SOFT_LIMIT_THRESHOLD = 0;
        public static final double ARM_IN_POSITION = 0;
        public static final double ARM_OUT_POSITION = 0;
        public static final double EXTENSION_GEAR_RATIO = 1;
        public static final double TOLERANCE = 0.5;
        public static final int PINION_TEETH = 10;
        public static final double PINION_DP = 10;
        public static final double PINION_CIRCUMFERENCE = Units.inchesToMeters(PINION_TEETH / PINION_DP * Math.PI);
    }
    public static class Turret {
        public static final double YAW_ACCELERATION = 1;
        public static final double YAW_CRUISE_VELOCITY = 1;

        public static final double YAW_KS = 0;
        public static final double YAW_KV = 0;
        public static final double YAW_KA = 0;
        public static final double YAW_KP = 150;
        public static final double YAW_KI = 0;
        public static final double YAW_KD = 96;

        public static final Pose2d HOOP_POSITION = new Pose2d(4.62534, 4.03479, new Rotation2d());
    }

    public static class Hopper {
        public static final double SPINNER_STATOR_CURRENT_LIMIT = 0;
        public static final double OUTAKE_STATOR_CURRENT_LIMIT = 0;

        public static final double SPINNER_VOLTAGE = 1;
        public static final double OUTAKE_VOLTAGE = 1;
    }

}