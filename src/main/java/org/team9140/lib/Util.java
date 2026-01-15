package org.team9140.lib;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;

public class Util {
    private static DriverStation.Alliance alliance = null;

    public static void updateAlliance() {
        alliance = DriverStation.getAlliance().orElse(null);
    }

    public static Optional<DriverStation.Alliance> getAlliance() {
        if (alliance == null) updateAlliance();
        return Optional.ofNullable(alliance);
    }
}
