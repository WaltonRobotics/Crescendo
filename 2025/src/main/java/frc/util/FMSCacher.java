package frc.util;

import edu.wpi.first.wpilibj.DriverStation;

public class FMSCacher {
    private static boolean hasSeenFMSEver = false;
    public static boolean getCachedFMSAttached() {
        if (hasSeenFMSEver) return true;
        else if (DriverStation.isFMSAttached()) {
            hasSeenFMSEver = true;
            return true;
        } else {
            return false;
        }
    }
}   
