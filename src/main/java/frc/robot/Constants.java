package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Constants {
    public static final double kFieldLength = 16.54; // meters
    public static final double kFieldWidth = 8.21; // meters

    public class AutoConstants {
        // TODO: check these values they're just copied from bubbles
        public static final double kPX = 3.25; // 8
        public static final double kPY = 3.25;
        public static final double kPTheta = 5.15; // 1
        public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(kPX),
            new PIDConstants(kPTheta),
            5,
            4.5, // FIXME
            new ReplanningConfig());
    }
}
