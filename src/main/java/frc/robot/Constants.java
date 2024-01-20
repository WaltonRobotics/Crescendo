package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static final double kFieldLength = 16.54; // meters
    public static final double kFieldWidth = 8.21; // meters

    public class Climber {
        // TODO: change these when they've been like actually made
        public static final int kLeftId = 31;
        public static final int kRightId = 32;
        public static final double kMetersPerRotation = 0.3; // TODO: check value
        public static final Measure<Distance> kMaxHeight = Inches.of(56); // TODO: check value
    }

    public class AutoConstants {
        // TODO: check these values they're just copied from bubbles
        public static final double kPX = 3.25; // 8
        public static final double kPY = 3.25;
        public static final double kPTheta = 5.15; // 1
        public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(kPX),
            new PIDConstants(kPTheta),
            5,
            Math.sqrt(Math.pow(10.3125, 12.375)),
            new ReplanningConfig());
    }
}
