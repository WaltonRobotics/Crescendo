package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public class Field {
        public static final double kFieldLength = 16.54; // meters
        public static final double kFieldWidth = 8.21; // meters

        // i couldn't find the real value i'll do it later
        public static final Measure<Distance> kTagXToSpeaker = Inches.of(18.11);
        public static final Measure<Distance> kTagZToSpeaker = Inches.of(30.25);
        public static final Translation3d kTagToSpeaker = new Translation3d(kTagXToSpeaker.in(Meters), 0,
            kTagZToSpeaker.in(Meters));
    }

    public class Shooter {
        // TODO: change once it's actually built
        public static final int kRightId = 11;
        public static final int kLeftId = 12;
        public static final int kTiltId = 13;
        public static final int kConveyorId = 14;
        public static final int kAimId = 15;
        public static final int kPAim = 3; // TODO: check this value
        public static final double kConversion = 360 / 42;;
    }

    public class Auto {
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
