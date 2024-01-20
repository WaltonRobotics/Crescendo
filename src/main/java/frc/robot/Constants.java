package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class Constants {
    public class Field {
        public static final Measure<Distance> kFieldLength = Meters.of(16.54);
        public static final Measure<Distance> kFieldWidth = Meters.of(8.21);

        public static final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        public static final Measure<Distance> kXToSpeaker = Inches.of(46.50);
        public static final Measure<Distance> kZToSpeaker = Inches.of(98.13);
        public static final Translation3d kBlueSpeakerPose = new Translation3d(
            -kXToSpeaker.baseUnitMagnitude(), 5.5, kZToSpeaker.baseUnitMagnitude());
        public static final Translation3d kRedSpeakerPose = new Translation3d(
            kFieldLength.magnitude() + kXToSpeaker.baseUnitMagnitude(), kFieldWidth.magnitude() - 5.5,
            kZToSpeaker.baseUnitMagnitude());
    }

    public class Vision {
        public static final Transform3d kCamToRobot = new Transform3d(); // FIXME
    }

    public class Shooter {
        // FIXME once it's actually built
        public static final int kRightId = 11;
        public static final int kLeftId = 12;
        public static final int kTiltId = 13;
        public static final int kConveyorId = 14;
        public static final int kAimId = 15;
        public static final int kPAim = 3;
        public static final double kConversion = 360 / 42;
    }

    public class Auto {
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