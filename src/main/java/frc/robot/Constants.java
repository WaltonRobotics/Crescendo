package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

// TODO fix everything in this class besides the field constants
public class Constants {
    public static final double kStickDeadband = 0.1;
    public static final String kCanbus = "fd";

    public class VisionK {
        public static final Transform3d kFrontTagCamLocation = new Transform3d(
            0.5, 0.5, 0.25, new Rotation3d());

        public static final Transform3d kRearTagCamLocation = new Transform3d(
            -0.5, 0.5, 0.25, new Rotation3d(0, 0, Units.degreesToRadians(180)));
    }

    public class FieldK {
        public static final Measure<Distance> kFieldLength = Meters.of(16.54);
        public static final Measure<Distance> kFieldWidth = Meters.of(8.21);

        public static final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        public static final Measure<Distance> kXToSpeaker = Inches.of(46.50);
        public static final Measure<Distance> kZToSpeaker = Inches.of(98.13);
        public static final Translation3d kBlueSpeakerPose = new Translation3d(
            kXToSpeaker.baseUnitMagnitude(), 5.5, kZToSpeaker.baseUnitMagnitude());
        public static final Translation3d kRedSpeakerPose = new Translation3d(
            kFieldLength.magnitude() - kXToSpeaker.baseUnitMagnitude(), kFieldWidth.magnitude() - 5.5,
            kZToSpeaker.baseUnitMagnitude());

        public static final Measure<Distance> kXToAmp = Inches.of(49.5);
        public static final Measure<Distance> kYToAmp = Inches.of(286.765);
        public static final Measure<Distance> kZToAmp = Inches.of(35);
        public static final Translation3d kBlueAmpPose = new Translation3d(
            kXToAmp.baseUnitMagnitude(), kYToAmp.baseUnitMagnitude(), kZToSpeaker.magnitude());
        public static final Translation3d kRedAmpPose = new Translation3d(
            kFieldLength.magnitude() - kXToAmp.baseUnitMagnitude(),
            kFieldWidth.magnitude() - kYToAmp.baseUnitMagnitude(),
            kZToAmp.baseUnitMagnitude());

        public static final int kBlueCenterTrapID = 14;
        public static final int kBlueLeftTrapID = 15;
        public static final int kBlueRightTrapID = 16;

        public static final int kBlueCenterTrapId = 14;
        public static final int kBlueLeftTrapId = 15;
        public static final int kBlueRightTrapId = 16;

        public static final int kRedLeftTrapId = 11;
        public static final int kRedRightTrapId = 12;
        public static final int kRedCenterTrapId = 13;

    }

    public class AutoK {
        public static final double kPX = 3.25; // 8
        public static final double kPY = 3.25;
        public static final double kPTheta = 5.15; // 1
        public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(kPX),
            new PIDConstants(kPTheta),
            5,
            Math.sqrt(Math.pow(Math.pow(10.3125, 2) + Math.pow(12.375, 2), 2)),
            new ReplanningConfig());
    }

    public class ClimberK {
        public static final int kLeftId = 31;
        public static final int kRightId = 32;
        public static final int kLimitSwitchId = 1;
        public static final double kMetersPerRotation = 0.3;
        public static final double kP = 3.25;
        public static final Measure<Distance> kMaxHeight = Inches.of(56);
    }

    public class ShooterK {
        public static final int kRightId = 11;
        public static final int kLeftId = 12;
        public static final int kTiltId = 13;
        public static final int kConveyorId = 14;
    }

    public class AimK {
        public static final int kAimId = 15;
        public static final int kCancoderId = 16;

        public static final double kGearRatio = 200;

        public static final double kS = 0.89;
        public static final double kV = 22.57; // V * s / rot
        public static final double kA = 0.12; // V * s^2 / rot
        public static final double kP = 100; // idk
        public static final double kG = 0.37;
        public static final double kAcceleration = 160;
    }
}
