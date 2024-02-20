package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.util.AllianceFlipUtil;

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

        // taken from 6328. All in blue alliance origin.
        /* speaker constants */
        public static final class SpeakerK {
            public static final int kRedSpeakerRightId = 3;
            public static final int kRedSpeakerId = 4;
            public static final int kBlueSpeakerRightId = 8;
            public static final int kBlueSpeakerId = 7;

            private static final Measure<Distance> kTopX = Inches.of(18.055);
            private static final Measure<Distance> kTopZ = Inches.of(83.091);
            public static final Translation3d kTopRight = new Translation3d(
                kTopX, Inches.of(238.815), kTopZ);
            public static final Translation3d kTopLeft = new Translation3d(
                kTopX, Inches.of(197.765), kTopZ);

            private static final Measure<Distance> kBotX = Inches.of(0);
            private static final Measure<Distance> kBotZ = Inches.of(78.324);
            // private static final Translation3d kBotRight = new Translation3d(
            // kBotX, Inches.of(238.815), kBotZ);
            public static final Translation3d kBotLeft = new Translation3d(
                kBotX, Inches.of(197.765), kBotZ);

            public static final Translation3d kBlueCenterOpening = kBotLeft.interpolate(kTopRight, 0.5);
            public static final Pose3d kBlueCenterOpeningPose3d = new Pose3d(
                kBlueCenterOpening, new Rotation3d());
            public static final Translation3d kRedCenterOpening = AllianceFlipUtil.flip(kBlueCenterOpening);
            public static final Pose3d kRedCenterOpeningPose3d = new Pose3d(
                kRedCenterOpening, new Rotation3d());

            public static final Measure<Distance> kAimOffset = Inches.of(25);
        }

        /* amp constants */
        public static final Measure<Distance> kXToAmp = Inches.of(49.5);
        public static final Measure<Distance> kYToAmp = Inches.of(286.765);
        public static final Measure<Distance> kZToAmp = Inches.of(35);
        public static final Measure<Distance> kZToSpeaker = Inches.of(98.13);

        public static final Translation3d kBlueAmpPose = new Translation3d(
            kXToAmp, kYToAmp, kZToSpeaker);

        public static final Translation3d kRedAmpPose = new Translation3d(
            kFieldLength.minus(kXToAmp), kFieldWidth.minus(kYToAmp), kZToAmp);

        /* stage constants */
        public static final double kBlueStageClearanceDs = Units.inchesToMeters(188.5);
        public static final double kBlueStageClearanceRight = Units.inchesToMeters(88.3);
        public static final double kBlueStageClearanceCenter = Units.inchesToMeters(243.2);
        public static final double kBlueStageClearanceLeft = Units.inchesToMeters(234.9);

        public static final double kRedStageClearanceDs = Units.inchesToMeters(542.2);
        public static final double kRedStageClearanceRight = Units.inchesToMeters(88.3);
        public static final double kRedStageClearanceCenter = Units.inchesToMeters(407.9);
        public static final double kRedStageClearanceLeft = Units.inchesToMeters(234.9);
    }

    public class AutoK {
        public static final double kPX = 3.25; // 8
        public static final double kPY = 3.25;
        public static final double kPTheta = 5.15; // 1
        public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(kPX),
            new PIDConstants(kPTheta),
            5,
            Math.hypot(10.3125, 12.375),
            new ReplanningConfig());
    }

    public class IntakeK {
        public static final int kIntakeId = 10;
        public static final int kFeederId = 11;
    }

    public class ConveyorK {
        public static final int kConveyorId = 12;
    }

    public class ShooterK {
        public static final int kRightId = 13;
        public static final int kLeftId = 14;

        public static final class FlywheelSimK {
            public static final double kGearRatio = 200; // TODO: find actual value

            // TODO: find the real value
            public static final double kInterval = 0.020;

            public static final double kMoi = 0.056699046875; // kg meters^2

            // basically just how much faster the wheels have to spin for 1 meter more of
            // distance idk how this will work
            public static final double kRpmFactor = 100; // TODO fix this i just chose a random value
        }
    }

    public class AimK {
        public static final int kAimId = 15;
        public static final int kHomeSwitch = 0; // TODO check

        public static final double kGearRatio = 200;
        public static final Measure<Distance> kLength = Inches.of(19.75);
        public static final Measure<Angle> kStageClearance = Degrees.of(47.097); // asin((22 -
        // kHeightTilShooter)/kLength)
        public static final Measure<Angle> kMinAngle = Degrees.of(0);
        public static final Measure<Angle> kMaxAngle = Degrees.of(150);

        public static final double kS = 0.89;
        public static final double kV = 22.57; // V * s / rot
        public static final double kA = 0.12; // V * s^2 / rot
        public static final double kP = 100; // idk
        public static final double kG = 0.37;
        public static final double kAcceleration = 160;

    }

    public class ClimberK {
        public static final int kLeftId = 20;
        public static final int kRightId = 21;
        public static final int kLimitSwitchId = 1;

        public static final Measure<Distance> kMetersPerRotation = Meters.of(0.3);
        public static final Measure<Distance> kMaxHeight = Inches.of(56);

        public static final double kP = 3.25;
    }

    public class RobotK {
        public static final double kHeightTilShooter = 7.533; // in inches ! height until the pivot point of shooter
        public static final boolean kTestMode = false;
    }
}
