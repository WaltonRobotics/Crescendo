package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
    public static boolean kDebugLoggingEnabled = true;

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

        public static final Translation3d kBlueAmpPose = new Translation3d(
            kXToAmp, kYToAmp, kZToAmp);

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
        public static final int kVisiSightId = 2;
    }

    public class ConveyorK {
        public static final String kDbTabName = "Conveyor";

        public static final int kConveyorId = 12;
    }

    public class ShooterK {
        public static final class ShooterConfigs {
            public final TalonFXConfiguration kRightConfigs = new TalonFXConfiguration();
            public final TalonFXConfiguration kLeftConfigs = new TalonFXConfiguration();

            public ShooterConfigs() {
                kRightConfigs.Feedback = kRightConfigs.Feedback
                    .withSensorToMechanismRatio(kGearRatio);

                kRightConfigs.Slot0 = kRightConfigs.Slot0
                    .withKP(kPRight)
                    .withKS(kSRight)
                    .withKV(kVRight)
                    .withKA(kARight);

                kLeftConfigs.Feedback = kLeftConfigs.Feedback
                    .withSensorToMechanismRatio(kGearRatio);

                kLeftConfigs.Slot0 = kLeftConfigs.Slot0
                    .withKP(kPLeft)
                    .withKS(kSLeft)
                    .withKV(kVLeft)
                    .withKA(kALeft);
            }
        }

        public static final String kDbTabName = "Shooter";

        public static final int kRightId = 13;
        public static final int kLeftId = 14;

        public static final double kPRight = 0.01776;
        public static final double kSRight = 0.25; // sysid 0.15837
        public static final double kVRight = 0.057376;
        public static final double kARight = 0.007685;

        public static final double kPLeft = 0.0475; // sysid 0.046968
        public static final double kSLeft = 0.257; // sysid 0.2057
        public static final double kVLeft = 0.0553; // sysid 0.052935
        public static final double kALeft = 0.017803;

        public static final double kSpinAmt = 0.7;

        public static final double kGearRatio = 18.0 / 36.0;

        public static final class FlywheelSimK {
            public static final double kMoi = 0.056699046875; // kg m^2

            // basically just how much faster the wheels have to spin for 1 meter more of
            // distance idk how this will work
            public static final double kRpmFactor = 100; // TODO fix this i just chose a random value
        }
    }

    public class AimK {
        public static final class AimConfigs {
            public final TalonFXConfiguration kAimConfigs = new TalonFXConfiguration();

            public AimConfigs() {
                kAimConfigs.Slot0 = kAimConfigs.Slot0.withKP(kP);

                kAimConfigs.Feedback = kAimConfigs.Feedback
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kGearRatio);

                kAimConfigs.MotorOutput = kAimConfigs.MotorOutput
                    .withPeakForwardDutyCycle(0.25)
                    .withPeakReverseDutyCycle(0.25);

                kAimConfigs.CurrentLimits = kAimConfigs.CurrentLimits
                    .withStatorCurrentLimit(15)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(8)
                    .withSupplyCurrentLimitEnable(true);

                kAimConfigs.MotionMagic = kAimConfigs.MotionMagic
                    .withMotionMagicCruiseVelocity(0)
                    .withMotionMagicExpo_kV(kV)
                    .withMotionMagicExpo_kA(kA);
            }
        }

        public final String kDbTabName = "Aim";

        public static final int kAimId = 15;
        public static final int kHomeSwitch = 1;

        // 125:1 MaxPlanetary, 24:60 belt drive, 312.5:1 total
        public static final double kGearRatio = (125 * (60 / 24));

        public static final Measure<Distance> kLength = Inches.of(19.75);
        // asin((22 - kHeightTilShooter) / kLength)
        public static final Measure<Angle> kStageClearance = Degrees.of(47.097);
        public static final Measure<Angle> kMinAngle = Degrees.of(22.5);
        public static final Measure<Angle> kInitAngle = Degrees.of(90);
        public static final Measure<Angle> kMaxAngle = Degrees.of(150);

        public static final double kS = 0.89;
        public static final double kV = 22.57; // V * s / rot
        public static final double kA = 0.12; // V * s^2 / rot
        public static final double kP = 30; // idk
        public static final double kG = 0.37;
        public static final double kAcceleration = 160;
    }

    // public class ClimberK {
    // public static final int kLeftId = 20;
    // public static final int kRightId = 21;
    // public static final int kLimitSwitchId = 2;

    // public static final Measure<Distance> kMetersPerRotation = Meters.of(0.3);
    // public static final Measure<Distance> kMaxHeight = Inches.of(56);

    // public static final double kP = 3.25;
    // }

    public class RobotK {
        public static final Measure<Distance> kHeightTilShooter = Inches.of(7.533);
        public static final boolean kTestMode = false;
        public static final double kSimInterval = 0.020;
    }
}
