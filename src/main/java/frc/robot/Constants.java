package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

// TODO fix everything in this class besides the field constants
public class Constants {
    public static final double kStickDeadband = 0.1;
    public static final String kCanbus = "fd";

    public class FieldK {
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

    public class VisionK {
        public static final Transform3d kCamToRobot = new Transform3d();
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
        public static final int kAimId = 15;
        public static final int kCancoderId = 16;

        public static final double kGearRatio = 200;

        public static final double kS = 0.89;
        public static final double kV = 22.57; // V * s / rot
        public static final double kA = 0.12; // V * s^2 / rot
        public static final double kP = 100; // idk
        public static final double kD = 0;
        public static final double kG = 0.37;
        public static final double kAcceleration = 160;
    }

    public static final class TrapK {
        public static final class TrapElevatorK {
            // TODO: fix
            public static final int kExtendCANID = 21;
            public static final int kLowerLimitSwitchPort = 20;

            public static final double kMaxVelocity = 3.25; // Meters Per Second
            public static final double kMaxAcceleration = 2.75; // Meters Per Second Squared

            public static final double kMaxHeight = 6; // Rotations? from base
            public static final double kMinHeight = 0; // Rotations? from base

            public static final double kP = 22;
            public static final double kS = 0.16114;
            public static final double kG = 0.16114;
            public static final double kV = 0.16114;
            public static final double kA = 0.16114;

            public static final double kPHold = 0.7;
            public static final double kHoldKs = 0.705;
            public static final double kVoltageCompSaturationVolts = 12.0;

            public static final ElevatorFeedforward kFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);
            public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(
                kMaxVelocity, kMaxAcceleration);
        }

        public static final class TrapTiltK {
            public static final String DB_TAB_NAME = "TrapTiltSubsys";

            public static final int kMotorCANID = 22;
            public static final int kAbsoluteEncoderPort = 23;
            public static final int kHomeSwitchPort = 24;
            public static final int kMotorCurrLimit = 20;

            public static final double kMaxVelocity = 3.25; // Meters Per Second
            public static final double kMaxAcceleration = 2.75; // Meters Per Second Squared
            public static final double kMaxVelocityForward = kMaxVelocity * 0.75;
            public static final double kMaxAccelerationForward = kMaxAcceleration * 0.75;

            public static final double kP = 2;
            public static final double kPHold = 0.7;
            public static final double kDHold = 0;
            public static final double kHoldKs = 0.705;
            public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(
                kMaxVelocity,
                kMaxAcceleration);

            public static final double kVoltageCompSaturationVolts = 12.0; // value directly yoinked from chargedup

            public static final double kAbsMaxDegree = 35; // stolen from chargedup, MUST adjust
        }

        public static final class TrapShooterK {
            public static final int kMotorCanId = 23;
        }
    }
}
