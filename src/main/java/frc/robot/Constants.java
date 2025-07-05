package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class Constants {
    public static boolean kDebugLoggingEnabled = true;
    public static final double kMetersPerInch = 0.0254;

    public static final double kStickDeadband = 0.1;
    public static final String kCanbus = "fd";

    public static final Velocity<Angle> RotationsPerMinute = Rotations.per(Minute);

    public static final class DriveK {
        public static final double kDutyCycleOpenLoopRamp = 0.05; // seconds
    }

    public class FieldK {
        public static final Measure<Distance> kFieldLength = Meters.of(16.54);
        public static final Measure<Distance> kFieldWidth = Meters.of(8.21);

        public static boolean inField(Pose3d pose) {
            return (pose.getX() > 0
                && pose.getX() < kFieldLength.in(Meters)
                && pose.getY() > 0
                && pose.getY() < kFieldWidth.in(Meters));
          }

        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        private static final List<AprilTag> kSpeakerTags =
            List.of(kTagLayout.getTags().get(2), kTagLayout.getTags().get(3), kTagLayout.getTags().get(6), kTagLayout.getTags().get(7));
        public static final AprilTagFieldLayout kTagLayout_SpeakerOnly = new AprilTagFieldLayout(
            kSpeakerTags, kTagLayout.getFieldLength(), kTagLayout.getFieldWidth());
        public static final Pose3d kTag4Pose = kTagLayout.getTagPose(4).get();
        public static final Pose3d kTag7Pose = kTagLayout.getTagPose(7).get();
    }

    public class RobotK {
        public static final String kDbTabName = "Superstructure";
        public static final Measure<Distance> kHeightTilShooter = Inches.of(7.533);
        public static final boolean kTestMode = false;
        public static final double kSimInterval = 0.020;

        public static final boolean kStopCoast = false;
    }
}
