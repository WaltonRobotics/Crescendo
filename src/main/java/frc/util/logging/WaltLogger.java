package frc.util.logging;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.struct.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.util.FMSCacher;
import frc.robot.Constants;

public class WaltLogger {
    private WaltLogger() {
    }

    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static final NetworkTable logTable = inst.getTable("Robot");

    private static boolean shouldPublishNt() {
        return Constants.kDebugLoggingEnabled && !FMSCacher.getCachedFMSAttached();
    }

    public static IntLogger logInt(String table, String name, PubSubOption... options) {
        return new IntLogger(table, name, options);
    }

    public static final class Pose2dLogger implements Consumer<Pose2d> {
        public final StructPublisher<Pose2d> ntPub;
        public final StructLogEntry<Pose2d> logEntry;

        public Pose2dLogger(String subTable, String name, PubSubOption... options) {
            StructTopic<Pose2d> topic = logTable.getSubTable(subTable).getStructTopic(name, new Pose2dStruct());
            ntPub = topic.publish(options);
            logEntry = StructLogEntry.create(DataLogManager.getLog(), "Robot/" + subTable + "/" + name, new Pose2dStruct());
        }

        @Override
        public void accept(Pose2d value) {
            if (shouldPublishNt()) {
                ntPub.set(value);
            } else {
                logEntry.append(value);
            }
        }
    }

    public static Pose2dLogger logPose2d(String table, String name, PubSubOption... options) {
        return new Pose2dLogger(table, name, options);
    }

    public static final class Pose3dLogger implements Consumer<Pose3d> {
        public final StructPublisher<Pose3d> ntPub;
        public final StructLogEntry<Pose3d> logEntry;

        public Pose3dLogger(String subTable, String name, PubSubOption... options) {
            StructTopic<Pose3d> topic = logTable.getSubTable(subTable).getStructTopic(name, new Pose3dStruct());
            ntPub = topic.publish(options);
            logEntry = StructLogEntry.create(DataLogManager.getLog(), "Robot/" + subTable + "/" + name, new Pose3dStruct());
        }

        @Override
        public void accept(Pose3d value) {
            if (shouldPublishNt()) {
                ntPub.set(value);
            } else {
                logEntry.append(value);
            }
        }

        public void accept(Translation3d value) {
            accept(new Pose3d(value.getX(), value.getY(), value.getZ(), new Rotation3d()));
        }
    }

    public static Pose3dLogger logPose3d(String name, String table, PubSubOption... options) {
        return new Pose3dLogger(name, table, options);
    }

    public static final class Transform3dLogger implements Consumer<Transform3d> {
        public final StructPublisher<Transform3d> ntPub;
        public final StructLogEntry<Transform3d> logEntry;

        public Transform3dLogger(String subTable, String name, PubSubOption... options) {
            StructTopic<Transform3d> topic = logTable.getSubTable(subTable).getStructTopic(name, new Transform3dStruct());
            ntPub = topic.publish(options);
            logEntry = StructLogEntry.create(DataLogManager.getLog(), "Robot/" + subTable + "/" + name, new Transform3dStruct());
        }

        @Override
        public void accept(Transform3d value) {
            if (shouldPublishNt()) {
                ntPub.set(value);
            } else {
                logEntry.append(value);
            }
        }
    }

    public static Transform3dLogger logTransform3d(String table, String name, PubSubOption... options) {
        return new Transform3dLogger(table, name, options);
    }

    public static final class Translation3dLogger implements Consumer<Translation3d> {
        public final StructPublisher<Translation3d> ntPub;
        public final StructLogEntry<Translation3d> logEntry;

        public Translation3dLogger(String subTable, String name, PubSubOption... options) {
            StructTopic<Translation3d> topic = logTable.getSubTable(subTable).getStructTopic(name, new Translation3dStruct());
            ntPub = topic.publish(options);
            logEntry = StructLogEntry.create(DataLogManager.getLog(), "Robot/" + subTable + "/" + name, new Translation3dStruct());
        }

        @Override
        public void accept(Translation3d value) {
            if (shouldPublishNt()) {
                ntPub.set(value);
            } else {
                logEntry.append(value);
            }
        }
    }

    public static Translation3dLogger logTranslation3d(String table, String name, PubSubOption... options) {
        return new Translation3dLogger(table, name, options);
    }

    public static final class Translation2dLogger implements Consumer<Translation2d> {
        public final StructPublisher<Translation2d> ntPub;
        public final StructLogEntry<Translation2d> logEntry;

        public Translation2dLogger(String subTable, String name, PubSubOption... options) {
            StructTopic<Translation2d> topic = logTable.getSubTable(subTable).getStructTopic(name, new Translation2dStruct());
            ntPub = topic.publish(options);
            logEntry = StructLogEntry.create(DataLogManager.getLog(), "Robot/" + subTable + "/" + name, new Translation2dStruct());
        }

        @Override
        public void accept(Translation2d value) {
            if (shouldPublishNt()) {
                ntPub.set(value);
            } else {
                logEntry.append(value);
            }
        }
    }

    public static Translation2dLogger logTranslation2d(String table, String name, PubSubOption... options) {
        return new Translation2dLogger(table, name, options);
    }

    public static final class IntLogger implements Consumer<Integer> {
        public final IntegerPublisher ntPub;
        public final IntegerLogEntry logEntry;

        public IntLogger(String subTable, String name, PubSubOption... options) {
            ntPub = NTPublisherFactory.makeIntPub(logTable.getSubTable(subTable), name, options);
            logEntry = new IntegerLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
        }

        @Override
        public void accept(Integer value) {
            if (shouldPublishNt()) {
                ntPub.set(value);
            } else {
                logEntry.append(value);
            }
        }
    }

    public static final class DoubleLogger implements Consumer<Double> {
        public final DoublePublisher ntPub;
        public final DoubleLogEntry logEntry;

        public DoubleLogger(String subTable, String name, PubSubOption... options) {
            ntPub = NTPublisherFactory.makeDoublePub(logTable.getSubTable(subTable), name, options);
            logEntry = new DoubleLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
        }

        @Override
        public void accept(Double value) {
            if (shouldPublishNt()) {
                ntPub.set(value);
            } else {
                logEntry.append(value);
            }
        }
    }

    public static DoubleLogger logDouble(String table, String name, PubSubOption... options) {
        return new DoubleLogger(table, name, options);
    }

    public static final class BooleanLogger implements Consumer<Boolean> {
        public final BooleanPublisher ntPub;
        public final BooleanLogEntry logEntry;

        public BooleanLogger(String subTable, String name, PubSubOption... options) {
            ntPub = NTPublisherFactory.makeBoolPub(logTable.getSubTable(subTable), name, options);
            logEntry = new BooleanLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
        }

        @Override
        public void accept(Boolean value) {
            if (shouldPublishNt()) {
                ntPub.set(value);
            } else {
                logEntry.append(value);
            }
        }

        public void accept(BooleanSupplier valueSup) {
            accept(valueSup.getAsBoolean());
        }
    }

    public static BooleanLogger logBoolean(String table, String name, PubSubOption... options) {
        return new BooleanLogger(table, name, options);
    }

    public static final class DoubleArrayLogger implements Consumer<double[]> {
        public final DoubleArrayPublisher ntPub;
        public final DoubleArrayLogEntry logEntry;

        public DoubleArrayLogger(String subTable, String name) {
            ntPub = NTPublisherFactory.makeDoubleArrPub(logTable.getSubTable(subTable), name);
            logEntry = new DoubleArrayLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
        }

        @Override
        public void accept(double[] value) {
            if (shouldPublishNt()) {
                ntPub.accept(value);
            } else {
                logEntry.append(value);
            }
        }
    }

    public static DoubleArrayLogger logDoubleArray(String table, String name) {
        return new DoubleArrayLogger(table, name);
    }

    public static final class StringLogger implements Consumer<String> {
        public final StringPublisher ntPub;
        public final StringLogEntry logEntry;

        public StringLogger(String subTable, String name) {
            ntPub = NTPublisherFactory.makeStringPub(logTable.getSubTable(subTable), name);
            logEntry = new StringLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
        }

        @Override
        public void accept(String value) {
            if (shouldPublishNt()) {
                ntPub.set(value);
            } else {
                logEntry.append(value);
            }
        }
    }

    public static StringLogger logString(String table, String name) {
        return new StringLogger(table, name);
    }
}