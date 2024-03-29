package frc.util.logging;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
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