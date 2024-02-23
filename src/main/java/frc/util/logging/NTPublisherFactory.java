package frc.util.logging;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.TimedRobot;

public final class NtPublisherFactory {
    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static final NetworkTable traceTable = inst.getTable("Trace");

    public static DoublePublisher makeDoublePub(NetworkTable table, String name, PubSubOption... options) {
        DoubleTopic topic = table.getDoubleTopic(name);
        return topic.publish(options);
    }

    public static DoublePublisher makeDoublePub(String table, String name, PubSubOption... options) {
        return makeDoublePub(inst.getTable(table), name, options);
    }

    public static DoublePublisher makeDoublePub(String name) {
        return makeDoublePub(traceTable, name, PubSubOption.periodic(TimedRobot.kDefaultPeriod));
    }

    public static DoubleArrayPublisher makeDoubleArrPub(NetworkTable table, String name, PubSubOption... options) {
        DoubleArrayTopic topic = table.getDoubleArrayTopic(name);
        return topic.publish(options);
    }

    public static DoubleArrayPublisher makeDoubleArrPub(String table, String name, PubSubOption... options) {
        return makeDoubleArrPub(inst.getTable(table), name, options);
    }

    public static DoubleArrayPublisher makeDoubleArrTracePub(String name) {
        return makeDoubleArrPub(traceTable, name, PubSubOption.periodic(TimedRobot.kDefaultPeriod));
    }

    public static BooleanPublisher makeBoolPub(NetworkTable table, String name, PubSubOption... options) {
        BooleanTopic topic = table.getBooleanTopic(name);
        return topic.publish(options);
    }

    public static BooleanPublisher makeBoolPub(String table, String name, PubSubOption... options) {
        return makeBoolPub(inst.getTable(table), name, options);
    }

    public static BooleanPublisher makeBoolTracePub(String name) {
        return makeBoolPub(traceTable, name, PubSubOption.periodic(TimedRobot.kDefaultPeriod));
    }

    public static StringPublisher makeStringPub(NetworkTable table, String name, PubSubOption... options) {
        StringTopic topic = table.getStringTopic(name);
        return topic.publish(options);
    }

    public static StringPublisher makeStringPub(String table, String name, PubSubOption... options) {
        return makeStringPub(inst.getTable(table), name, options);
    }

    public static StringPublisher makeStringTracePub(String name) {
        return makeStringPub(traceTable, name, PubSubOption.periodic(TimedRobot.kDefaultPeriod));
    }
}