package frc.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LoggedTunableNumber {
    private static final String tableKey = "TunableNumbers";

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable(tableKey);

    private double m_default;
    private DoubleSubscriber m_sub;

    public LoggedTunableNumber(String dbKey, double defaultVal) {
        m_default = defaultVal;
        m_sub = table.getDoubleTopic(dbKey).subscribe(m_default);
    }

    public double get() {
        return m_sub.get();
    }
}
