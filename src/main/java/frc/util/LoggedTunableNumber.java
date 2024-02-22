package frc.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class LoggedTunableNumber {
    private static final String tableKey = "TunableNumbers";

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable(tableKey);

    private double m_default;
    private DoubleSubscriber m_sub;

    public LoggedTunableNumber(String dbKey, double defaultVal) {
        m_default = defaultVal;
        table.putValue(dbKey, NetworkTableValue.makeDouble(m_default));
        m_sub = table.getDoubleTopic(dbKey).subscribe(m_default);
    }

    public double get() {
        return m_sub.get();
    }
}
