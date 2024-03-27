package frc.util.logging;

import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Consumer;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.RobotK;

/* reference Mechanical Advantages' LoggedTunableNumber */
/* this is prob not right, im not entirely sure what im doing help */

public class LoggedTunableNumber {
    private static final String m_tableKey = "TunableNums";

    private String m_key;
    private boolean m_hasDefault = false;
    private double m_defaultVal;
    private NetworkTableEntry nte_dashNumEntry;
    private NetworkTableInstance nte_inst = NetworkTableInstance.getDefault(); //idk?
    private HashMap<Integer, Double> m_lastHasChangedVals = new HashMap<>();

    public LoggedTunableNumber(String dashboardKey) {
        m_key = m_tableKey + "/" + dashboardKey;
    }

    public LoggedTunableNumber(String dashboardKey, double defaultVal) {
        this(dashboardKey);
        setDefault(defaultVal);
    }

    public void setDefault(double defaultVal) {
        if(!m_hasDefault) {
            m_hasDefault = true;
            m_defaultVal = defaultVal;
            if(RobotK.kTestMode) {
                nte_dashNumEntry = new NetworkTableEntry(nte_inst, 1); // idk what a handle is T^T
                nte_dashNumEntry.setDouble(defaultVal);
            }
        }
    }

    public double get() {
        if(!m_hasDefault) {
            return 0.0;
        } 
        return nte_dashNumEntry.getDouble(m_defaultVal);
    }

    public boolean hasChanged(int id) {
        double currentVal = get();
        double lastVal = m_lastHasChangedVals.get(id);

        if(lastVal == 0 || currentVal != lastVal) {
            m_lastHasChangedVals.put(id, currentVal);
            return true;
        }

        return false;
    }

    public static void ifChanged(
        int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
            if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
                action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
            }
    }

    public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
        ifChanged(id, values -> action.run(), tunableNumbers);
    }
}
