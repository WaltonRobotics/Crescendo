package frc.util.logging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoggedTunableNumber {
    private static final String m_tableKey = "TunableNumbers";
    
    private String m_key;
    private boolean m_hasDefault = false;
    private double m_defaultVal;
    private double m_value;

    public LoggedTunableNumber(String key) {
        this(key, 0.0);
    }

    public LoggedTunableNumber(String key, double defaultVal) {
        m_key = m_tableKey + "/" + key;
        setDefault(defaultVal);
        m_value = m_defaultVal;
    }

    public void setDefault(double defaultVal) {
        if (!m_hasDefault) {
            m_hasDefault = true;
            m_defaultVal = defaultVal;
        }
        if (!DriverStation.isFMSAttached()) {
            SmartDashboard.putNumber(m_key, SmartDashboard.getNumber(m_key, m_defaultVal));
        }
    }

    public double get() {
        if (!m_hasDefault) {
            return 0.0;
        }

        if (DriverStation.isFMSAttached()) {
            return m_defaultVal;
        }
        
        return m_value;
    }

    public void periodic() {
        m_value = SmartDashboard.getNumber(m_key, m_defaultVal);
    }
}
