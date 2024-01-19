package frc.robot.subsystems.Trap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static frc.robot.Constants.TrapK.TrapTiltK.*;

public class TrapTilt extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(kMotorCANID, MotorType.kBrushless);
    private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(kAbsoluteEncoderPort); // i didn't initally include this but neos are supposed to have one

    private boolean m_isCoast = false;

    // TODO: work out how much of this is relevant and what values to change
    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, 0, 0, kConstraints);
	private final PIDController m_holdController = new PIDController(kPHold, 0, 0); // modified these to be a "P controller i think, if it doesn't work ig fix it
	private double m_targetAngle = 0;
	private static double m_pdEffort = 0;
	private double m_holdPdEffort = 0;
	private double m_holdFfEffort = 0;

    public TrapTilt() {
        double subsystemInitBegin = Timer.getFPGATimestamp();
        // System.out.println("[INIT] TrapTilt Init Begin");
        // needs "absolute encoder"? check to see if we have this
    }

    // this WAS an i_ method but i have no idea whats up with them and i think i heard somethingn about not doing them anymore
    public void setTarget(double degrees) {
        m_targetAngle = MathUtil.clamp(degrees, 0, kAbsMaxDegree);
    }

    public boolean atForwardLimit() {
        if (getDegrees() >= kAbsMaxDegree) {
            return true;
        }
        return false;
    }

    public boolean getHomeSwitch() {
        // seems to get position of some kind of limit switch denoting if it's at its reverse limit
        // check to see if we have this and what we need to do if we don't
        return true;
    }

    public boolean atReverseLimit() {
        return getHomeSwitch();
    }

    public double getDegrees() {
        // absolute encoder required
        return 0;
    }

    public double getEffortForTarget(double angleDegrees) { // why is this param "angleDegrees"? doesn't seem consistent with other methods (it is the angle in degrees lol)
        m_pdEffort = m_controller.calculate(getDegrees(), angleDegrees);
        return 0;
    }

}
