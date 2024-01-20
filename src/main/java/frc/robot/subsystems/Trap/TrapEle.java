package frc.robot.subsystems.Trap;

import static frc.robot.Constants.TrapK.TrapEleK.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TrapEle extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(kExtendCANID);
    private final DigitalInput m_lowerLimit = new DigitalInput(kLowerLimitSwitchPort);
    private final Trigger m_lowerLimitTrigger = new Trigger(m_lowerLimit::get).negate();

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, 0, 0, kConstraints);
    private final PIDController m_holdController = new PIDController(kPHold, 0, 0);

    private double m_pdEffort = 0;
	private double m_ffEffort = 0;
    private double m_holdPdEffort = 0;
	private double m_holdFfEffort = 0;
    
    private double m_targetHeight = 0;

	private boolean m_isCoast = false;

    // TODO: add a constructor
    public TrapEle() {

    }

    /**
     * @return height in rotations from min height value
     */
    public double getActualHeightRot() {
		return m_motor.getRotorPosition().getValue();
	}

    // TODO: figure out how to do
    public double getActualHeightMeters() {
        return 0;
    }
 
    /**
     * @return the target height in meters
     */
    public double getTargetHeightMeters() {
        return m_targetHeight;
    }

    /**
     * @return whether the ele is hitting the sensor
     */
    public boolean isAtBottom() {
        return !m_lowerLimit.get();
    }

    // TODO: figure out how to do
    private double getActualVeloMps() {
        return 0;
    }

    /**
     * @param targetHeightMeters target height (meters)
     * @return effort needed to reach target height
     */
    public double getEffortForTarget(double targetHeightMeters) {
		m_pdEffort = m_controller.calculate(getActualHeightMeters(), targetHeightMeters);
		m_ffEffort = 0;
		var pdSetpoint = m_controller.getSetpoint();
		if (pdSetpoint.velocity != 0) {
			m_ffEffort = kFeedforward.calculate(pdSetpoint.velocity); // what is this !!!
		}
		double totalEffort = m_ffEffort + m_pdEffort;
		
		// insert logging here

		return totalEffort;
	}

    /**
     * @param heightMeters height to hold in (meters)
     * @return effort needed to hold at a height
     */
    private double getEffortToHold(double heightMeters) {
		m_holdPdEffort = m_holdController.calculate(getActualHeightMeters(), heightMeters);
		m_holdFfEffort = 0;
		var pdSetpoint = m_holdController.getSetpoint();
		if (pdSetpoint != 0) {
			m_holdFfEffort = kHoldKs;
		}
		double totalEffort = m_holdFfEffort + m_holdPdEffort;
		return totalEffort;
	}

    /**
     * @param meters the height in meters
     * sets target height (meters)
     */
    private void i_setTargetHeight(double meters) {
        m_targetHeight = MathUtil.clamp(meters, kMinHeight, kMaxHeight);
    }

    /**
     * see above ^-^
     * @param meters the height in meters
     * @return a command to set the target height
     */
    public Command setTargetHeight(double meters) {
        return runOnce(() -> {
            i_setTargetHeight(meters);
        });
    }

    /**
     * @param coast
     * @return a command to setCoast to coast value
     */
    public Command setCoast(boolean coast) {
        return runOnce(() -> {
            m_motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        });
    }

    /**
     * @return a command to move the ele back to the bottom (hits limit)
     */
    public Command autoReset() {
		return Commands.sequence(
			startEnd(() -> {
				m_motor.setVoltage(-2);
			}, () -> {
				m_motor.setVoltage(0);
			}).until(m_lowerLimitTrigger)
		);
	}

    /**
     * if there is no input, set the elevator to a target height
     * @param pwr power used
     * @return moving the ele using the controller
     */
    public Command teleopCmd(DoubleSupplier pwr) {
        return Commands.none();
    }

    @Override
    public void periodic() {

    }
}
