package frc.robot.subsystems.Trap;

import static frc.robot.Constants.TrapK.TrapEleK.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

    /*
     * Returns height in rotations from min height value
     */
    public double getActualHeightRot() {
		return m_motor.getRotorPosition().getValue();
	}

    // TODO: figure out how to do
    public double getActualHeightMeters() {
        return 0;
    }

    /*
     * Returns whether the ele is hitting the sensor
     */
    public boolean isAtBottom() {
        return !m_lowerLimit.get();
    }

    // TODO: figure out how to do
    private double getActualVeloMps() {
        return 0;
    }

    /*
     * Goes back to bottom (hits limit)
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
}
