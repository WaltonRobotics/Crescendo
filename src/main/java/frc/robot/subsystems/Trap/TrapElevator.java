package frc.robot.subsystems.trap;

import static frc.robot.Constants.TrapK.TrapElevatorK.*;
import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TrapElevator extends SubsystemBase {
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
    public TrapElevator() {

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
     * @param meters
     *            the height in meters
     *            sets target height (meters)
     */
    private void i_setTargetHeight(double meters) {
        m_targetHeight = MathUtil.clamp(meters, kMinHeight, kMaxHeight);
    }

    /**
     * see above ^-^
     * 
     * @param meters
     *            the height in meters
     * @return a command to set the target height
     */
    public Command setTargetHeight(double meters) {
        return runOnce(() -> {
            i_setTargetHeight(meters);
        });
    }

    /**
     * @return whether the ele is hitting the sensor
     */
    public boolean isAtBottom() {
        return !m_lowerLimit.get();
    }

    /**
     * @return sets the motor position to 0 (rotations)
     */
    private Command rezero() {
        return runOnce(() -> {
            m_motor.setPosition(0);
        });
    }

    /**
     * @param targetHeightMeters
     *            target height (meters)
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
     * @param heightMeters
     *            height to hold in (meters)
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
     * @param coast
     * @return a command to setCoast to coast value
     */
    public Command setCoast(boolean coast) {
        return runOnce(() -> {
            m_motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
            m_isCoast = coast;
        });
    }

    /**
     * @return m_isCoast
     */
    public boolean getCoast() {
        return m_isCoast;
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
            }).until(m_lowerLimitTrigger),
            rezero());
    }

    /**
     * if there is no input, set the elevator to a target height
     * TODO: figure out how to do
     * 
     * @param pwr
     *            power used
     * @return moving the ele using the controller
     */
    public Command teleopCmd(DoubleSupplier pwr) {
        return run(() -> {
            double direction = Math.signum(pwr.getAsDouble());
            double output = 0;

            if (!(isAtBottom() && direction == -1)) {
                output = MathUtil.applyDeadband(pwr.getAsDouble(), kStickDeadband);
            }

            m_targetHeight += output * .02;
            double effort = getEffortForTarget(m_targetHeight);
            double holdEffort = getEffortToHold(m_targetHeight);

            if (output > 0) {
                m_motor.setVoltage(effort);
            } else {
                output = MathUtil.applyDeadband(pwr.getAsDouble(), kStickDeadband);
                m_motor.setVoltage(holdEffort);
            }
        });
    }

    /**
     * @param targetHeightMeters
     *            the target height (meters)
     * @return a command that can move the elevator to a certain height
     */
    public Command toHeightCmd(Double targetHeightMeters) {
        var setupCmd = runOnce(() -> {
            m_controller.reset(getActualHeightMeters());
            i_setTargetHeight(targetHeightMeters);
        });
        var runCmd = run(() -> {
            var effort = MathUtil.clamp(
                getEffortForTarget(m_targetHeight),
                -kVoltageCompSaturationVolts,
                kVoltageCompSaturationVolts);
            m_motor.set(effort / kVoltageCompSaturationVolts);
        }).until(() -> {
            return m_controller.atGoal();
        });
        var doneCmd = runOnce(() -> {
            m_motor.set(0);
        });

        return Commands.sequence(
            setupCmd,
            runCmd,
            doneCmd);
    }

    @Override
    public void periodic() {

    }
}
