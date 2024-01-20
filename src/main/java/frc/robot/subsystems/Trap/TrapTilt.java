package frc.robot.subsystems.trap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static frc.robot.Constants.*;
import static frc.robot.Constants.TrapK.TrapTiltK.*;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class TrapTilt extends SubsystemBase {
	private final CANSparkMax m_motor = new CANSparkMax(kMotorCANID, MotorType.kBrushless);
	private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(kAbsoluteEncoderPort); // i didn't initally
																									// include this but
																									// neos are supposed
																									// to have one
	private final DigitalInput m_homeSwitch = new DigitalInput(kHomeSwitchPort);

	// private boolean m_isCoast = false;

	// TODO: work out how much of this is relevant and what values to change
	private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, 0, 0, kConstraints);
	private final PIDController m_holdController = new PIDController(kPHold, 0, 0);
	private double m_targetAngle = 0;
	private static double m_pdEffort = 0;
	private double m_holdPdEffort = 0;
	private double m_holdFfEffort = 0;

	// private final DoubleLogger log_actualAngle =
	// WaltLogger.logDouble(DB_TAB_NAME, "ActualAngle");
	// private final DoubleLogger log_rawAbsVal = WaltLogger.logDouble(DB_TAB_NAME,
	// "RawAbs");

	public final Trigger m_homeSwitchTrigger = new Trigger(m_homeSwitch::get).negate();
	// private final GenericEntry nte_isCoast;

	// public static double nte_pdEffort = m_pdEffort;

	public TrapTilt() {
		double subsystemInitBegin = Timer.getFPGATimestamp();
		// System.out.println("[INIT] TrapTilt Init Begin");
		m_absoluteEncoder.reset();
		m_motor.setIdleMode(IdleMode.kBrake);
		m_motor.setSmartCurrentLimit(kMotorCurrLimit);

		m_holdController.setTolerance(0.75);

		double subsysInitElapsed = Timer.getFPGATimestamp() - subsystemInitBegin;
		System.out.println("[INIT] TiltSubsystem Init End: " + subsysInitElapsed + "s");

		// nte_isCoast = Shuffleboard.getTab(DB_TAB_NAME)
		// .add("tilt coast", false)
		// .withWidget(BuiltInWidgets.kToggleSwitch)
		// .getEntry();
	}

	// this WAS an i_ method but i have no idea whats up with them and i think i
	// heard somethingn about not doing them anymore
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
		return m_homeSwitchTrigger.getAsBoolean();
	}

	public boolean atReverseLimit() {
		return getHomeSwitch();
	}

	public double getEffortForTarget(double angleDegrees) { // why is this param "angleDegrees"? doesn't seem consistent
															// with other methods (it is the angle in degrees lol)
		m_pdEffort = m_controller.calculate(getDegrees(), angleDegrees);
		return m_pdEffort;
	}

	private double getEffortToHold(double degrees) {
		m_holdPdEffort = m_holdController.calculate(getDegrees(), degrees);
		m_holdFfEffort = 0;
		var pdSetpoint = m_holdController.getSetpoint();
		if (pdSetpoint != 0) {
			m_holdFfEffort = kHoldKs * Math.signum(m_holdController.getPositionError());
		}
		double totalEffort = m_holdFfEffort + m_holdPdEffort;
		SmartDashboard.putNumber("tiltHoldPEff", m_holdPdEffort);
		SmartDashboard.putNumber("tiltHoldFFEff", m_holdFfEffort);
		SmartDashboard.putNumber("totalHoldEffort", totalEffort);
		return totalEffort;
	}

	public Command holdAngle() {
		return run(() -> {
			var holdEffort = MathUtil.clamp(getEffortToHold(m_targetAngle), -kVoltageCompSaturationVolts,
				kVoltageCompSaturationVolts);
			setVoltage(holdEffort / kVoltageCompSaturationVolts);
		})
			.withName("Hold Angle");
	}

	public double getDegrees() {
		var rawDeg = ((m_absoluteEncoder.get()) * 360);
		return MathUtil.clamp(rawDeg, 0, kAbsMaxDegree);
	}

	public Command resetEncoder() {
		return Commands.runOnce(() -> m_absoluteEncoder.reset()).ignoringDisable(true);
	}

	public Command teleopCmd(DoubleSupplier power) {
		return run(() -> {
			double powerVal = MathUtil.applyDeadband(power.getAsDouble(), kStickDeadband);
			m_targetAngle += powerVal * 1.2;
			double effort = getEffortForTarget(m_targetAngle);
			double holdEffort = getEffortToHold(m_targetAngle);

			if (powerVal > 0) {
				setVoltage(effort);
			} else {
				powerVal = MathUtil.applyDeadband(power.getAsDouble(), kStickDeadband);
				setVoltage(holdEffort);
			}
			setVoltage(effort);
		});
	}

	public void setSpeed(double power) {
		double output;
		double dir = Math.signum(power);
		double powerVal = MathUtil.applyDeadband(power, kStickDeadband);
		if ((atForwardLimit() && dir == 1) || (atReverseLimit() && dir == -1)) {
			output = 0;
		} else {
			output = powerVal;
		}
		m_motor.set(output);
	}

	public void setVoltage(double voltage) {
		double output;
		double dir = Math.signum(voltage);
		double powerVal = MathUtil.applyDeadband(voltage, kStickDeadband);
		if ((atForwardLimit() && dir == 1) || (atReverseLimit() && dir == -1)) {
			output = 0;
		} else {
			output = powerVal;
		}
		m_motor.setVoltage(output);
	}

	public Command autoHome() {
		return new DeferredCommand(() -> {
			if (atReverseLimit()) {
				return Commands.runOnce(() -> m_absoluteEncoder.reset());
			} else {
				return Commands.sequence(
					startEnd(() -> {
						setVoltage(-2);
					}, () -> {
						setVoltage(0);
					}).until(m_homeSwitchTrigger)
						.andThen(
							atReverseLimit() ? new InstantCommand(() -> m_absoluteEncoder.reset()) : Commands.none()));
			}
		}, Set.of(this));
	}

	public Command toAngle(double angle) {
		var setupCmd = runOnce(() -> {
			if (angle > getDegrees()) {
				double tempMaxVelocity = kMaxVelocityForward;
				double tempMaxAcceleration = kMaxAccelerationForward;

				m_controller.setConstraints(new TrapezoidProfile.Constraints(tempMaxVelocity, tempMaxAcceleration));
			} else {
				m_controller.setConstraints(kConstraints);
			}
			m_controller.reset(getDegrees());
			setTarget(angle);
		});

		var moveCmd = run(() -> {
			var effort = MathUtil.clamp(getEffortForTarget(m_targetAngle), -12, 12);
			setVoltage(effort);
		})
			.until(() -> {
				return m_controller.atGoal();
			})
			.finallyDo((intr) -> {
				m_motor.set(0);
			})
			.withTimeout(1.6)
			.withName("ToAngle");

		return Commands.sequence(
			setupCmd,
			moveCmd);
	}

	public void setCoast(boolean coast) {
		m_motor.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
	}

	// fun extra stuff

	// @Override
	// public void periodic() {
	// updateShuffleBoard();
	// setCoast(m_isCoast);
	// }

	// public void updateShuffleBoard() {
	// // Push telemetry
	// log_actualAngle.accept(getDegrees());
	// log_rawAbsVal.accept(m_absoluteEncoder.get());
	// m_isCoast = nte_isCoast.getBoolean(false);
	// SmartDashboard.putNumber("tiltHoldPEff", m_holdPdEffort);
	// SmartDashboard.putNumber("tiltHoldFFEff", m_holdFfEffort);
	// SmartDashboard.putNumber("targetTiltAngle", m_targetAngle);
	// }
}
