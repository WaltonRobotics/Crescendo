package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;
	private ApplyChassisSpeeds m_autoRequest = new ApplyChassisSpeeds();

	public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
			SwerveModuleConstants... modules) {
		super(driveTrainConstants, OdometryUpdateFrequency, modules);
		if (Utils.isSimulation()) {
			startSimThread();
		}
	}

	public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
		if (Utils.isSimulation()) {
			startSimThread();
		}
	}

	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

	private void startSimThread() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		m_simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			m_lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		m_simNotifier.startPeriodic(kSimLoopPeriod);
	}

	public void logModulePositions() {
		for (int i = 0; i < Modules.length; i++) {
			SmartDashboard.putNumber("Module " + i + "/Position",
					getModule(i).getDriveMotor().getPosition().getValueAsDouble());
		}
	}

	public void setTestMode() {
		for (int i = 0; i < Modules.length; i++) {
			getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
			getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
		}
	}

	// TODO: make the trajectory chooser at some point
	public Command choreoSwerveCommand() {
		ChoreoTrajectory traj = Choreo.getTrajectory("5pc");

		var resetPoseCmd = runOnce(() -> {
			seedFieldRelative(traj.getInitialPose());
		});

		var choreoFollowCmd = Choreo.choreoSwerveCommand(
			traj,
			()-> getState().Pose,
			new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0),
			new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0),
			new PIDController(Constants.AutoConstants.kPThetaController, 0.0, 0.0),
			(speeds) -> setControl(m_autoRequest.withSpeeds(speeds)),
			() -> {
				Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
				return alliance.isPresent() && alliance.get() == Alliance.Red;
			},
			this
		);

		return Commands.sequence(resetPoseCmd, choreoFollowCmd).withName("ChoreoFollower");
	}
}
