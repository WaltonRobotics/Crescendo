package frc.robot.subsystems;

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
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static frc.robot.Constants.AutoConstants.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;
	private ApplyChassisSpeeds m_autoRequest = new ApplyChassisSpeeds();
	private SwerveDriveBrake m_brake = new SwerveDriveBrake();

	public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
		SwerveModuleConstants... modules) {
		super(driveTrainConstants, OdometryUpdateFrequency, modules);
		if (Utils.isSimulation()) {
			startSimThread();
		}
	}

	public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
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

	public Command resetPoseToSpeaker() {
		return runOnce(() -> {
			if (DriverStation.getAlliance().get() == Alliance.Blue) {
				seedFieldRelative(new Pose2d(1.45, 5.5, Rotation2d.fromRadians(0)));
			} else {
				seedFieldRelative(new Pose2d(1.45, 2.7, Rotation2d.fromRadians(0)));
			}
		});
	}

	public Command choreoSwerveCommand(ChoreoTrajectory traj) {
		var resetPoseCmd = runOnce(() -> {
			seedFieldRelative(traj.getInitialPose());
		});

		var choreoFollowCmd = Choreo.choreoSwerveCommand(
			traj,
			() -> getState().Pose,
			new PIDController(kPXController, 0.0, 0.0),
			new PIDController(kPYController, 0.0, 0.0),
			new PIDController(kPThetaController, 0.0, 0.0),
			(speeds) -> setControl(m_autoRequest.withSpeeds(speeds)),
			() -> {
				Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
				return alliance.isPresent() && alliance.get() == Alliance.Red;
			},
			this);

		var brakeCmd = runOnce(() -> setControl(m_brake));

		return Commands.sequence(resetPoseCmd, choreoFollowCmd, brakeCmd).withName("ChoreoFollower");
	}
}
