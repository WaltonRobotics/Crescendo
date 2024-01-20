package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

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
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.auton.AutonChooser;
import frc.util.AdvantageScopeUtil;

import static frc.robot.Constants.Field.*;
import static frc.robot.Constants.Auto.*;
import static frc.robot.Constants.Vision.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * Subsystem so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	private final ApplyChassisSpeeds m_autoRequest = new ApplyChassisSpeeds();
	private final SwerveDriveBrake m_brake = new SwerveDriveBrake();
	private final PIDController m_xController = new PIDController(kPX, 0.0, 0.0);
	private final PIDController m_yController = new PIDController(kPY, 0.0, 0.0);
	private final PIDController m_thetaController = new PIDController(kPTheta, 0.0, 0.0);
	private final PhotonCamera m_cam = new PhotonCamera("cameraName");

	private void configureAutoBuilder() {
		AutoBuilder.configureHolonomic(
			() -> getPose().toPose2d(),
			this::seedFieldRelative,
			() -> m_kinematics.toChassisSpeeds(getState().ModuleStates),
			(speeds) -> setControl(m_autoRequest.withSpeeds(speeds)),
			kPathFollowerConfig,
			() -> {
				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					return alliance.get() == Alliance.Red;
				}
				return false;
			},
			this);
	}

	public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
		SwerveModuleConstants... modules) {
		super(driveTrainConstants, OdometryUpdateFrequency, modules);
		configureAutoBuilder();
		if (Utils.isSimulation()) {
			startSimThread();
		}
	}

	public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
		configureAutoBuilder();
		if (Utils.isSimulation()) {
			startSimThread();
		}
	}

	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> setControl(requestSupplier.get()));
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
				seedFieldRelative(new Pose2d(1.45, kFieldWidth.magnitude() - 5.5, Rotation2d.fromRadians(0)));
			}
		});
	}

	public Command goToAutonPose() {
		return run(() -> {
			var bluePose = AutonChooser.getChosenAutonInitPose();
			if (bluePose.isPresent()) {
				Pose2d pose;
				if (DriverStation.getAlliance().get() == Alliance.Red) {
					Translation2d redTranslation = new Translation2d(bluePose.get().getX(),
						kFieldWidth.magnitude() - bluePose.get().getY());
					Rotation2d redRotation = bluePose.get().getRotation().times(-1);
					pose = new Pose2d(redTranslation, redRotation);
				} else {
					pose = bluePose.get();
				}

				SmartDashboard.putNumberArray("desired pose", AdvantageScopeUtil.toDoubleArr(pose));

				var curPose = getPose().toPose2d();
				var xSpeed = m_xController.calculate(curPose.getX(), pose.getX());
				var ySpeed = m_yController.calculate(curPose.getY(), pose.getY());
				var thetaSpeed = m_thetaController.calculate(curPose.getRotation().getRadians(),
					pose.getRotation().getRadians());
				var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, pose.getRotation());

				setControl(m_autoRequest.withSpeeds(speeds));
			}
		});
	}

	public Pose3d getPose() {
		var result = m_cam.getLatestResult();
		var target = result.getBestTarget();
		var robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
			kFieldLayout.getTagPose(target.getFiducialId()).get(), kCamToRobot);
		return robotPose;
	}

	public Command choreoSwerveCommand(ChoreoTrajectory traj) {
		BooleanSupplier shouldMirror = () -> {
			Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
			return alliance.isPresent() && alliance.get() == Alliance.Red;
		};

		var resetPoseCmd = runOnce(() -> {
			var properTraj = shouldMirror.getAsBoolean() ? traj.flipped() : traj;
			seedFieldRelative(properTraj.getInitialPose());
		});

		var choreoFollowCmd = Choreo.choreoSwerveCommand(
			traj,
			() -> getPose().toPose2d(),
			m_xController,
			m_yController,
			m_thetaController,
			(speeds) -> setControl(m_autoRequest.withSpeeds(speeds)),
			shouldMirror,
			this);

		var brakeCmd = runOnce(() -> setControl(m_brake));

		return Commands.sequence(resetPoseCmd, choreoFollowCmd, brakeCmd).withName("ChoreoFollower");
	}
}
