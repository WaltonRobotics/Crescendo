package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveTranslation;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveK;

import static edu.wpi.first.units.Units.Volts;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * Subsystem so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	private final SwerveRequest.FieldCentricFacingAngle m_facingAngle = new SwerveRequest.FieldCentricFacingAngle();

	public final DoubleSupplier m_gyroYawRadsSupplier;

	private final SysIdSwerveTranslation characterization = new SysIdSwerveTranslation();

	private final SysIdRoutine m_sysId = new SysIdRoutine(
		new SysIdRoutine.Config(
			null,
			Volts.of(7),
			null,
			(state) -> SignalLogger.writeString("state", state.toString())),
		new SysIdRoutine.Mechanism(
			(volts) -> setControl(characterization.withVolts(volts)),
			null,
			this));

	public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
		if (Utils.isSimulation()) {
			startSimThread();
		}

		var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(DriveK.kDutyCycleOpenLoopRamp);
		for (var module : Modules) {
			module.getDriveMotor().getConfigurator().apply(openLoopConfig);
		}

		m_gyroYawRadsSupplier = () -> Units.degreesToRadians(getPigeon2().getAngle());
		m_facingAngle.HeadingController.enableContinuousInput(0, 2 * Math.PI);
	}

	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> setControl(requestSupplier.get()));
	}

	public Command applyFcRequest(Supplier<SwerveRequest.FieldCentric> requestSupplier) {
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
			SmartDashboard.putNumber("Module " + i + "/position",
				getModule(i).getDriveMotor().getPosition().getValueAsDouble());
		}
	}

	public Command resetModulePositions() {
		return Commands.runOnce(() -> {
			for (int i = 0; i < Modules.length; i++) {
				getModule(i).getDriveMotor().setPosition(0);
			}
		});
	}

	public void setTestMode() {
		for (int i = 0; i < Modules.length; i++) {
			getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
			getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
		}
	}

	public Command resetPose(PathPlannerPath path) {
		return Commands.runOnce(() -> {
			var alliance = DriverStation.getAlliance();
			var correctedPath = path;
			if (alliance.isPresent() && alliance.get() == Alliance.Red) {
				correctedPath = path.flipPath();
			}
			var correctedTraj = correctedPath.getTrajectory(new ChassisSpeeds(), new Rotation2d());
			var correctedPose = correctedTraj.getInitialTargetHolonomicPose();
			seedFieldRelative(correctedPose);
		});
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysId.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return m_sysId.dynamic(direction);
	}

	public void periodic() {}
}
