package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveTranslation;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Vision.VisionMeasurement3d;
import frc.robot.auton.AutonChooser;
import frc.util.AdvantageScopeUtil;
import frc.util.AllianceFlipUtil;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.BooleanLogger;
import frc.util.logging.WaltLogger.DoubleArrayLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

import static frc.robot.Constants.FieldK.*;
import static frc.robot.generated.TunerConstants.kDriveRadius;
import static frc.robot.generated.TunerConstants.kDriveRotationsPerMeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AutoK.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * Subsystem so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	private final ApplyChassisSpeeds m_autoRequest = new ApplyChassisSpeeds()
		.withDriveRequestType(DriveRequestType.Velocity);
	private final SwerveRequest.RobotCentric m_characterisationReq = new SwerveRequest.RobotCentric()
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveDriveBrake m_brake = new SwerveDriveBrake();
	private final PIDController m_xController = new PIDController(kPTranslation, 0.0, 0.0);
	private final PIDController m_yController = new PIDController(kPTranslation, 0.0, 0.0);
	private final PIDController m_thetaController = new PIDController(kPTheta, 0.0, 0.0);

	private Rotation2d m_desiredRot = new Rotation2d();

	private final double m_characterisationSpeed = 1;
	public final DoubleSupplier m_gyroYawRadsSupplier;
	private final SlewRateLimiter m_omegaLimiter = new SlewRateLimiter(1);

	private double lastGyroYawRads = 0;
	private double accumGyroYawRads = 0;

	private double[] startWheelPositions = new double[4];
	private double currentEffectiveWheelRadius = 0;

	// vision yaw align
	boolean m_hasVisionYaw = false;
	Measure<Angle> m_visionYaw = Rotations.of(0);
	Timer m_visYawTimer = new Timer();

	private final SysIdSwerveTranslation characterization = new SysIdSwerveTranslation();
	// private final SysIdSwerveRotation characterization = new
	// SysIdSwerveRotation();
	// private final SysIdSwerveSteerGains characterization = new
	// SysIdSwerveSteerGains();
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

	private final DoubleLogger log_accumGyro = WaltLogger.logDouble("Swerve", "accumGyro");
	private final DoubleLogger log_avgWheelPos = WaltLogger.logDouble("Swerve", "avgWheelPos");
	private final DoubleLogger log_curEffWheelRad = WaltLogger.logDouble("Swerve", "curEffWheelRad");
	private final DoubleLogger log_lastGyro = WaltLogger.logDouble("Swerve", "lastGyro");
	private final DoubleLogger log_rotationSpeed = WaltLogger.logDouble("Swerve", "rot_sec",
		PubSubOption.sendAll(true));

	private final DoubleLogger log_desiredRot = WaltLogger.logDouble("Swerve", "desiredRot");
	private final DoubleLogger log_rot = WaltLogger.logDouble("Swerve", "rotation");
	private final DoubleArrayLogger log_poseError = WaltLogger.logDoubleArray("Swerve", "poseError");
	private double[] m_poseError = new double[3];
	private final DoubleArrayLogger log_desiredPose = WaltLogger.logDoubleArray("Swerve", "desiredPose");
	private double[] m_desiredPose = new double[3];
	private final DoubleArrayLogger log_wheelVeloErrors = WaltLogger.logDoubleArray("Swerve", "wheelVeloErrors");
	private double[] m_wheelVeloErrs = new double[4];
	private final DoubleArrayLogger log_wheelVelos = WaltLogger.logDoubleArray("Swerve", "wheelVelos");
	private double[] m_wheelVelos = new double[4];
	private final DoubleArrayLogger log_wheelVeloTargets = WaltLogger.logDoubleArray("Swerve", "wheelVeloTargets");
	private double[] m_wheelVeloTargets = new double[4];

	private final DoubleLogger log_yawErr = WaltLogger.logDouble("Swerve", "yawError");
	private final DoubleLogger log_yawEffort = WaltLogger.logDouble("Swerve", "yawEffort");
	private final DoubleLogger log_yawErrOpt = WaltLogger.logDouble("Swerve", "yawErrorOpt");
	private final BooleanLogger log_hasYaw = WaltLogger.logBoolean("Swerve", "hasYaw");

	public void addVisionMeasurement3d(VisionMeasurement3d measurement) {
		// sadge!
		var now = Timer.getFPGATimestamp();
		var timestamp = measurement.estimate().timestampSeconds;
		if (timestamp > now) return;

		addVisionMeasurement(
			measurement.estimate().estimatedPose.toPose2d(),
			timestamp,
			measurement.stdDevs());
	}

	private void configureAutoBuilder() {
		AutoBuilder.configureHolonomic(
			() -> getState().Pose,
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

	public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
		configureAutoBuilder();
		if (Utils.isSimulation()) {
			startSimThread();
		}
		m_gyroYawRadsSupplier = () -> Units.degreesToRadians(getPigeon2().getAngle());
		m_thetaController.enableContinuousInput(0, 2 * Math.PI);
		m_visYawTimer.reset();
	}

	public Command wheelRadiusCharacterisation(double omegaDirection) {
		var initialize = runOnce(() -> {
			lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
			accumGyroYawRads = 0;
			currentEffectiveWheelRadius = 0;
			for (int i = 0; i < Modules.length; i++) {
				var pos = Modules[i].getPosition(true);
				startWheelPositions[i] = pos.distanceMeters * kDriveRotationsPerMeter;
			}
			m_omegaLimiter.reset(0);
		});

		var executeEnd = runEnd(
			() -> {
				setControl(m_characterisationReq
					.withRotationalRate(m_omegaLimiter.calculate(m_characterisationSpeed * omegaDirection)));
				accumGyroYawRads += MathUtil.angleModulus(m_gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
				lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
				double averageWheelPosition = 0;
				double[] wheelPositions = new double[4];
				for (int i = 0; i < Modules.length; i++) {
					var pos = Modules[i].getPosition(true);
					wheelPositions[i] = pos.distanceMeters * kDriveRotationsPerMeter;
					averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
				}
				averageWheelPosition /= 4.0;
				currentEffectiveWheelRadius = (accumGyroYawRads * kDriveRadius) / averageWheelPosition;
				log_lastGyro.accept(lastGyroYawRads);
				log_avgWheelPos.accept(averageWheelPosition);
				log_accumGyro.accept(accumGyroYawRads);
				log_curEffWheelRad.accept(currentEffectiveWheelRadius);
			}, () -> {
				setControl(m_characterisationReq.withRotationalRate(0));
				if (Math.abs(accumGyroYawRads) <= Math.PI * 2.0) {
					System.out.println("not enough data for characterization " + accumGyroYawRads);
				} else {
					System.out.println(
						"effective wheel radius: "
							+ currentEffectiveWheelRadius
							+ " inches");
				}
			});

		return Commands.sequence(
			initialize, executeEnd);
	}

	public Command faceSpeakerTag(Supplier<SwerveRequest.FieldCentric> rqSup) {
		return applyRequest(() -> {
			// var speakerMeasurementOpt = vision.speakerTargetSupplier().get();
			if (!m_hasVisionYaw) {
				return rqSup.get();
			}

			var yawEffort = m_visionYaw.in(Radians) * 1.2;
			log_yawEffort.accept(yawEffort);

			return rqSup.get()
				.withRotationalDeadband(0)
				.withRotationalRate(yawEffort);
			}
		);
	}

	public void calculateYawErr(Optional<VisionMeasurement3d> measOpt, boolean tagsPresent) {
		if (measOpt.isPresent()) {
			var pose = measOpt.get().estimate().estimatedPose;
			var speakerTrans = AllianceFlipUtil.apply(SpeakerK.kBlueCenterOpening);
			var dist = speakerTrans.minus(pose.getTranslation());
			var desiredYaw = Math.atan2(dist.getY(), dist.getX());
			var curYaw = pose.getRotation().getZ();
			var yawErr = ((desiredYaw - curYaw) * Math.PI) % (2 * Math.PI);
			if (yawErr > 2 * Math.PI - Math.PI / 4) {
				yawErr -= 2 * Math.PI;
			}
			log_yawErrOpt.accept(Units.radiansToDegrees(yawErr));
			m_hasVisionYaw = true;
			m_visYawTimer.restart();
			m_visionYaw = Radians.of(yawErr);
		}
		m_hasVisionYaw = tagsPresent && !m_visYawTimer.hasElapsed(0.1);
		log_yawErr.accept(m_visionYaw.in(Degrees));
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

	public void setTestMode() {
		for (int i = 0; i < Modules.length; i++) {
			getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
			getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
		}
	}

	public Command resetPoseToSpeaker() {
		return runOnce(() -> {
			var startPose = AllianceFlipUtil.apply(new Pose2d(1.45, 5.5, Rotation2d.fromRadians(0)));
			seedFieldRelative(startPose);
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

				SmartDashboard.putNumberArray("desiredPose", AdvantageScopeUtil.toDoubleArr(pose));

				var curPose = getState().Pose;
				var xSpeed = m_xController.calculate(curPose.getX(), pose.getX());
				var ySpeed = m_yController.calculate(curPose.getY(), pose.getY());
				var thetaSpeed = m_thetaController.calculate(curPose.getRotation().getRadians(),
					pose.getRotation().getRadians());
				var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, pose.getRotation());

				setControl(m_autoRequest.withSpeeds(speeds));
			}
		});
	}

	public Command aim(double radians) {
		return run(() -> {
			m_desiredRot = AllianceFlipUtil.apply(Rotation2d.fromRadians(radians));
			var curPose = getState().Pose;
			var thetaSpeed = m_thetaController.calculate(curPose.getRotation().getRadians(),
				m_desiredRot.getRadians());
			var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, thetaSpeed, m_desiredRot);

			setControl(m_autoRequest.withSpeeds(speeds));
		}).until(() -> {
			boolean check = MathUtil.isNear(m_desiredRot.getDegrees(), getState().Pose.getRotation().getDegrees(), 1);
			return check;
		});
	}

	public Pose3d getPose3d() {
		var txr2d = getState().Pose.getTranslation();
		// we're on the floor. I hope. (i'm going to make the robot fly! >:D)
		return new Pose3d(txr2d.getX(), txr2d.getY(), 0, getRotation3d());
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
			() -> getState().Pose,
			m_xController,
			m_yController,
			m_thetaController,
			(speeds) -> setControl(m_autoRequest.withSpeeds(speeds)),
			shouldMirror,
			this);

		var brakeCmd = runOnce(() -> setControl(m_brake));

		return Commands.sequence(resetPoseCmd, choreoFollowCmd, brakeCmd).withName("ChoreoFollower");
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysId.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return m_sysId.dynamic(direction);
	}

	public void periodic() {
		var swerveState = getState();
		log_rotationSpeed.accept(Units.radiansToRotations(swerveState.speeds.omegaRadiansPerSecond));
		log_desiredRot.accept(m_desiredRot.getDegrees());
		log_rot.accept(swerveState.Pose.getRotation().getDegrees());
		m_poseError[0] = m_xController.getPositionError();
		m_poseError[1] = m_yController.getPositionError();
		m_poseError[2] = Units.radiansToDegrees(m_thetaController.getPositionError());
		log_poseError.accept(m_poseError);
		m_desiredPose[0] = m_xController.getSetpoint();
		m_desiredPose[1] = m_yController.getSetpoint();
		m_desiredPose[2] = Units.radiansToDegrees(m_thetaController.getSetpoint());
		log_desiredPose.accept(m_desiredPose);

		for (int i = 0; i < Modules.length; i++) {
			m_wheelVelos[i] = Math.abs(swerveState.ModuleStates[i].speedMetersPerSecond);
			m_wheelVeloTargets[i] = Math.abs(swerveState.ModuleTargets[i].speedMetersPerSecond);
			m_wheelVeloErrs[i] = Math.abs(m_wheelVeloTargets[i] - m_wheelVelos[i]);
		}
		log_wheelVelos.accept(m_wheelVelos);
		log_wheelVeloTargets.accept(m_wheelVeloTargets);
		log_wheelVeloErrors.accept(m_wheelVeloErrs);
		log_hasYaw.accept(m_hasVisionYaw);
	}
}
