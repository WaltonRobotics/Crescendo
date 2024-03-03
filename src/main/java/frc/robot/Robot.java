// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import org.photonvision.PhotonCamera;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AimK;
import frc.robot.Constants.FieldK.SpeakerK;
import frc.robot.auton.AutonChooser;
import frc.robot.auton.AutonChooser.AutonOption;
import frc.robot.auton.AutonFactory;
import frc.robot.auton.Trajectories;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.Conveyor;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.AllianceFlipUtil;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;

import static frc.robot.Constants.RobotK.*;

import java.util.function.Supplier;

public class Robot extends TimedRobot {
	public static final double kMaxSpeed = 5; // 5 meters per second desired top speed
	public static final double kMaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final CommandXboxController driver = new CommandXboxController(0); // My joystick
	private final CommandXboxController manipulator = new CommandXboxController(1);

	public final Swerve swerve = TunerConstants.drivetrain;
	public final Vision vision = new Vision(swerve::addVisionMeasurement);
	public final Supplier<Pose3d> robotPoseSupplier = swerve::getPose3d;
	public final Shooter shooter = new Shooter();
	public final Aim aim = new Aim(robotPoseSupplier);
	public final Intake intake = new Intake();
	public final Conveyor conveyor = new Conveyor();

	public final Superstructure superstructure = new Superstructure(
		aim, intake, conveyor, shooter,
		manipulator.leftTrigger(), driver.rightTrigger(),
		(intensity) -> driverRumble(intensity), (intensity) -> manipulatorRumble(intensity));

	public static Translation3d speakerPose;

	public static final Field2d field2d = new Field2d();

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(kMaxSpeed * 0.1).withRotationalDeadband(kMaxAngularRate * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	// .withCenterOfRotation(new Translation2d(Units.inchesToMeters(2), 0));
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final Telemetry logger = new Telemetry(kMaxSpeed);

	private Command m_autonomousCommand;

	public Robot() {
		DriverStation.silenceJoystickConnectionWarning(true);
		PhotonCamera.setVersionCheckEnabled(false);
		// disable joystick not found warnings when in sim
		if (Robot.isSimulation()) {
			DriverStation.silenceJoystickConnectionWarning(true);
		}
		addPeriodic(vision::run, 0.01);
	}

	private void mapAutonCommands() {
		AutonChooser.setDefaultAuton(AutonOption.DO_NOTHING);
		AutonChooser.assignAutonCommand(AutonOption.DO_NOTHING, Commands.none());
		AutonChooser.assignAutonCommand(AutonOption.ONE_METER, AutonFactory.oneMeter(swerve),
			Trajectories.oneMeter.getInitialPose());
		AutonChooser.assignAutonCommand(AutonOption.SIMPLE_THING, AutonFactory.simpleThing(swerve),
			Trajectories.simpleThing.getInitialPose());
		AutonChooser.assignAutonCommand(AutonOption.LEAVE, AutonFactory.leave(superstructure, swerve),
			Trajectories.leave.getInitialPose());
		AutonChooser.assignAutonCommand(AutonOption.TWO_PC, AutonFactory.twoPc(superstructure, swerve),
			Trajectories.leave.getInitialPose());
		AutonChooser.assignAutonCommand(AutonOption.THREE_PC,
			AutonFactory.threePiece(swerve, intake, shooter, aim, conveyor),
			Trajectories.threePc.getInitialPose());
		AutonChooser.assignAutonCommand(AutonOption.FOUR_PC,
			AutonFactory.fourPiece(swerve, intake, shooter, aim, conveyor));
		AutonChooser.assignAutonCommand(AutonOption.FIVE_PC,
			AutonFactory.fivePiece(swerve, intake, shooter, aim, conveyor),
			Trajectories.fivePc.getInitialPose());
	}

	private void driverRumble(double intensity) {
		driver.getHID().setRumble(RumbleType.kBothRumble, intensity);
	}

	private void manipulatorRumble(double intensity) {
		manipulator.getHID().setRumble(RumbleType.kBothRumble, intensity);
	}

	private void configureBindings() {
		/* drivetrain */
		if (Utils.isSimulation()) {
			swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}
		swerve.registerTelemetry(logger::telemeterize);

		/* driver controls */
		swerve.setDefaultCommand(swerve.applyRequest(() -> {
			double leftY = -driver.getLeftY();
			double leftX = -driver.getLeftX();
			return drive
				.withVelocityX(leftY * kMaxSpeed)
				.withVelocityY(leftX * kMaxSpeed)
				.withRotationalRate(-driver.getRightX() * kMaxAngularRate);
		}));
		driver.a().whileTrue(swerve.applyRequest(() -> brake));
		driver.b().whileTrue(swerve
			.applyRequest(
				() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(),
					-driver.getLeftX()))));
		driver.x().whileTrue(swerve.goToAutonPose());
		driver.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldRelative()));

		/* sysid buttons */
		driver.back().and(driver.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
		driver.back().and(driver.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
		driver.start().and(driver.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
		driver.start().and(driver.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

		/* characterisation buttons */
		driver.back().and(driver.a()).whileTrue(swerve.wheelRadiusCharacterisation(-1));
		driver.start().and(driver.a()).whileTrue(swerve.wheelRadiusCharacterisation(1));

		/* manipulator controls */
		manipulator.rightTrigger().whileTrue(intake.outtake());
		manipulator.b().and(manipulator.povUp()).whileTrue(conveyor.runFast());
		manipulator.rightBumper().whileTrue(superstructure.aim(() -> AimK.kSubwooferAngle, false));
		manipulator.leftBumper().whileTrue(superstructure.aim(() -> AimK.kAmpAngle, true));
		manipulator.b().and(manipulator.povDown())
			.onTrue(Commands.runOnce(() -> superstructure.forceStateToShooting()));
		manipulator.x().and((manipulator.back().and(manipulator.start())).negate()).whileTrue(aim.hardStop());

		/* testing buttons */
		manipulator.a().whileTrue(superstructure.backwardsRun());
		manipulator.povUp().onTrue(aim.increaseAngle());
		manipulator.povDown().and(manipulator.b().negate())
			.onTrue(aim.decreaseAngle());
		manipulator.povLeft().onTrue(aim.to90ish());
		manipulator.povRight().onTrue(aim.amp());

		manipulator.start().whileTrue(Commands.startEnd(
			() -> aim.setCoast(true), () -> aim.setCoast(false)));

		/* sysid buttons */
		manipulator.back().and(manipulator.y()).whileTrue(shooter.sysIdDynamic(Direction.kForward));
		manipulator.back().and(manipulator.x()).whileTrue(shooter.sysIdDynamic(Direction.kReverse));
		manipulator.start().and(manipulator.y()).whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
		manipulator.start().and(manipulator.x()).whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
	}

	private Command getAutonomousCommand() {
		return AutonChooser.getChosenAutonCmd();
	}

	@Override
	public void robotInit() {
		addPeriodic(() -> {
			superstructure.fastPeriodic();
			superstructure.sensorEventLoop.poll();
		}, 0.00125);
		SmartDashboard.putData(field2d);
		speakerPose = AllianceFlipUtil.apply(SpeakerK.kBlueCenterOpening);
		mapAutonCommands();
		configureBindings();
		if (kTestMode) {
			swerve.setTestMode();
		}
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		if (kTestMode) {
			swerve.logModulePositions();
		}
	}

	@Override
	public void disabledInit() {
		SignalLogger.stop();
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		SignalLogger.start();
		superstructure.forceStateToNoteReady();
		m_autonomousCommand = getAutonomousCommand();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		SignalLogger.start();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}

	@Override
	public void simulationPeriodic() {
		getTrajLines();
		simulateAim();
	}

	private void getTrajLines() {
		var traj = AutonChooser.getChosenTrajectory();
		var alliance = DriverStation.getAlliance();
		ChoreoTrajectory realTraj;
		if (alliance.isPresent() && alliance.get().equals(Alliance.Red)) {
			realTraj = traj.flipped();
		} else {
			realTraj = traj;
		}
		field2d.getObject("trajectory").setPoses(realTraj.getPoses());
	}

	private void simulateAim() {
		var drivePose = swerve.getState().Pose;
		var y = Inches.of(Math.sin(drivePose.getRotation().getRadians()));
		var speakerPose2d = AllianceFlipUtil.apply(SpeakerK.kBlueCenterOpeningPose3d.toPose2d());
		var endPose = drivePose
			.plus(new Transform2d(new Translation2d(AimK.kLength.negate(), y), new Rotation2d()));
		field2d.getObject("desAimPose").setPoses(drivePose, speakerPose2d);
		field2d.getObject("aimPose").setPoses(drivePose, endPose);
		field2d.getRobotObject().setPose(drivePose);
	}
}
