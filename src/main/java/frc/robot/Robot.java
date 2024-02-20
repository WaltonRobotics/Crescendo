// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.Supplier;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {
	public static final double maxSpeed = 5; // 5 meters per second desired top speed
	public static final double maxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final CommandXboxController driver = new CommandXboxController(0); // My joystick
	private final CommandXboxController manipulator = new CommandXboxController(1);

	public final Swerve drivetrain = TunerConstants.Drivetrain;
	public final Vision vision = new Vision(drivetrain::addVisionMeasurement);
	public final Supplier<Pose3d> robotPoseSupplier = drivetrain::getPose3d;
	public final Shooter shooter = new Shooter();
	public final Aim aim = new Aim(robotPoseSupplier);
	public final Intake intake = new Intake();
	public final Climber climber = new Climber();
	public final Conveyor conveyor = new Conveyor();

	public static Translation3d speakerPose;

	public static final Field2d field2d = new Field2d();

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.Velocity); // I want field-centric driving in open loop
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final Telemetry logger = new Telemetry(maxSpeed);

	private Command m_autonomousCommand;

	public Robot() {
		// disable joystick not found warnings when in sim
		if (Robot.isSimulation()) {
			DriverStation.silenceJoystickConnectionWarning(true);
		}
		addPeriodic(vision::run, 0.01);
	}

	private void registerCommands() {
		NamedCommands.registerCommand("intake", intake.intake());
		NamedCommands.registerCommand("shoot", shooter.spinUp());
	}

	private void configureBindings() {
		drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
			double leftY = -driver.getLeftY();
			double leftX = -driver.getLeftX();
			return drive
				.withVelocityX(leftY * maxSpeed)
				.withVelocityY(leftX * maxSpeed)
				.withRotationalRate(-driver.getRightX() * maxAngularRate);
		}));

		driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
		driver.b().whileTrue(drivetrain
			.applyRequest(
				() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
		driver.x().whileTrue(drivetrain.goToAutonPose());

		// reset the field-centric heading on left bumper press
		driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
		driver.rightBumper().onTrue(drivetrain.resetPoseToSpeaker());
		driver.rightTrigger().whileTrue(shooter.spinUp());

		manipulator.rightBumper().whileTrue(intake.intake());

		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}
		drivetrain.registerTelemetry(logger::telemeterize);

		// shooter.setDefaultCommand(shooter.teleopCmd(() -> -manipulator.getRightY()));
		manipulator.x().onTrue(aim.goTo90());
		manipulator.y().onTrue(aim.goTo30());
		// climber.setDefaultCommand(climber.teleopCmd(() -> -manipulator.getLeftY()));
		aim.setDefaultCommand(aim.teleop(() -> -manipulator.getLeftY()));
		manipulator.rightTrigger().whileTrue(shooter.spinUp());
		manipulator.leftBumper().whileTrue(conveyor.convey());

		driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
	}

	public void mapAutonCommands() {
		AutonChooser.setDefaultAuton(AutonOption.DO_NOTHING);
		AutonChooser.assignAutonCommand(AutonOption.DO_NOTHING, Commands.none());
		AutonChooser.assignAutonCommand(AutonOption.ONE_METER, AutonFactory.oneMeter(drivetrain),
			Trajectories.oneMeter.getInitialPose());
		AutonChooser.assignAutonCommand(AutonOption.SIMPLE_THING, AutonFactory.simpleThing(drivetrain),
			Trajectories.simpleThing.getInitialPose());
		AutonChooser.assignAutonCommand(AutonOption.THREE_PC,
			AutonFactory.threePiece(drivetrain, intake, shooter, aim, conveyor),
			Trajectories.threePc.getInitialPose());
		AutonChooser.assignAutonCommand(AutonOption.FOUR_PC,
			AutonFactory.fourPiece(drivetrain, intake, shooter, aim, conveyor));
		AutonChooser.assignAutonCommand(AutonOption.FIVE_PC,
			AutonFactory.fivePiece(drivetrain, intake, shooter, aim, conveyor),
			Trajectories.fivePc.getInitialPose());
	}

	private Command getAutonomousCommand() {
		return AutonChooser.getChosenAutonCmd();
	}

	@Override
	public void robotInit() {
		SmartDashboard.putData(field2d);
		speakerPose = AllianceFlipUtil.apply(SpeakerK.kBlueCenterOpening);
		mapAutonCommands();
		registerCommands();
		configureBindings();
		// drivetrain.setTestMode();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		drivetrain.logModulePositions();
	}

	@Override
	public void disabledInit() {
		SignalLogger.stop();
		aim.coast();
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
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
		// TODO make better work good
		var drivePose = drivetrain.getState().Pose;
		var y = Inches.of(Math.sin(drivePose.getRotation().getRadians()));

		var speakerPose2d = AllianceFlipUtil.apply(SpeakerK.kBlueCenterOpeningPose3d.toPose2d());
		var endPose = drivePose
			.plus(new Transform2d(new Translation2d(AimK.kLength.negate(), y), new Rotation2d()));
		field2d.getObject("desAimPose").setPoses(drivePose, speakerPose2d);
		field2d.getObject("aimPose").setPoses(drivePose, endPose);
		field2d.getRobotObject().setPose(drivePose);
	}

	@Override
	public void simulationPeriodic() {
		getTrajLines();
		simulateAim();
	}
}
