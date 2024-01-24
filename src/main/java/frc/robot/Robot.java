// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.auton.AutonChooser;
import frc.robot.auton.AutonChooser.AutonOption;
import frc.robot.auton.AutonFactory;
import frc.robot.auton.Trajectories;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
	private double MaxSpeed = 5; // 6 meters per second desired top speed
	private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final CommandXboxController driver = new CommandXboxController(0); // My joystick
	private final CommandXboxController manipulator = new CommandXboxController(1);
	public final Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
	public final Intake intake = new Intake();
	public final Shooter shooter = new Shooter();
	public final Climber climber = new Climber();

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final Telemetry logger = new Telemetry(MaxSpeed);

	private Command m_autonomousCommand;

	private void registerCommands() {
		NamedCommands.registerCommand("intake", intake.intake());
		NamedCommands.registerCommand("shoot", shooter.shoot());
	}

	private void configureBindings() {
		drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
			double leftY = -driver.getLeftY();
			double leftYSig = Math.signum(leftY);
			leftY *= leftY;
			leftY *= leftYSig;

			double leftX = -driver.getLeftX();
			double leftXSig = Math.signum(leftX);
			leftX *= leftX;
			leftX *= leftXSig;
			return drive
				.withVelocityX(leftY * MaxSpeed)
				.withVelocityY(leftX * MaxSpeed)
				.withRotationalRate(-driver.getRightX() * MaxAngularRate);
		}));

		driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
		driver.b().whileTrue(drivetrain
			.applyRequest(
				() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
		driver.x().whileTrue(drivetrain.goToAutonPose());

		// reset the field-centric heading on left bumper press
		driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
		driver.rightBumper().onTrue(drivetrain.resetPoseToSpeaker());
		driver.rightTrigger().whileTrue(shooter.shoot());

		manipulator.rightBumper().whileTrue(intake.intake());

		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}
		drivetrain.registerTelemetry(logger::telemeterize);

		// shooter.setDefaultCommand(shooter.aimAtSpeaker(drivetrain));
		climber.setDefaultCommand(climber.teleopCmd(() -> -manipulator.getLeftY()));

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
		AutonChooser.assignAutonCommand(AutonOption.THREE_PC, AutonFactory.threePiece(drivetrain, intake, shooter),
			Trajectories.threePc.getInitialPose());
		AutonChooser.assignAutonCommand(AutonOption.FOUR_PC, AutonFactory.fourPiece(drivetrain, intake, shooter));
		AutonChooser.assignAutonCommand(AutonOption.FIVE_PC, AutonFactory.fivePiece(drivetrain, intake, shooter),
			Trajectories.fivePc.getInitialPose());
		// AutonChooser.assignAutonCommand(AutonOption.THING,
		// AutonFactory.thing(drivetrain));
	}

	private Command getAutonomousCommand() {
		return AutonChooser.getChosenAutonCmd();
	}

	@Override
	public void robotInit() {
		shooter.getSpeakerPose();
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

	@Override
	public void simulationPeriodic() {
	}
}
