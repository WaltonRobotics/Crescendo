// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.RobotK.*;

import java.util.function.Supplier;

public class Robot extends TimedRobot {
	/** 5.21 meters per second desired top speed */
	public static final double kMaxSpeed = 5;
	/** 1.5 of a rotation per second max angular velocity */
	public static final double kMaxAngularRate = 1.5 * (Math.PI * 2);

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final CommandXboxController driver = new CommandXboxController(0); // My joystick

	private final Swerve swerve = TunerConstants.drivetrain;

	public static final Field2d field2d = new Field2d();

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(kMaxSpeed * 0.1) // Add a 5% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	private final Telemetry logger = new Telemetry(kMaxSpeed);

	private Command m_autonomousCommand;

	public Robot() {
		DriverStation.silenceJoystickConnectionWarning(true);
		PhotonCamera.setVersionCheckEnabled(false);
		// disable joystick not found warnings when in sim
		if (Robot.isSimulation()) {
			DriverStation.silenceJoystickConnectionWarning(true);
		}
	}

	private Supplier<SwerveRequest.FieldCentric> getTeleSwerveReq() {
		return () -> {
			double leftY = -driver.getLeftY();
			double leftX = -driver.getLeftX();
			return drive
				.withVelocityX(leftY * kMaxSpeed)
				.withVelocityY(leftX * kMaxSpeed)
				.withRotationalRate(-driver.getRightX() * kMaxAngularRate)
				.withRotationalDeadband(kMaxAngularRate * 0.1);
		};
	}

	private void configureBindings() {
		/* drivetrain */
		if (Utils.isSimulation()) {
			swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}
		swerve.registerTelemetry(logger::telemeterize);

		/* driver controls */
		swerve.setDefaultCommand(swerve.applyFcRequest(getTeleSwerveReq()));

		// rezero
		driver.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldRelative()));
	}

	@Override
	public void robotInit() {
		configureBindings();
		if (kTestMode) {
			swerve.setTestMode();
		}
		FollowPathCommand.warmupCommand();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		swerve.logModulePositions();
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
}
