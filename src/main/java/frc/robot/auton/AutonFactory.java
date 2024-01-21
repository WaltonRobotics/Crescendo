package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import static frc.robot.auton.Trajectories.*;
import static frc.robot.auton.Paths.*;

import com.pathplanner.lib.auto.AutoBuilder;

public final class AutonFactory {
	private static double shooterTimeout = 0.25;
	private static double intakeTimeout = 0.25;

	public static Command oneMeter(Swerve swerve) {
		return swerve.choreoSwerveCommand(oneMeter);
	}

	public static Command simpleThing(Swerve swerve) {
		return swerve.choreoSwerveCommand(simpleThing);
	}

	// TODO make faster (robot should be able to shoot from farther away)
	public static Command threePiece(Swerve swerve, Intake intake, Shooter shooter) {
		var pathCmd1 = swerve.choreoSwerveCommand(threePc[0]).asProxy();
		var shootCmd1 = shooter.shoot().asProxy();
		var intakeCmd1 = intake.intake().asProxy();
		var shootCmd2 = shooter.shoot().asProxy();
		var pathCmd2 = swerve.choreoSwerveCommand(threePc[1]).asProxy();
		var intakeCmd2 = intake.intake().asProxy();
		var shootCmd3 = shooter.shoot().asProxy();

		return Commands.sequence(
			shootCmd1,
			Commands.parallel(
				pathCmd1,
				Commands.sequence(
					Commands.waitSeconds(0.59),
					intakeCmd1)),
			shootCmd2,
			Commands.parallel(
				pathCmd2,
				Commands.sequence(
					Commands.waitSeconds(0.57),
					intakeCmd2)),
			shootCmd3);
	}

	public static Command fourPiece(Swerve swerve, Intake intake, Shooter shooter) {
		var resetPoseCmd = swerve.resetPose(fourPcPath);
		var pathCmd = AutoBuilder.followPath(fourPcPath);
		var shootCmd1 = shooter.shoot().withTimeout(shooterTimeout).asProxy();
		var intakeCmd1 = intake.intake().withTimeout(intakeTimeout).asProxy();
		var shootCmd2 = shooter.shoot().withTimeout(shooterTimeout).asProxy();
		var intakeCmd2 = intake.intake().withTimeout(intakeTimeout).asProxy();
		var shootCmd3 = shooter.shoot().withTimeout(shooterTimeout).asProxy();
		var intakeCmd3 = intake.intake().withTimeout(intakeTimeout).asProxy();
		var shootCmd4 = shooter.shoot().withTimeout(shooterTimeout).asProxy();

		return Commands.sequence(
			resetPoseCmd,
			shootCmd1,
			Commands.parallel(
				pathCmd,
				Commands.sequence(
					// TODO: check timing :(
					Commands.waitSeconds(0.58),
					intakeCmd1,
					Commands.waitSeconds(0.29),
					shootCmd2,
					Commands.waitSeconds(1.34),
					intakeCmd2,
					Commands.waitSeconds(1.62),
					shootCmd3,
					Commands.waitSeconds(1.17),
					intakeCmd3,
					Commands.waitSeconds(1.63),
					shootCmd4)));
	}

	public static Command fivePiece(Swerve swerve, Intake intake, Shooter shooter) {
		var resetPoseCmd = swerve.resetPose(fivePcPath);
		var pathCmd = AutoBuilder.followPath(fivePcPath);
		var shootCmd1 = shooter.shoot().withTimeout(shooterTimeout).asProxy();
		var finalIntake = intake.intake().withTimeout(intakeTimeout).asProxy();
		var finalShot = shooter.shoot().withTimeout(shooterTimeout).asProxy();

		return Commands.sequence(
			resetPoseCmd,
			shootCmd1,
			intakeShotCycle(intake, shooter),
			Commands.parallel(
				pathCmd,
				Commands.sequence(
					Commands.waitSeconds(0.52),
					intakeShotCycle(intake, shooter)),
				Commands.sequence(
					Commands.waitSeconds(1.38),
					intakeShotCycle(intake, shooter)),
				Commands.sequence(
					Commands.waitSeconds(2.86),
					finalIntake)),
			finalShot);
	}

	// public static Command thing(Swerve swerve) {
	// var resetPoseCmd = swerve.resetPose(thing);
	// var pathCmd = AutoBuilder.followPath(thing);
	// return Commands.sequence(resetPoseCmd, pathCmd);
	// }

	private static Command intakeShotCycle(Intake intake, Shooter shooter) {
		var shootCmd = shooter.shoot().withTimeout(shooterTimeout).asProxy();
		var intakeCmd = intake.intake().withTimeout(intakeTimeout).asProxy();
		return Commands.parallel(intakeCmd, shootCmd);
	}
}
