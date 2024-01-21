package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import static frc.robot.auton.Paths.*;

import com.pathplanner.lib.auto.AutoBuilder;

public final class AutonFactory {
	private static double shooterTimeout = 0.25;
	private static double intakeTimeout = 0.25;

	public static Command oneMeter(Swerve swerve) {
		return AutoBuilder.followPath(oneMeter);
	}

	public static Command simpleThing(Swerve swerve) {
		return AutoBuilder.followPath(simpleThing);
	}

	public static Command threePiece(Swerve swerve, Intake intake, Shooter shooter) {
		var pathCmd = AutoBuilder.followPath(threePc);
		var shootCmd = shooter.shoot().asProxy();

		return Commands.sequence(
			shootCmd,
			Commands.parallel(
				pathCmd,
				Commands.sequence(
					Commands.waitSeconds(0.45),
					intakeShotCycle(intake, shooter))),
			intakeShotCycle(intake, shooter));
	}

	public static Command fourPiece(Swerve swerve, Intake intake, Shooter shooter) {
		var resetPoseCmd = swerve.resetPose(fourPc);
		var pathCmd = AutoBuilder.followPath(fourPc);
		var shootCmd1 = shooter.shoot().withTimeout(shooterTimeout).asProxy();
		var intakeCmd1 = intake.intake().withTimeout(intakeTimeout).asProxy();
		var shootCmd2 = shooter.shoot().withTimeout(shooterTimeout).asProxy();
		var intakeCmd2 = intake.intake().withTimeout(intakeTimeout).asProxy();
		var shootCmd3 = shooter.shoot().withTimeout(shooterTimeout).asProxy();

		return Commands.sequence(
			resetPoseCmd,
			shootCmd1,
			Commands.parallel(
				pathCmd,
				Commands.sequence(
					// TODO: check timing :(
					Commands.waitSeconds(0.58),
					intakeShotCycle(intake, shooter),
					Commands.waitSeconds(1.34),
					intakeCmd1,
					Commands.waitSeconds(1.62),
					shootCmd2,
					Commands.waitSeconds(1.17),
					intakeCmd2,
					Commands.waitSeconds(1.63),
					shootCmd3)));
	}

	public static Command fivePiece(Swerve swerve, Intake intake, Shooter shooter) {
		var resetPoseCmd = swerve.resetPose(fivePc);
		var pathCmd = AutoBuilder.followPath(fivePc);
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
