package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.Conveyor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Intake;

import static frc.robot.auton.Paths.*;

import com.pathplanner.lib.auto.AutoBuilder;

public final class AutonFactory {
	// TODO check
	private static double conveyorTimeout = 0.25;
	private static double shooterTimeout = 0.25;
	private static double intakeTimeout = 0.25;

	public static Command oneMeter(Swerve swerve) {
		return AutoBuilder.followPath(oneMeter);
	}

	public static Command simpleThing(Swerve swerve) {
		return AutoBuilder.followPath(simpleThing);
	}

	public static Command threePiece(Swerve swerve, Intake intake, Shooter shooter, Aim aim, Conveyor conveyor) {
		var resetPoseCmd = swerve.resetPose(threePc);
		var pathCmd = AutoBuilder.followPath(threePc);
		var shootCmd = shooter.shoot().withTimeout(shooterTimeout).asProxy();

		return Commands.sequence(
			resetPoseCmd,
			shootCmd,
			Commands.parallel(
				pathCmd,
				Commands.sequence(
					Commands.waitSeconds(0.45),
					intakeShotCycle(intake, shooter, conveyor))),
			intakeShotCycle(intake, shooter, conveyor));
	}

	public static Command fourPiece(Swerve swerve, Intake intake, Shooter shooter, Aim aim, Conveyor conveyor) {
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
					intakeShotCycle(intake, shooter, conveyor),
					Commands.waitSeconds(1.34),
					intakeCmd1,
					Commands.waitSeconds(1.62),
					shootCmd2,
					Commands.waitSeconds(1.17),
					intakeCmd2,
					Commands.waitSeconds(1.63),
					shootCmd3)));
	}

	public static Command fivePiece(Swerve swerve, Intake intake, Shooter shooter, Aim aim, Conveyor conveyor) {
		var resetPoseCmd = swerve.resetPose(fivePc);
		var pathCmd = AutoBuilder.followPath(fivePc);
		// var aimCmd = aim.aimAtSpeaker().asProxy();
		var shootCmd = shooter.shoot().withTimeout(shooterTimeout).asProxy();
		var finalIntake = intake.intake().withTimeout(intakeTimeout).asProxy();
		var finalShot = shooter.shoot().withTimeout(shooterTimeout).asProxy();

		return Commands.sequence(
			resetPoseCmd,
			aim.aimAtSpeaker().asProxy(),
			shootCmd,
			Commands.parallel(
				aim.aimAtSpeaker().asProxy(),
				intakeShotCycle(intake, shooter, conveyor)),
			Commands.parallel(
				pathCmd,
				Commands.sequence(
					Commands.waitSeconds(0.52),
					Commands.parallel(
						aim.aimAtSpeaker().asProxy(),
						intakeShotCycle(intake, shooter, conveyor))),
				Commands.sequence(
					Commands.waitSeconds(1.38),
					Commands.parallel(
						aim.aimAtSpeaker().asProxy(),
						intakeShotCycle(intake, shooter, conveyor))),
				Commands.sequence(
					Commands.waitSeconds(2.86),
					finalIntake)),
			aim.aimAtSpeaker().asProxy(),
			finalShot);
	}

	private static Command intakeShotCycle(Intake intake, Shooter shooter, Conveyor conveyor) {
		// TODO add spinup
		var conveyCmd = conveyor.convey().withTimeout(conveyorTimeout).asProxy();
		var shootCmd = shooter.shoot().withTimeout(shooterTimeout).asProxy();
		var intakeCmd = intake.intake().withTimeout(intakeTimeout).asProxy();
		return Commands.parallel(conveyCmd, intakeCmd, shootCmd);
	}
}
