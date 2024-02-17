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
	private static double spinUpTimeout = 0.25;
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
		var toAngle = Commands.repeatingSequence(aim.toTarget());
		var resetPoseCmd = swerve.resetPose(threePc);
		var pathCmd = AutoBuilder.followPath(threePc);
		var shootCmd = shooter.spinUp().withTimeout(shooterTimeout).asProxy();

		return Commands.parallel(
			toAngle,
			Commands.sequence(
				resetPoseCmd,
				aimAtSpeaker(aim),
				shootCmd,
				Commands.parallel(
					pathCmd,
					Commands.sequence(
						Commands.waitSeconds(0.45),
						aimAtSpeaker(aim),
						intakeShotCycle(intake, conveyor, shooter))),
				aimAtSpeaker(aim),
				intakeShotCycle(intake, conveyor, shooter)));
	}

	public static Command fourPiece(Swerve swerve, Intake intake, Shooter shooter, Aim aim, Conveyor conveyor) {
		var toAngle = Commands.repeatingSequence(aim.toTarget());
		var resetPoseCmd = swerve.resetPose(fourPc);
		var pathCmd = AutoBuilder.followPath(fourPc);
		var shootCmd1 = shooter.spinUp().withTimeout(shooterTimeout).asProxy();
		var intakeCmd1 = intake.intake().withTimeout(intakeTimeout).asProxy();
		var shootCmd2 = shooter.spinUp().withTimeout(shooterTimeout).asProxy();
		var intakeCmd2 = intake.intake().withTimeout(intakeTimeout).asProxy();
		var shootCmd3 = shooter.spinUp().withTimeout(shooterTimeout).asProxy();

		return Commands.parallel(
			toAngle,
			Commands.sequence(
				resetPoseCmd,
				aimAtSpeaker(aim),
				shootCmd1,
				Commands.parallel(
					pathCmd,
					Commands.sequence(
						// TODO: check timing :(
						Commands.waitSeconds(0.58),
						aimAtSpeaker(aim),
						intakeShotCycle(intake, conveyor, shooter),
						Commands.waitSeconds(1.34),
						intakeCmd1,
						Commands.waitSeconds(1.62),
						aimAtSpeaker(aim),
						shootCmd2,
						Commands.waitSeconds(1.17),
						intakeCmd2,
						Commands.waitSeconds(1.63),
						aimAtSpeaker(aim),
						shootCmd3))));
	}

	public static Command fivePiece(Swerve swerve, Intake intake, Shooter shooter, Aim aim, Conveyor conveyor) {
		var toAngle = Commands.repeatingSequence(aim.toTarget());
		var resetPoseCmd = swerve.resetPose(fivePc);
		var pathCmd = AutoBuilder.followPath(fivePc);
		var shootCmd = shooter.shoot().withTimeout(shooterTimeout).asProxy();
		var finalIntake = intake.intake().withTimeout(intakeTimeout).asProxy();
		var finalShot = shooter.spinUp().withTimeout(shooterTimeout).asProxy();

		return Commands.parallel(
			toAngle,
			Commands.sequence(
				resetPoseCmd,
				aimAtSpeaker(aim),
				shootCmd,
				Commands.parallel(
					aimAtSpeaker(aim),
					intakeShotCycle(intake, conveyor, shooter)),
				Commands.parallel(
					pathCmd,
					Commands.sequence(
						Commands.waitSeconds(0.52),
						Commands.parallel(
							aimAtSpeaker(aim),
							intakeShotCycle(intake, conveyor, shooter))),
					Commands.sequence(
						Commands.waitSeconds(1.38),
						Commands.parallel(
							aimAtSpeaker(aim),
							intakeShotCycle(intake, conveyor, shooter))),
					Commands.sequence(
						Commands.waitSeconds(2.86),
						finalIntake)),
				aimAtSpeaker(aim),
				finalShot));
	}

	private static Command intakeShotCycle(Intake intake, Conveyor conveyor, Shooter shooter) {
		// TODO add spinup
		var conveyCmd = conveyor.convey().withTimeout(conveyorTimeout).asProxy();
		var spinUpCmd = shooter.spinUp().withTimeout(spinUpTimeout).asProxy();
		var shootCmd = shooter.shoot().withTimeout(shooterTimeout).asProxy();
		var spinUpAndShoot = Commands.sequence(spinUpCmd, shootCmd).asProxy();
		var intakeCmd = intake.intake().withTimeout(intakeTimeout).asProxy();
		return Commands.parallel(conveyCmd, intakeCmd, spinUpAndShoot);
	}

	private static Command aimAtSpeaker(Aim aim) {
		return Commands.runOnce(() -> aim.aimAtSpeaker());
	}
}
