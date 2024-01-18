package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import static frc.robot.auton.Trajectories.*;

public final class AutonFactory {
	public static Command oneMeter(Swerve swerve) {
		return swerve.choreoSwerveCommand(oneMeter);
	}

	public static Command simpleThing(Swerve swerve) {
		return swerve.choreoSwerveCommand(simpleThing);
	}

	// TODO: make faster (robot should be able to shoot from farther away)
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
		var pathCmd = swerve.choreoSwerveCommand(fourPc);
		var shootCmd1 = shooter.shoot();
		var intakeCmd1 = intake.intake();
		var shootCmd2 = shooter.shoot();
		var intakeCmd2 = intake.intake();
		var shootCmd3 = shooter.shoot();
		var intakeCmd3 = intake.intake();
		var shootCmd4 = shooter.shoot();

		return Commands.sequence(
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
					shootCmd4
				)
			)
		);
	}

	public static Command fivePiece(Swerve swerve, Intake intake, Shooter shooter) {
		// TODO: check to make sure the path starts at a good place
		var pathCmd = swerve.choreoSwerveCommand(fivePc);
		var shootCmd1 = shooter.shoot().asProxy();
		var intakeCmd1 = intake.intake().asProxy();
		var shootCmd2 = shooter.shoot().asProxy();
		var intakeCmd2 = intake.intake().asProxy();
		var shootCmd3 = shooter.shoot().asProxy();
		var intakeCmd3 = intake.intake().asProxy();
		var shootCmd4 = shooter.shoot().asProxy();
		var intakeCmd4 = intake.intake().asProxy();
		var shootCmd5 = shooter.shoot().asProxy();

		return Commands.sequence(
			shootCmd1,
			intakeCmd1,
			Commands.parallel(
				pathCmd,
				Commands.sequence(
					Commands.waitSeconds(0.41),
					shootCmd2,
					Commands.waitSeconds(0.31),
					intakeCmd2,
					Commands.waitSeconds(0.28),
					shootCmd3,
					Commands.waitSeconds(0.38),
					intakeCmd3,
					shootCmd4,
					Commands.waitSeconds(1.46),
					intakeCmd4)),
			shootCmd5);
	}
}
