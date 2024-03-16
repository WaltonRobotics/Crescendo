package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.AimK.kSubwooferAngle;

import com.pathplanner.lib.auto.AutoBuilder;

public final class AutonFactory {
	private static Command parallel(Command... commands) {
		return Commands.parallel(commands);
	}

	private static Command sequence(Command... commands) {
		return Commands.sequence(commands);
	}

	public static Command twoPc(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var spinUp = shooter.subwoofer().asProxy();
		var resetPose = swerve.resetPose(Paths.ampSide1).asProxy();
		var pathFollow = AutoBuilder.followPath(Paths.ampSide1).asProxy();
		var preloadShot = preloadShot(superstructure, aim, shooter);
		var intake = superstructure.autonIntakeCmd().asProxy();
		var swerveAim = swerve.aim(0).asProxy();
		// TODO make this just aim
		// var aimAndSpinUp = superstructure.aimAndSpinUp(Degrees.of(3.2), true).until(superstructure.stateTrg_idle);
		var aimCmd = aim.toAngleUntilAt(Degrees.of(3.2), Degrees.of(1));
		var secondShot = superstructure.autonShootReq().asProxy();

		return Commands.race(
			spinUp,
			sequence(
				parallel(
					resetPose, 
					preloadShot
				),
				parallel(
					Commands.print("going to intake now"),
					intake,
					pathFollow
				),
				parallel(
					sequence(
						Commands.waitUntil(superstructure.stateTrg_noteReady),
						Commands.print("note ready")
					),
					sequence(
						swerveAim.withTimeout(0.2),
						Commands.print("aimed")
					)
				),
				Commands.print("note ready & aimed"),
				parallel(
					aimCmd,
					secondShot
				)
			)
		);
	}

	public static Command threePc(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var twoPc = twoPc(superstructure, shooter, swerve, aim);
		/* everything from 3 piece */
		var spinUp = shooter.subwoofer().asProxy();
		var pathFollow = AutoBuilder.followPath(Paths.ampSide2).asProxy();
		var intake = superstructure.autonIntakeCmd().asProxy();
		var swerveAim = swerve.aim(0.4).asProxy();
		// var aimAndSpinUp = superstructure.aimAndSpinUp(Degrees.of(1), true);
		var aimCmd = aim.toAngleUntilAt(Degrees.of(1), Degrees.of(1));
		var thirdShot = superstructure.autonShootReq().asProxy();
		
		return sequence(
			/* 2 piece */
			twoPc,
			/* 3 piece */
			Commands.race(
				spinUp,
				parallel(
					sequence(
						Commands.waitSeconds(0.1),
						intake
					),
					pathFollow
				),
				parallel(
					swerveAim.withTimeout(0.3),
					aimCmd,
					thirdShot
				)
			)
		);
	}

	public static Command threePointFive(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var threePc = threePc(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide3).asProxy();
		var intake = superstructure.autonIntakeCmd().asProxy();

		return sequence( // 3pc then (path and (wait then intake))
			threePc,
			Commands.waitUntil(superstructure.stateTrg_idle),
			parallel( // path and (wait then intake) 
				sequence( // wait then intake
					Commands.waitSeconds(0.8),
					intake
				),
				pathFollow
			)
		);
	}

	public static Command followThreePointFive(Swerve swerve) {
		var resetPose = swerve.resetPose(Paths.ampSide1).asProxy();
		var pathFollow1 = AutoBuilder.followPath(Paths.ampSide1).asProxy();
		var pathFollow2 = AutoBuilder.followPath(Paths.ampSide2).asProxy();
		var pathFollow3 = AutoBuilder.followPath(Paths.ampSide3).asProxy();

		return sequence(
			resetPose,
			pathFollow1,
			pathFollow2,
			pathFollow3
		);
	}

	public static Command preloadShot(Superstructure superstructure, Aim aim, Shooter shooter) {
		var aimCmd = aim.toAngleUntilAt(kSubwooferAngle, Degrees.of(1));
		var noteReady = superstructure.forceStateToNoteReady().asProxy();
		var shoot = Commands.runOnce(() -> superstructure.preloadShootReq()).asProxy();

		return Commands.sequence(
			noteReady,
			Commands.parallel(
				Commands.print("aim and spin up"),
				aimCmd.andThen(Commands.print("aim done")),
				shoot.andThen(Commands.print("shoot_DONE"))
			)
		);
	}
}
