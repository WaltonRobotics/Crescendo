package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
		var spinUp = shooter.subwoofer(RobotModeTriggers.autonomous().negate());
		var resetPose = swerve.resetPose(Paths.ampSide1);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide1);
		var preloadShot = preloadShot(superstructure, aim);
		var intake = superstructure.autonIntakeCmd();
		var swerveAim = swerve.aim(0);
		// TODO make this just aim
		// var aimAndSpinUp = superstructure.aimAndSpinUp(Degrees.of(3.2), true).until(superstructure.stateTrg_idle);
		var aimCmd = aim.toAngleUntilAt(Degrees.of(3.2));
		var secondShot = superstructure.autonShootReq();

		return sequence(
				parallel(
					resetPose,
					spinUp,
					preloadShot
				),
				parallel(
					intake,
					pathFollow
				),
				parallel(
					Commands.waitUntil(superstructure.stateTrg_noteReady),
					swerveAim.withTimeout(0.2)
				),
				parallel(
					aimCmd,
					secondShot
				)
			);
	}

	public static Command threePc(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var twoPc = twoPc(superstructure, shooter, swerve, aim);
		/* everything from 3 piece */
		var pathFollow = AutoBuilder.followPath(Paths.ampSide2);
		var intake = superstructure.autonIntakeCmd();
		var swerveAim = swerve.aim(0.4);
		var aimCmd = aim.toAngleUntilAt(Degrees.of(1));
		var thirdShot = superstructure.autonShootReq();
		
		return sequence(
			/* 2 piece */
			twoPc,
			/* 3 piece */
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
			);
	}

	public static Command threePointFive(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var threePc = threePc(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide3);
		var intake = superstructure.autonIntakeCmd();

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
		var resetPose = swerve.resetPose(Paths.ampSide1);
		var pathFollow1 = AutoBuilder.followPath(Paths.ampSide1);
		var pathFollow2 = AutoBuilder.followPath(Paths.ampSide2);
		var pathFollow3 = AutoBuilder.followPath(Paths.ampSide3);

		return sequence(
			resetPose,
			pathFollow1,
			pathFollow2,
			pathFollow3
		);
	}

	public static Command preloadShot(Superstructure superstructure, Aim aim) {
		var aimCmd = aim.toAngleUntilAt(kSubwooferAngle);
		var noteReady = superstructure.forceStateToNoteReady();
		var shoot = superstructure.autonShootReq();

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
