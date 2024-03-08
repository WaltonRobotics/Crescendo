package frc.robot.auton;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.DoubleLogger;
import frc.robot.subsystems.Superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.AimK.kPodiumAngle;
import static frc.robot.Constants.AimK.kSubwooferAngle;

import com.pathplanner.lib.auto.AutoBuilder;

public final class AutonFactory {
	public static final DoubleLogger log_state = WaltLogger.logDouble(
		"Auton", "currentCmd", PubSubOption.sendAll(true));
	public static double currentState = 0;

	private static Command parallel(Command... commands) {
		return Commands.parallel(commands);
	}

	private static Command sequence(Command... commands) {
		return Commands.sequence(commands);
	}

	public static Command oneMeter() {
		return AutoBuilder.followPath(Paths.oneMeter);
	}

	public static Command simpleThing(Swerve swerve) {
		var resetPose = swerve.resetPose(Paths.simpleThing);

		return Commands.sequence(
			resetPose, 
			AutoBuilder.followPath(Paths.simpleThing)
		);
	}

	public static Command leave(Superstructure superstructure, Shooter shooter, Swerve swerve) {
		var resetPose = swerve.resetPose(Paths.leave).asProxy();
		var shoot = shoot(superstructure, shooter);
		var pathFollow = AutoBuilder.followPath(Paths.leave).asProxy();
		
		return Commands.sequence(
			Commands.parallel(
				resetPose,
				Commands.sequence(incrementState(), shoot)
			),
			incrementState(),
			pathFollow
		);
	}

	public static Command twoPc(Superstructure superstructure, Shooter shooter, Swerve swerve) {
		var resetPose = swerve.resetPose(Paths.twoPc).asProxy();
		var pathFollow = AutoBuilder.followPath(Paths.twoPc).asProxy();
		var preloadShot = shoot(superstructure, shooter);
		var intake = superstructure.autonIntakeCmd().asProxy();
		var swerveAim = swerve.aim(0).asProxy();
		var aimAndSpinUp = superstructure.aimAndSpinUp(Degrees.of(3.2), true).until(superstructure.stateTrg_idle);
		var secondShot = superstructure.autonShootReq().asProxy();

		return Commands.sequence(
			Commands.parallel(
				resetPose, 
				preloadShot),
			Commands.parallel(
				Commands.print("going to intake now"),
				intake,
				pathFollow
			),
			Commands.parallel(
				Commands.sequence(
					Commands.waitUntil(superstructure.stateTrg_noteReady),
					Commands.print("note ready")
				),
				Commands.sequence(
					swerveAim.withTimeout(0.2),
					Commands.print("aimed")
				)
			),
			Commands.print("note ready & aimed"),
			Commands.parallel(
				aimAndSpinUp,
				secondShot
			)
		);
	}

	public static Command twoPcAmp(Superstructure superstructure, Shooter shooter, Swerve swerve) {
		var resetPose = swerve.resetPose(Paths.twoDiff).asProxy();
		var pathFollow = AutoBuilder.followPath(Paths.twoDiff).asProxy();
		var preloadShot = shoot(superstructure, shooter);
		var intake = superstructure.autonIntakeCmd().asProxy();
		var swerveAim = swerve.aim(0.4).asProxy();
		var aimAndSpinUp = superstructure.aimAndSpinUp(kPodiumAngle, true).until(superstructure.stateTrg_idle);
		var secondShot = superstructure.autonShootReq().asProxy();

		return Commands.sequence(
			Commands.parallel(
				resetPose, 
				preloadShot),
			Commands.parallel(
				Commands.print("going to intake now"),
				intake,
				pathFollow
			),
			Commands.parallel(
				Commands.sequence(
					Commands.waitUntil(superstructure.stateTrg_noteReady),
					Commands.print("note ready")
				),
				Commands.sequence(
					swerveAim.withTimeout(1),
					Commands.print("aimed")
				)
			),
			Commands.print("note ready & aimed"),
			Commands.parallel(
				aimAndSpinUp,
				secondShot
			)
		);
	}

	public static Command followThree(Swerve swerve) {
		var reset = swerve.resetPose(Paths.twoPc).asProxy();
		var pathFollow1 = AutoBuilder.followPath(Paths.twoPc).asProxy();
		var pathFollow2 = AutoBuilder.followPath(Paths.three).asProxy();

		return Commands.sequence(
			reset,
			pathFollow1,
			pathFollow2
		);
	}

	public static Command threePc(Superstructure superstructure, Shooter shooter, Swerve swerve) {
		var twoPc = twoPc(superstructure, shooter, swerve);

		/* everything from 3 piece */
		var pathFollow2 = AutoBuilder.followPath(Paths.three).asProxy();
		var intake2 = superstructure.autonIntakeCmd().asProxy();
		var swerveAim2 = swerve.aim(0.5).asProxy();
		var aimAndSpinUp2 = superstructure.aimAndSpinUp(Degrees.of(1), true);
		var thirdShot = superstructure.autonShootReq().asProxy();
		
		return Commands.sequence(
			/* 2 piece */
			twoPc,
			/* 3 piece */
			Commands.parallel(
				Commands.sequence(
					Commands.waitSeconds(0.1),
					intake2
				),
				pathFollow2
			),
			Commands.parallel(
				swerveAim2.withTimeout(0.2),
				aimAndSpinUp2,
				thirdShot
			)
		);
	}

	public static Command threePointFive(Superstructure superstructure, Shooter shooter, Swerve swerve) {
		var threePc = threePc(superstructure, shooter, swerve);

		var pathFollow = AutoBuilder.followPath(Paths.threePointFive).asProxy();
		var intake = superstructure.autonIntakeCmd().asProxy();

		return sequence( // 3pc then (path and (wait then intake))
			threePc,
			Commands.waitUntil(superstructure.stateTrg_idle),
			Commands.waitSeconds(0.1),
			parallel( // path and (wait then intake) 
				sequence( // wait then intake
					Commands.waitSeconds(0.8),
					intake
				),
				pathFollow
			)
		);
	}

	public static Command shoot(Superstructure superstructure, Shooter shooter) {
		var aimAndSpinUp = superstructure.aimAndSpinUp(kSubwooferAngle, false).until(superstructure.stateTrg_idle);
		var noteReady = superstructure.forceStateToNoteReady().asProxy();
		var shoot = Commands.runOnce(() -> superstructure.preloadShootReq()).asProxy();

		return Commands.sequence(
			noteReady,
			Commands.parallel(
				Commands.print("aim and spin up"),
				aimAndSpinUp.andThen(Commands.print("aimAndSpinUp_DONE")),
				shoot.andThen(Commands.print("shoot_DONE"))
			)
		);
	}

	private static Command incrementState() {
		return Commands.runOnce(
			() -> {
				currentState++;
				log_state.accept(
					currentState);
			}
		);
	}
}
