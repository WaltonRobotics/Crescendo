package frc.robot.auton;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.IntLogger;
import frc.robot.subsystems.Superstructure;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.AimK.kSubwooferAngle;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.AutoBuilder;

// TODO reorganise
public final class AutonFactory {
	private static IntLogger log_autonSeqInt = WaltLogger.logInt("Auton", "SequenceNum", PubSubOption.sendAll(true));
	private static int m_seqVal = 0;
	private static Command logSeqIncr() {
		return runOnce(() -> {
			log_autonSeqInt.accept(m_seqVal);
			m_seqVal++;
		});
	}

	static {
		RobotModeTriggers.disabled().onTrue(runOnce(() -> m_seqVal = 0));
	}

	private static Trigger notAuton() {
		return RobotModeTriggers.autonomous().negate();
	}

	private static Command anyShotWrapper(Command auton, Shooter shooter) {
		return race(
			shooter.subwoofer(notAuton()),
			auton
		).withName("AnyShotWrapper");
	}

	public static Command two(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = two(superstructure, shooter, swerve, aim, true);

		return anyShotWrapper(auton, shooter).withName("TwoPcFullAuton");
	}

	public static Command two(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim, boolean composed) {
		var resetPose = swerve.resetPose(Paths.ampSide1);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide1).withName("PathFollow");
		var preloadShot = preloadShot(superstructure, aim);
		var intake = superstructure.autonIntakeCmd();
		var aimCmd = aim.toAngleUntilAt(Degrees.of(4.5)).asProxy(); // superstructure requires Aim so this brokey stuff
		var secondShotReq = superstructure.autonShootReq();

		return sequence(
			logSeqIncr(),
			parallel(
				resetPose,
				preloadShot
			),
			print("resetPose and preloadShot done"),
			logSeqIncr(),
			waitUntil(superstructure.stateTrg_idle),
			logSeqIncr(),
			parallel(
				print("should be intaking"),
				intake,
				aimCmd,
				pathFollow
			),
			logSeqIncr(),
			secondShotReq,
			logSeqIncr(),
			waitUntil(superstructure.stateTrg_idle)
		).withName("TwoPcSequence");
	}

	public static Command three(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim, boolean composed) {
		var twoPc = two(superstructure, shooter, swerve, aim, true);
		/* everything from 3 piece */
		var pathFollow = AutoBuilder.followPath(Paths.ampSide2);
		var intake = superstructure.autonIntakeCmd();
		// var swerveAim = swerve.aim(0.4);
		var aimCmd = aim.toAngleUntilAt(Degrees.of(1.5)).asProxy();
		var thirdShotReq = superstructure.autonShootReq();
		
		return sequence(
			/* 2 piece */
			twoPc,
			/* 3 piece */
			parallel(
				sequence(
					waitSeconds(0.1),
					intake
				),
				pathFollow
			),
			aimCmd,
			thirdShotReq,
			waitUntil(superstructure.stateTrg_idle)
		).withName("ThreePcSequence");
	}

		public static Command three(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = three(superstructure, shooter, swerve, aim, true);

		return anyShotWrapper(auton, shooter).withName("ThreePcFullAuton");
	}


	public static Command four(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = four(superstructure, shooter, swerve, aim, true);

		return anyShotWrapper(auton, shooter).withName("FourFullAuton");
	}

	public static Command four(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim, boolean composed) {
		var threePc = three(superstructure, shooter, swerve, aim, true);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide3);
		var intake = superstructure.autonIntakeCmd();
		var aimCmd = aim.toAngleUntilAt(Degrees.of(2.5)).asProxy();
		var fourthShotReq = superstructure.autonShootReq();

		return sequence( // 3pc then (path and (wait then intake))
			threePc,
			waitUntil(superstructure.stateTrg_idle),
			parallel( // path and (wait then intake) 
				sequence( // wait then intake
					waitSeconds(0.8),
					intake
				),
				aimCmd,
				pathFollow
			),
			fourthShotReq,
			waitUntil(superstructure.stateTrg_idle)
		);
	}

	public static Command five(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var fourPc = four(superstructure, shooter, swerve, aim, true);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide4);
		var intake = superstructure.autonIntakeCmd();
		var aimCmd = aim.toAngleUntilAt(Degrees.of(2.5)).asProxy();
		var fifthShotReq = superstructure.autonShootReq();

		var auton = sequence( // 3pc then (path and (wait then intake))
			fourPc,
			waitUntil(superstructure.stateTrg_idle),
			parallel( // path and (wait then intake) 
				sequence( // wait then intake
					waitSeconds(0.8),
					intake
				),
				aimCmd,
				pathFollow
			),
			fifthShotReq,
			waitUntil(superstructure.stateTrg_idle)
		);

		return anyShotWrapper(auton, shooter).withName("FivePcFullAuton");
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
		var aimCmd = aim.toAngleUntilAt(kSubwooferAngle).withTimeout(1).asProxy();
		var noteReady = superstructure.forceStateToNoteReady();
		var shoot = superstructure.autonShootReq();

		return sequence(
			noteReady,
			parallel(
				print("aim and spin up"),
				aimCmd.andThen(print("aim done")),
				shoot.andThen(print("shoot done"))
			),
			print("finished")
		);
	}
}
