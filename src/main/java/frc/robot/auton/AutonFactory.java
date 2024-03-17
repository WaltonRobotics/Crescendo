package frc.robot.auton;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import java.util.Set;
import java.util.function.Supplier;

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

	private static Timer m_autonTimer = new Timer();

	static {
		RobotModeTriggers.disabled().onTrue(runOnce(() -> m_seqVal = 0));
	}

	private static Trigger notAuton() {
		return RobotModeTriggers.autonomous().negate();
	}

	private static Command printLater(Supplier<String> stringSup) {
		return Commands.defer(() -> {
			return print(stringSup.get());
		}, Set.of());
	}

	private static Command preloadShot(Superstructure superstructure, Aim aim) {
		var aimCmd = aim.toAngleUntilAt(kSubwooferAngle, kSubwooferAngle.times(0.2)).withTimeout(1).asProxy();
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

	private static Command theWrapper(Command auton, Shooter shooter) {
		return sequence(
			runOnce(() -> m_autonTimer.restart()),
			race(
				shooter.subwoofer(notAuton()),
				auton
			),
			runOnce(() -> m_autonTimer.stop()),
			printLater(() -> "Auton Complete in " + m_autonTimer.get() + " seconds")
		).withName("TheAutonWrapper");
	}

	private static Command twoInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
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

	public static Command two(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = twoInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter).withName("TwoPcFullAuton");
	}

	private static Command threeInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var twoPc = twoInternal(superstructure, shooter, swerve, aim);
		/* everything from 3 piece */
		var pathFollow = AutoBuilder.followPath(Paths.ampSide2);
		var intake = superstructure.autonIntakeCmd();
		// var swerveAim = swerve.aim(0.4);
		var aimCmd = aim.toAngleUntilAt(Degrees.of(1.52)).asProxy();
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
		var auton = threeInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter).withName("ThreePcFullAuton");
	}

	private static Command fourInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var threePc = threeInternal(superstructure, shooter, swerve, aim);
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

	public static Command four(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = fourInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter).withName("FourFullAuton");
	}

	public static Command five(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var fourPc = fourInternal(superstructure, shooter, swerve, aim);
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

		return theWrapper(auton, shooter).withName("FivePcFullAuton");
	}

	public static Command clearTwoInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var resetPose = swerve.resetPose(Paths.center1);
		var pathFollow = AutoBuilder.followPath(Paths.center1).withName("PathFollow");
		var preloadShot = preloadShot(superstructure, aim);
		var intake = superstructure.autonIntakeCmd();
		var aimCmd = aim.toAngleUntilAt(Degrees.of(2.5)).asProxy(); // superstructure requires Aim so this brokey stuff
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
				sequence(
					waitSeconds(0.8),
					intake
				),
				pathFollow
			),
			aimCmd,
			logSeqIncr(),
			secondShotReq,
			logSeqIncr(),
			waitUntil(superstructure.stateTrg_idle)
		).withName("ClearTwoPcSequence");
	}

	public static Command clearTwo(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = clearTwoInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter).withName("ClearTwoFullAuton");
	}

	public static Command clearThreeInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var twoPc = clearTwoInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.center2);
		var intake = superstructure.autonIntakeCmd();
		// var swerveAim = swerve.aim(0.4);
		var aimCmd = aim.toAngleUntilAt(Degrees.of(2.5)).asProxy();
		var thirdShotReq = superstructure.autonShootReq();
		
		return sequence(
			/* 2 piece */
			twoPc,
			/* 3 piece */
			parallel(
				sequence(
					waitSeconds(0.8),
					intake
				),
				pathFollow
			),
			aimCmd,
			thirdShotReq,
			waitUntil(superstructure.stateTrg_idle)
		).withName("ThreePcSequence");
	}

	public static Command clearThree(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = clearThreeInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter).withName("ClearThreeFullAuton");
	}

	public static Command clearFour(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var threePc = clearThreeInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.center3);
		var intake = superstructure.autonIntakeCmd();
		var aimCmd = aim.toAngleUntilAt(Degrees.of(2.5)).asProxy();
		var fourthShotReq = superstructure.autonShootReq();

		var auton = sequence( // 3pc then (path and (wait then intake))
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

		return theWrapper(auton, shooter).withName("ClearFourFullAuton");
	}
}
