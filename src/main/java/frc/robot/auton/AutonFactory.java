package frc.robot.auton;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
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
import java.util.Set;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.AutoBuilder;

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
	private static Timer m_shotTimer = new Timer();
	public static void beginTimers() {
		m_autonTimer.restart();
		m_shotTimer.restart();
	}

	static {
		RobotModeTriggers.disabled().onTrue(runOnce(() -> m_seqVal = 0));
	}

	private static Trigger notAuton() {
		return RobotModeTriggers.autonomous().negate();
	}

	private static Command printLater(Supplier<String> stringSup) {
		return defer(() -> {
			return print(stringSup.get());
		}, Set.of());
	}

	private static Command logTimer(String epochName, Supplier<Timer> timerSup) {
		return defer(() -> {
			var timer = timerSup.get();
			return printLater(() -> epochName + " at " + timer.get() + " s");
		}, Set.of());
	}

	private static Command preloadShot(Superstructure superstructure, Aim aim) {
		var aimCmd = aim.toAngleUntilAt(kSubwooferAngle, kSubwooferAngle.times(0.4)).withTimeout(1).asProxy();
		var noteReady = superstructure.forceStateToNoteReady();
		var shoot = superstructure.autonShootReq();

		return sequence(
			runOnce(() -> m_shotTimer.restart()),
			noteReady,
			parallel(
				print("aim and spin up"),
				race(
					aimCmd.andThen(print("aim done")),
					race(
						sequence(
							waitSeconds(0.15),
							superstructure.forceStateToShooting()
						),
						waitUntil(superstructure.extStateTrg_shooting.or(superstructure.stateTrg_idle))
					),
					shoot.andThen(print("shoot done"))
				)
			),
			logTimer("PreloadShot", () -> m_shotTimer)
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

	private static Command closeTwoInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var resetPose = swerve.resetPose(Paths.close1);
		var pathFollow = AutoBuilder.followPath(Paths.close1).withName("PathFollow");
		var preloadShot = preloadShot(superstructure, aim);
		var intake = superstructure.autonIntakeReq();
		var secondShotReq = superstructure.autonShootReq();

		return sequence(
			parallel(
				resetPose,
				preloadShot
			),
			parallel(
				intake,
				pathFollow
			),
			secondShotReq,
			logTimer("SecondShot", () -> m_shotTimer),
			waitUntil(superstructure.stateTrg_idle)
		).withName("CloseTwoPcSequence");
	}

	public static Command closeTwo(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = closeTwoInternal(superstructure, shooter, swerve, aim);

		return theWrapper(auton, shooter).withName("CloseTwoPcFullAuton");
	}

	private static Command closeThreeInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var two = closeTwoInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.close2);
		var intake = superstructure.autonIntakeReq();
		var thirdShotReq = superstructure.autonShootReq();

		return sequence(
			two,
			parallel(
				pathFollow,
				intake
			),
			thirdShotReq,
			logTimer("ThirdShot", () -> m_shotTimer),
			waitUntil(superstructure.stateTrg_idle)
		).withName("CloseThreePcSequence");
	}

	public static Command closeThree(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = closeThreeInternal(superstructure, shooter, swerve, aim);

		return theWrapper(auton, shooter).withName("CloseThreePcFullAuton");
	}

	private static Command closeFourInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var three = closeThreeInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.close2);
		var intake = superstructure.autonIntakeReq();
		var fourthShotReq = superstructure.autonShootReq();

		return sequence(
			three,
			parallel(
				pathFollow,
				intake
			),
			fourthShotReq,
			logTimer("FourthShot", () -> m_shotTimer),
			waitUntil(superstructure.stateTrg_idle)
		).withName("CloseFourPcSequence");
	}

	public static Command closeFour(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = closeFourInternal(superstructure, shooter, swerve, aim);

		return theWrapper(auton, shooter).withName("CloseFourPcFullAuton");
	}

	public static Command one(Superstructure superstructure, Shooter shooter, Aim aim) {
		var auton = preloadShot(superstructure, aim);

		return theWrapper(auton, shooter);
	}

	public static Command ampPointFive(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var resetPose = swerve.resetPose(Paths.ampSide1);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide1).withName("PathFollow");
		var intake = superstructure.autonIntakeReq();

		var auton = sequence(
			resetPose,
			parallel(
				print("should be intaking"),
				intake,
				pathFollow
			)).withName("PointFivePcSequence");

		return theWrapper(auton, shooter).withName("PointFiveFullAuton");
	}

	private static Command ampTwoInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var resetPose = swerve.resetPose(Paths.ampSide1);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide1).withName("PathFollow");
		var preloadShot = preloadShot(superstructure, aim);
		var intake = superstructure.autonIntakeReq();
		var aimCmd = aim.toAngleUntilAt(Degrees.of(2.25)).asProxy(); // superstructure requires Aim so this brokey stuff
		var secondShotReq = superstructure.autonShootReq();
		// var swerveAim = swerve.faceSpeakerTagAuton().withTimeout(0.5);

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
			// swerveAim,
			logSeqIncr(),
			secondShotReq,
			race(
				sequence(
					waitSeconds(0.5),
					superstructure.forceStateToShooting()
				),
				waitUntil(superstructure.extStateTrg_shooting.or(superstructure.stateTrg_idle))
			),
			logSeqIncr(),
			waitUntil(superstructure.stateTrg_idle)
		).withName("TwoPcSequence");
	}

	public static Command ampTwo(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = ampTwoInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter).withName("AmpTwoFullAuton");
	}

	private static Command ampThreeInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var two = ampTwoInternal(superstructure, shooter, swerve, aim);
		/* everything from 3 piece */
		var pathFollow = AutoBuilder.followPath(Paths.ampSide2);
		var intake = superstructure.autonIntakeReq();
		// var swerveAim = swerve.aim(0.4);
		var aimCmd = aim.toAngleUntilAt(Degrees.of(0)).asProxy();
		// var swerveAim = swerve.faceSpeakerTagAuton().withTimeout(0.5);
		var thirdShotReq = superstructure.autonShootReq();
		
		return sequence(
			/* 2 piece */
			two,
			/* 3 piece */
			parallel(
				sequence(
					waitSeconds(0.1),
					intake
				),
				pathFollow,
				aimCmd
			),
			// swerveAim,
			thirdShotReq,
			race(
				sequence(
					waitSeconds(0.5),
					superstructure.forceStateToShooting()
				),
				waitUntil(superstructure.extStateTrg_shooting.or(superstructure.stateTrg_idle))
			),
			waitUntil(superstructure.stateTrg_idle)
		).withName("ThreePcSequence");
	}

	public static Command ampThree(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = ampThreeInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter).withName("AmpThreeFullAuton");
	}

	private static Command ampFourInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var three = ampThreeInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide3);
		var intake = superstructure.autonIntakeReq();
		var redAim = aim.toAngleUntilAt(Degrees.of(1)).asProxy();
		var fourthShotReq = superstructure.autonShootReq();
		// var swerveAim = swerve.faceSpeakerTagAuton().withTimeout(0.5);

		return sequence( // 3pc then (path and (wait then intake))
			three,
			waitUntil(superstructure.stateTrg_idle),
			parallel( // path and (wait then intake) 
				sequence( // wait then intake
					waitSeconds(0.8),
					intake
				),
				pathFollow
			),
			// swerveAim,
			parallel(
				redAim,
				fourthShotReq
			),
			race(
				sequence(
					waitUntil(superstructure.stateTrg_noteReady),
					waitSeconds(0.5),
					superstructure.forceStateToShooting()
				),
				waitUntil(superstructure.extStateTrg_shooting.or(superstructure.stateTrg_idle))
			),
			waitUntil(superstructure.stateTrg_idle)
		);
	}

	public static Command ampFour(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = ampFourInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter).withName("AmpFourFullAuton");
	}

	public static Command ampFive(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var four = ampFourInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide4);
		var intake = superstructure.autonIntakeReq();
		var redAim = aim.toAngleUntilAt(Degrees.of(1)).asProxy();
		var fifthShotReq = superstructure.autonShootReq();
		var swerveAim = swerve.faceSpeakerTagAuton().withTimeout(0.5);

		var auton = sequence( // 3pc then (path and (wait then intake))
			four,
			parallel( // path and (wait then intake) 
				sequence( // wait then intake
					waitSeconds(0.8),
					intake
				),
				pathFollow
			),
			swerveAim,
			parallel(
				redAim,
				fifthShotReq
			),
			race(
				sequence(
					waitUntil(superstructure.stateTrg_noteReady),
					waitSeconds(0.5),
					superstructure.forceStateToShooting()
				),
				waitUntil(superstructure.extStateTrg_shooting.or(superstructure.stateTrg_idle))
			),
			waitUntil(superstructure.stateTrg_idle)
		);

		return theWrapper(auton, shooter).withName("FivePcFullAuton");
	}

	private static Command sillyFiveInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var three = ampThreeInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.ampSideAlt1);
		var intake = superstructure.autonIntakeReq();
		var resetFlags = runOnce(() -> superstructure.resetAutonFlags());
		var redAim = aim.toAngleUntilAt(Degrees.of(1)).asProxy();
		var pathFollow2 = AutoBuilder.followPath(Paths.ampSideAlt5);
		var intake2 = superstructure.autonIntakeReq();

		return sequence(
			three,
			parallel(
				redAim,
				sequence(
					waitSeconds(0.8),
					intake
				),
				pathFollow
			),
			resetFlags,
			either(
				skipNote(superstructure, shooter, swerve, aim), 
				dontSkipNote(superstructure, shooter, swerve, aim), 
				superstructure.stateTrg_idle
			),
			parallel(
				pathFollow2,
				sequence(
					waitSeconds(0.8),
					intake2
				)
			)
		);
	}

	public static Command sillyFive(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = sillyFiveInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter);
	}

	private static Command skipNote(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var pathFollow = AutoBuilder.followPath(Paths.ampSideSkip1);
		var intake = superstructure.autonIntakeReq();
		var pathFollow2 = AutoBuilder.followPath(Paths.ampSideAlt4);
		var shoot = superstructure.autonShootReq();

		return sequence(
			parallel(
				sequence(
					waitSeconds(0.4),
					intake
				),
				pathFollow
			),
			pathFollow2,
			shoot
		);
	}

	private static Command dontSkipNote(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var pathFollow = AutoBuilder.followPath(Paths.ampSideAlt2);
		var shoot = superstructure.autonShootReq();
		var pathFollow2 = AutoBuilder.followPath(Paths.ampSideAlt3);
		var intake = superstructure.autonIntakeReq();
		var pathFollow3 = AutoBuilder.followPath(Paths.ampSideAlt4);
		var shoot2 = superstructure.autonShootReq();

		return sequence(
			pathFollow,
			shoot,
			parallel(
				sequence(
					waitSeconds(0.8),
					intake
				),
				pathFollow2
			),
			pathFollow3,
			shoot2
		);
	}

	public static Command sourceTwoInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var resetPose = swerve.resetPose(Paths.sourceSide1);
		var pathFollow = AutoBuilder.followPath(Paths.sourceSide1).withName("PathFollow");
		var preloadShot = preloadShot(superstructure, aim);
		var intake = superstructure.autonIntakeReq();
		var aimCmd = aim.toAngleUntilAt(Degrees.of(0)).asProxy(); // superstructure requires Aim so this brokey stuff
		var secondShotReq = superstructure.autonShootReq();
		var swerveAim = swerve.faceSpeakerTagAuton().withTimeout(0.5);

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
					waitSeconds(1.5),
					intake
				),
				pathFollow.andThen(print("path follow finished")),
				aimCmd.until(superstructure.trg_atAngle)
			),
			print("aim finished, path follow finished, should be shooting"),
			logSeqIncr(),
			swerveAim,
			secondShotReq,
			race(
				sequence(
					waitSeconds(0.5),
					superstructure.forceStateToShooting()
				),
				waitUntil(superstructure.stateTrg_shooting)
			),
			logSeqIncr(),
			waitUntil(superstructure.stateTrg_idle)
		).withName("SourceTwoPcSequence");
	}

	public static Command sourceTwo(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = sourceTwoInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter).withName("SourceTwoFullAuton");
	}

	public static Command sourceThreeInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var two = sourceTwoInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.sourceSide2);
		var intake = superstructure.autonIntakeReq();
		// var swerveAim = swerve.aim(0.4);
		var aimCmd = aim.toAngleUntilAt(Degrees.of(0)).asProxy();
		var thirdShotReq = superstructure.autonShootReq();
		var swerveAim = swerve.faceSpeakerTagAuton().withTimeout(0.5);
		
		return sequence(
			/* 2 piece */
			two,
			/* 3 piece */
			parallel(
				sequence(
					waitSeconds(0.8),
					intake
				),
				pathFollow,
				aimCmd.until(superstructure.trg_atAngle)
			),
			swerveAim,
			thirdShotReq,
			race(
				sequence(
					waitSeconds(0.5),
					superstructure.forceStateToShooting()
				),
				waitUntil(superstructure.stateTrg_shooting)
			),
			waitUntil(superstructure.stateTrg_idle)
		).withName("ThreePcSequence");
	}

	public static Command sourceThree(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = sourceThreeInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter).withName("SourceThreeFullAuton");
	}

	public static Command sourceThreePointFive(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var three = sourceThreeInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.sourceSideAlt1);
		var intake = superstructure.autonIntakeReq();
		var hardStop = aim.hardStop().asProxy();
		
		var auton = sequence(
			three,
			parallel(
				hardStop,
				pathFollow,
				sequence(
					waitSeconds(0.8),
					intake
				)
			)
		).withName("SourceThreePointFiveSequence");

		return theWrapper(auton, shooter).withName("SourceThreePointFiveFullAuton"); // what a silly and goofy long name
	}

	public static Command sourceFour(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var three = sourceThreeInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.sourceSide3);
		var intake = superstructure.autonIntakeReq();
		var aimCmd = aim.toAngleUntilAt(Degrees.of(20)).asProxy();
		var fourthShotReq = superstructure.autonShootReq();
		
		var auton = sequence(
			three,
			parallel(
				pathFollow,
				intake,
				aimCmd
			),
			race(
				sequence(
					waitSeconds(0.5),
					superstructure.forceStateToShooting()
				),
				waitUntil(superstructure.stateTrg_shooting)
			),
			fourthShotReq
		).withName("SourceFourSequence");

		return theWrapper(auton, shooter).withName("SourceFourFullAuton"); // what a silly and goofy long name
	}

	public static Command g28Counter(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var resetPose = swerve.resetPose(Paths.g28Counter1);
		var pathFollow = AutoBuilder.followPath(Paths.g28Counter1).withName("PathFollow");
		var preloadShot = preloadShot(superstructure, aim);
		var intake = superstructure.autonIntakeReq();
		var aimCmd = aim.toAngleUntilAt(Degrees.of(1)).asProxy(); // superstructure requires Aim so this brokey stuff
		var secondShotReq = superstructure.autonShootReq();
		var pathFollow2 = AutoBuilder.followPath(Paths.g28Counter2).withName("PathFollow2");
		var intake2 = superstructure.autonIntakeReq();
		var aimCmd2 = aim.toAngleUntilAt(Degrees.of(1)).asProxy(); // superstructure requires Aim so this brokey stuff
		var thirdShotReq = superstructure.autonShootReq();

		var auton = sequence(
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
					waitSeconds(1.5),
					intake
				),
				pathFollow.andThen(print("path follow finished")),
				aimCmd.until(superstructure.trg_atAngle)
			),
			print("aim finished, path follow finished, should be shooting"),
			logSeqIncr(),
			secondShotReq,
			race(
				sequence(
					waitSeconds(0.5),
					superstructure.forceStateToShooting()
				),
				waitUntil(superstructure.stateTrg_shooting)
			),
			logSeqIncr(),
			waitUntil(superstructure.stateTrg_idle),
			parallel(
				sequence(
					waitSeconds(1.5),
					intake2
				),
				pathFollow2.andThen(print("path follow finished")),
				aimCmd2.until(superstructure.trg_atAngle)
			),
			thirdShotReq,
			race(
				sequence(
					waitSeconds(0.5),
					superstructure.forceStateToShooting()
				),
				waitUntil(superstructure.stateTrg_shooting)
			),
			waitUntil(superstructure.stateTrg_idle)
		).withName("G28Counter");

		return theWrapper(auton, shooter).withName("G28CounterFull");
	}

	private static Command veryAmpTwoInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var resetPose = swerve.resetPose(Paths.veryAmp1);
		var pathFollow = AutoBuilder.followPath(Paths.veryAmp1).withName("PathFollow");
		var preloadShot = preloadShot(superstructure, aim);
		var intake = superstructure.autonIntakeReq();
		var aimCmd = aim.toAngleUntilAt(Degrees.of(0)).asProxy(); // superstructure requires Aim so this brokey stuff
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
					waitSeconds(1.5),
					intake
				),
				pathFollow.andThen(print("path follow finished")),
				aimCmd.until(superstructure.trg_atAngle)
			),
			print("aim finished, path follow finished, should be shooting"),
			logSeqIncr(),
			secondShotReq,
			race(
				sequence(
					waitSeconds(0.5),
					superstructure.forceStateToShooting()
				),
				waitUntil(superstructure.stateTrg_shooting)
			),
			logSeqIncr(),
			waitUntil(superstructure.stateTrg_idle)
		).withName("TwoPcSequence");
	}

	public static Command veryAmpTwo(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = veryAmpTwoInternal(superstructure, shooter, swerve, aim);

		return theWrapper(auton, shooter).withName("VeryAmpTwoFullAuton");
	}

	private static Command veryAmpThreeInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var two = veryAmpTwoInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.veryAmp2);
		var intake = superstructure.autonIntakeReq();
		// var swerveAim = swerve.aim(0.4);
		var aimCmd = aim.toAngleUntilAt(Degrees.of(1)).asProxy();
		var thirdShotReq = superstructure.autonShootReq();
		
		return sequence(
			/* 2 piece */
			two,
			/* 3 piece */
			parallel(
				sequence(
					waitSeconds(0.8),
					intake
				),
				pathFollow,
				aimCmd.until(superstructure.trg_atAngle)
			),
			thirdShotReq,
			race(
				sequence(
					waitSeconds(0.5),
					superstructure.forceStateToShooting()
				),
				waitUntil(superstructure.stateTrg_shooting)
			),
			waitUntil(superstructure.stateTrg_idle)
		).withName("ThreePcSequence");
	}

	public Command veryAmpThree(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = veryAmpThreeInternal(superstructure, shooter, swerve, aim);

		return theWrapper(auton, shooter).withName("VeryAmpThreeFullAuton");
	}

	public static Command veryAmpThreePointFive(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var three = veryAmpThreeInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.veryAmp3);
		var intake = superstructure.autonIntakeReq();
		
		var auton = sequence(
			three,
			parallel(
				pathFollow,
				sequence(
					waitSeconds(0.8),
					intake
				)
			)
		).withName("VeryAmpThreePointFiveSequence");

		return theWrapper(auton, shooter).withName("VeryAmpThreePointFiveFullAuton"); // what a silly and goofy long name
	}

	public static Command followAmpSide(Swerve swerve) {
		var resetPose = swerve.resetPose(Paths.veryAmp1);
		var pathFollow = AutoBuilder.followPath(Paths.veryAmp1);

		return sequence(
			resetPose,
			pathFollow
		);
	}
}
