package frc.robot.auton;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

	private static Command ampTwoInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var resetPose = swerve.resetPose(Paths.ampSide1);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide1).withName("PathFollow");
		var preloadShot = preloadShot(superstructure, aim);
		var intake = superstructure.autonIntakeReq();
		var aimCmd = aim.toAngleUntilAt(Degrees.of(2.25)).asProxy(); // superstructure requires Aim so this brokey stuff
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
			thirdShotReq,
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
		var blueAim = aim.toAngleUntilAt(Degrees.of(0.5)).asProxy();
		var fourthShotReq = superstructure.autonShootReq();

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
			parallel(
				either(
					blueAim,
					redAim,
					() -> {
						var alliance = DriverStation.getAlliance();
						return alliance.isPresent() && alliance.get() == Alliance.Blue;
					}
				),
				fourthShotReq
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
		var blueAim = aim.toAngleUntilAt(Degrees.of(0.5)).asProxy();
		var fifthShotReq = superstructure.autonShootReq();

		var auton = sequence( // 3pc then (path and (wait then intake))
			four,
			waitUntil(superstructure.stateTrg_idle),
			parallel( // path and (wait then intake) 

				sequence( // wait then intake
					waitSeconds(0.8),
					intake
				),
				pathFollow
			),
			parallel(
				either(
					blueAim,
					redAim,
					() -> {
						var alliance = DriverStation.getAlliance();
						return alliance.isPresent() && alliance.get() == Alliance.Blue;
					}
				),
				fifthShotReq
			),
			waitUntil(superstructure.stateTrg_idle)
		);

		return theWrapper(auton, shooter).withName("FivePcFullAuton");
	}

	public static Command sourceTwoInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var resetPose = swerve.resetPose(Paths.sourceSide1);
		var pathFollow = AutoBuilder.followPath(Paths.sourceSide1).withName("PathFollow");
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
		
		var auton = sequence(
			three,
			parallel(
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
			thirdShotReq
		).withName("G28Counter");

		return theWrapper(auton, shooter).withName("G28CounterFull");
	}
}
