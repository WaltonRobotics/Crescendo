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
import frc.robot.Constants.AimK;
import frc.robot.subsystems.Superstructure;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.util.CommandDoodads.*;

import com.pathplanner.lib.auto.AutoBuilder;

public final class VisionAutonFactory {
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

	private static Command preloadShot(Superstructure superstructure, Aim aim) {
		var noteReady = superstructure.forceStateToNoteReady();
		var setTarget = aim.setTarget(AimK.kSubwooferAngle);
		var shoot = superstructure.autonShootReq();

		return sequence(
			noteReady,
			setTarget,
			parallel(
				print("aim and spin up"),
				shoot.andThen(print("shoot done"))
			),
			print("finished")
		);
	}

	private static Command theWrapper(Command auton, Shooter shooter, Aim aim) {
		return sequence(
			runOnce(() -> m_autonTimer.restart()),
			race(
                aim.aim(),
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
		var aimTarget = aim.setTarget(Degrees.of(2.25));
		var swerveAim = swerve.faceSpeakerTagAuton();
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
				pathFollow
			),
			logSeqIncr(),
			either(aimTarget, swerveAim, () -> aim.m_measurementTimer.hasElapsed(0.1)),
			secondShotReq,
			logSeqIncr(),
			waitUntil(superstructure.stateTrg_idle)
		).withName("TwoPcSequence");
	}

	public static Command ampTwo(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = ampTwoInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter, aim).withName("AmpTwoFullAuton");
	}

	private static Command ampThreeInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var two = ampTwoInternal(superstructure, shooter, swerve, aim);
		/* everything from 3 piece */
		var pathFollow = AutoBuilder.followPath(Paths.ampSide2);
		var intake = superstructure.autonIntakeReq();
		var aimTarget = aim.setTarget(Degrees.of(0));
		var swerveAim = swerve.faceSpeakerTagAuton();
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
				pathFollow
			),
			either(aimTarget, swerveAim, () -> aim.m_measurementTimer.hasElapsed(0.1)),
			thirdShotReq,
			waitUntil(superstructure.stateTrg_idle)
		).withName("ThreePcSequence");
	}

	public static Command ampThree(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = ampThreeInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter, aim).withName("AmpThreeFullAuton");
	}

	private static Command ampFourInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var three = ampThreeInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide3);
		var intake = superstructure.autonIntakeReq();
		var aimTarget = aim.setTarget(Degrees.of(1));
		var swerveAim = swerve.faceSpeakerTagAuton();
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
			either(aimTarget, swerveAim, () -> aim.m_measurementTimer.hasElapsed(0.1)),
            fourthShotReq,
			waitUntil(superstructure.stateTrg_idle)
		);
	}

	public static Command ampFour(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = ampFourInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter, aim).withName("AmpFourFullAuton");
	}

	public static Command ampFive(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var four = ampFourInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.ampSide4);
		var intake = superstructure.autonIntakeReq();
		var aimTarget = aim.setTarget(Degrees.of(1));
		var swerveAim = swerve.faceSpeakerTagAuton();
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
			either(aimTarget, swerveAim, () -> aim.m_measurementTimer.hasElapsed(0.1)),
            fifthShotReq,
			waitUntil(superstructure.stateTrg_idle)
		);

		return theWrapper(auton, shooter, aim).withName("FivePcFullAuton");
	}

	public static Command sourceTwoInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var resetPose = swerve.resetPose(Paths.sourceSide1);
		var pathFollow = AutoBuilder.followPath(Paths.sourceSide1).withName("PathFollow");
		var preloadShot = preloadShot(superstructure, aim);
		var intake = superstructure.autonIntakeReq();
		var secondShotReq = superstructure.autonShootReq();
		var swerveAim = swerve.faceSpeakerTagAuton();

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
				pathFollow.andThen(print("path follow finished"))
			),
			print("aim finished, path follow finished, should be shooting"),
			logSeqIncr(),
			swerveAim,
			secondShotReq,
			logSeqIncr(),
			waitUntil(superstructure.stateTrg_idle)
		).withName("SourceTwoPcSequence");
	}

	public static Command sourceTwo(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = sourceTwoInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter, aim).withName("SourceTwoFullAuton");
	}

	public static Command sourceThreeInternal(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var two = sourceTwoInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.sourceSide2);
		var intake = superstructure.autonIntakeReq();
		var thirdShotReq = superstructure.autonShootReq();
		var swerveAim = swerve.faceSpeakerTagAuton();
		
		return sequence(
			/* 2 piece */
			two,
			/* 3 piece */
			parallel(
				sequence(
					waitSeconds(0.8),
					intake
				),
				pathFollow
			),
			swerveAim,
			thirdShotReq,
			waitUntil(superstructure.stateTrg_idle)
		).withName("ThreePcSequence");
	}

	public static Command sourceThree(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var auton = sourceThreeInternal(superstructure, shooter, swerve, aim);
		return theWrapper(auton, shooter, aim).withName("SourceThreeFullAuton");
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

		return theWrapper(auton, shooter, aim).withName("SourceThreePointFiveFullAuton"); // what a silly and goofy long name
	}

	public static Command sourceFour(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var three = sourceThreeInternal(superstructure, shooter, swerve, aim);
		var pathFollow = AutoBuilder.followPath(Paths.sourceSide3);
		var intake = superstructure.autonIntakeReq();
		var fourthShotReq = superstructure.autonShootReq();
		
		var auton = sequence(
			three,
			parallel(
				pathFollow,
				intake
			),
			fourthShotReq
		).withName("SourceFourSequence");

		return theWrapper(auton, shooter, aim).withName("SourceFourFullAuton");
	}

	public static Command g28Counter(Superstructure superstructure, Shooter shooter, Swerve swerve, Aim aim) {
		var resetPose = swerve.resetPose(Paths.g28Counter1);
		var pathFollow = AutoBuilder.followPath(Paths.g28Counter1).withName("PathFollow");
		var preloadShot = preloadShot(superstructure, aim);
		var intake = superstructure.autonIntakeReq();
		var secondShotReq = superstructure.autonShootReq();
		var pathFollow2 = AutoBuilder.followPath(Paths.g28Counter2).withName("PathFollow2");
		var intake2 = superstructure.autonIntakeReq();
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
				pathFollow.andThen(print("path follow finished"))
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
				pathFollow2.andThen(print("path follow finished"))
			),
			thirdShotReq,
			waitUntil(superstructure.stateTrg_idle)
		).withName("G28Counter");

		return theWrapper(auton, shooter, aim).withName("G28CounterFull");
	}
}
