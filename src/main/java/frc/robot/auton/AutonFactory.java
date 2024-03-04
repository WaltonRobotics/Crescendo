package frc.robot.auton;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.Conveyor;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.DoubleLogger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;

// import static frc.robot.Constants.AimK.kPodiumAngle;
import static frc.robot.Constants.AimK.kSubwooferAngle;
import static frc.robot.auton.Paths.*;

import com.pathplanner.lib.auto.AutoBuilder;

public final class AutonFactory {
	// TODO check
	private static double spinUpTimeout = 0.25;
	private static double conveyorTimeout = 0.25;
	private static double shooterTimeout = 0.25;
	private static double intakeTimeout = 0.25;

	public static DoubleLogger log_state = WaltLogger.logDouble(
		"Auton", "currentCmd", PubSubOption.sendAll(true));
	public static double currentState = 0;

	public static Command oneMeter() {
		return AutoBuilder.followPath(oneMeter);
	}

	public static Command simpleThing(
		Swerve swerve) {
		var resetPose = swerve.resetPose(simpleThing);

		return Commands.sequence(
			resetPose, AutoBuilder.followPath(
				simpleThing));
	}

	public static Command leave(Superstructure superstructure, Shooter shooter, Swerve swerve) {
		var resetPose = swerve.resetPose(path_leave).asProxy();
		var shoot = shoot(superstructure, shooter, true);

		var pathFollow = AutoBuilder.followPath(path_leave).asProxy();
		return Commands.sequence(
			Commands.parallel(
				resetPose,
				Commands.sequence(incrementState(), shoot)
			),
			incrementState(),
			pathFollow
		);
	}

	public static Command altTwoPc(Superstructure superstructure, Shooter shooter, Swerve swerve) {
		var resetPose = swerve.resetPose(altTwo).asProxy();
		var pathFollow = AutoBuilder.followPath(altTwo).asProxy();
		var preloadShot = shoot(superstructure, shooter, true);
		var intake = superstructure.autonIntakeCmd().asProxy();
		var swerveAim = swerve.aim();
		// var aimAndSpinUp = superstructure.aimAndSpinUp(() -> kPodiumAngle, false, true);
		// var secondShot = Commands.runOnce(() -> superstructure.forceStateToShooting()).asProxy();

		return Commands.sequence(
			Commands.parallel(
				resetPose, 
				preloadShot),
			Commands.parallel(
				intake.andThen(Commands.print("intake done")),
				pathFollow
			),
			Commands.print("path done, waiting for note ready"),
			Commands.parallel(
				Commands.waitUntil(superstructure.stateTrg_noteReady),
				swerveAim)
			// Commands.print("note ready & aimed"),
			// Commands.parallel(
			// 	aimAndSpinUp,
			// 	secondShot
			// )
		);
	}

	public static Command twoPc(Superstructure superstructure, Shooter shooter, Swerve swerve) {
		var leave = leave(superstructure, shooter, swerve);
		var intake = superstructure.autonIntakeCmd().asProxy();
		var pathFollow = AutoBuilder.followPath(twoPc).asProxy();
		var aimAndSpinUp = superstructure.aimAndSpinUp(() -> kSubwooferAngle, false, false);
		var shoot = Commands.runOnce(() -> superstructure.forceStateToShooting()).asProxy();

		return Commands.sequence(
			leave,
			Commands.parallel(
				intake,
				pathFollow,
				Commands.sequence(
					Commands.waitUntil(superstructure.stateTrg_noteReady),
					Commands.parallel(
						aimAndSpinUp,
						Commands.sequence(
							Commands.waitUntil(() -> pathFollow.isFinished()), 
							shoot
						)
					)
				)
			)
		);
	}

	public static Command threePiece(
		Swerve swerve, Intake intake, Shooter shooter, Aim aim, Conveyor conveyor) {
		var toAngle = Commands.repeatingSequence(aim.toTarget());
		var resetPoseCmd = swerve.resetPose(threePc);
		var aimCmd1 = aim.setAimTarget().asProxy();
		var shootCmd = shooter.spinUp().withTimeout(shooterTimeout).asProxy();
		var pathCmd = AutoBuilder.followPath(threePc);
		var aimCmd2 = aim.setAimTarget().asProxy();
		var aimCmd3 = aim.setAimTarget().asProxy();

		return Commands.parallel(
			toAngle,
			Commands.sequence(
				resetPoseCmd,
				aimCmd1,
				shootCmd,
				Commands.parallel(
					pathCmd,
					Commands.sequence(
						Commands.waitSeconds(
							0.45),
						aimCmd2,
						intakeShotCycle(
							intake, conveyor, shooter))),
				aimCmd3,
				intakeShotCycle(
					intake, conveyor, shooter
				)
			)
		);
	}

	public static Command fourPiece(Swerve swerve, Intake intake, Shooter shooter, Aim aim, Conveyor conveyor) {
		var toAngle = Commands.repeatingSequence(aim.toTarget());
		var resetPoseCmd = swerve.resetPose(fourPc);
		var aimCmd1 = aim.setAimTarget().asProxy();
		var pathCmd = AutoBuilder.followPath(fourPc);
		var shootCmd1 = shooter.spinUp().withTimeout(shooterTimeout).asProxy();
		var aimCmd2 = aim.setAimTarget().asProxy();
		var intakeCmd1 = intake.outtake().withTimeout(intakeTimeout).asProxy();
		var aimCmd3 = aim.setAimTarget().asProxy();
		var shootCmd2 = shooter.spinUp().withTimeout(shooterTimeout).asProxy();
		var intakeCmd2 = intake.outtake().withTimeout(intakeTimeout).asProxy();
		var aimCmd4 = aim.setAimTarget().asProxy();
		var shootCmd3 = shooter.spinUp().withTimeout(shooterTimeout).asProxy();

		return Commands.parallel(
			toAngle,
			Commands.sequence(
				resetPoseCmd,
				aimCmd1,
				shootCmd1,
				Commands.parallel(
					pathCmd,
					Commands.sequence(
						// TODO: check timing :(
						Commands.waitSeconds(
							0.58),
						aimCmd2,
						intakeShotCycle(
							intake, conveyor, shooter),
						Commands.waitSeconds(
							1.34),
						intakeCmd1,
						Commands.waitSeconds(
							1.62),
						aimCmd3,
						shootCmd2,
						Commands.waitSeconds(
							1.17),
						intakeCmd2,
						Commands.waitSeconds(
							1.63),
						aimCmd4,
						shootCmd3
					)
				)
			)
		);
	}

	public static Command fivePiece(
		Swerve swerve, Intake intake, Shooter shooter, Aim aim, Conveyor conveyor) {
		var toAngle = Commands.repeatingSequence(
			aim.toTarget());
		var resetPoseCmd = swerve.resetPose(fivePc);
		var aimCmd1 = aim.setAimTarget().asProxy();
		var aimCmd2 = aim.setAimTarget().asProxy();
		var pathCmd = AutoBuilder.followPath(fivePc);
		var aimCmd3 = aim.setAimTarget().asProxy();
		var aimCmd4 = aim.setAimTarget().asProxy();
		var shootCmd = shooter.subwoofer().withTimeout(shooterTimeout).asProxy();
		var finalIntake = intake.outtake().withTimeout(intakeTimeout).asProxy();
		var aimCmd5 = aim.setAimTarget().asProxy();
		var finalShot = shooter.spinUp().withTimeout(shooterTimeout).asProxy();

		return Commands.parallel(
			toAngle,
			Commands.sequence(
				resetPoseCmd,
				aimCmd1,
				shootCmd,
				Commands.parallel(
					aimCmd2,
					intakeShotCycle(intake, conveyor, shooter)
				),
				Commands.parallel(
					pathCmd,
					Commands.sequence(
						Commands.waitSeconds(0.52),
						Commands.parallel(
							aimCmd3,
							intakeShotCycle(
								intake, conveyor, shooter)
							)
						),
					Commands.sequence(
						Commands.waitSeconds(1.38),
						Commands.parallel(
							aimCmd4,
							intakeShotCycle(intake, conveyor, shooter)
						)
					),
					Commands.sequence(
						Commands.waitSeconds(2.86),
						finalIntake
					)
				),
				aimCmd5,
				finalShot
			)
		);
	}

	public static Command fivePcFollow(Swerve swerve) {
		var resetPoseCmd = swerve.resetPose(fivePc);
		var pathCmd = AutoBuilder.followPath(fivePc);

		return resetPoseCmd.andThen(pathCmd);
	}

	private static Command intakeShotCycle(Intake intake, Conveyor conveyor, Shooter shooter) {
		var conveyCmd = conveyor.runFast().withTimeout(conveyorTimeout).asProxy();
		var spinUpCmd = shooter.spinUp().withTimeout(spinUpTimeout).asProxy();
		var shootCmd = shooter.subwoofer().withTimeout(shooterTimeout).asProxy();
		var spinUpAndShoot = Commands.sequence(spinUpCmd, shootCmd).asProxy();
		var intakeCmd = intake.outtake().withTimeout(intakeTimeout).asProxy();
		return Commands.parallel(conveyCmd, intakeCmd, spinUpAndShoot);
	}

	public static Command shoot(Superstructure superstructure, Shooter shooter, boolean force) {
		var aimAndSpinUp = superstructure.aimAndSpinUp(() -> kSubwooferAngle, false, true);
		var waitUntilSpunUp = Commands.waitUntil(() -> shooter.spinUpFinished());
		var noteReady = superstructure.forceStateToNoteReady().asProxy();
		var shoot = Commands.runOnce(() -> superstructure.forceStateToShooting()).asProxy();

		return Commands.sequence(
			noteReady,
			Commands.parallel(
				Commands.print("aim and spin up"),
				aimAndSpinUp.andThen(Commands.print("aimAndSpinUp_DONE")),
				Commands.sequence(
					Commands.print("waiting for spin up"),
					waitUntilSpunUp.andThen(Commands.print("waitUntilSpunUp_DONE")),
					Commands.print("shooting"),
					shoot.andThen(Commands.print("shoot_DONE")),
					Commands.print("shot")
				)
			)
		);
	}

	private static Command incrementState() {
		return Commands.runOnce(
			() -> {
				currentState++;
				log_state.accept(
					currentState);
			});
	}
}
