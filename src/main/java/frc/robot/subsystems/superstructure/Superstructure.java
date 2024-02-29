package frc.robot.subsystems.superstructure;

import static frc.robot.Constants.RobotK.kDbTabName;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.Conveyor;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.BooleanLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

public class Superstructure extends SubsystemBase {
    public Aim m_aim;
    public Intake m_intake;
    public Conveyor m_conveyor;
    public Shooter m_shooter;

    private State m_state;

    /* note trackers. the note is named timothy. */
    private boolean timothyEntered = false;
    private boolean readyToCheck = false;
    private boolean timothyIn = false;
    private boolean timothyFieldTrip = false;

    private final CommandXboxController simController = new CommandXboxController(3);

    private final Trigger trg_intaking = simController.pov(0);
    // private final Trigger trg_frontSensor = new Trigger(m_intake.m_sightTrigger);
    private final Trigger trg_frontSensor = simController.x();
    // private final Trigger trg_a1 = new Trigger(() -> m_state == State.A1);
    private final Trigger trg_timothyEntered = new Trigger(() -> timothyEntered);
    private final Trigger trg_readyToCheck = new Trigger(() -> readyToCheck);
    // private final Trigger trg_a2 = new Trigger(() -> m_curState == State.A2);
    // private final Trigger trg_b1 = new Trigger(() -> m_state == State.B2);
    private final Trigger trg_timothyIn = new Trigger(() -> timothyIn);
    // private final Trigger trg_beamBreak = new Trigger(m_conveyor.m_note);
    private final Trigger trg_beamBreak = simController.y();
    private final Trigger trg_startShoot = simController.pov(45);
    // private final Trigger trg_shoot = new Trigger(() -> m_state == State.SHOOT);
    // private final Trigger trg_spunUp = new Trigger(m_shooter::spinUpFinished);
    private final Trigger trg_spunUp = simController.a();
    // private final Trigger trg_aimed = new Trigger(m_aim::aimFinished);
    private final Trigger trg_aimed = simController.b();
    private final Trigger trg_shooting = new Trigger(() -> m_state == State.SHOOTING);
    private final Trigger trg_d1 = new Trigger(() -> m_state == State.D1);
    private final Trigger trg_d2 = new Trigger(() -> m_state == State.D2);
    private final Trigger trg_timothyFieldTrip = new Trigger(() -> timothyFieldTrip);

    private final DoubleLogger log_state = WaltLogger.logDouble(kDbTabName, "state");
    private final BooleanLogger log_timothyEntered = WaltLogger.logBoolean(kDbTabName, "timothyEntered");
    private final BooleanLogger log_timothyIn = WaltLogger.logBoolean(kDbTabName, "timothyIn");
    private final BooleanLogger log_timothyFieldTrip = WaltLogger.logBoolean(kDbTabName, "timothyLeaving");

    public Superstructure(Aim aim, Intake intake, Conveyor conveyor, Shooter shooter, Trigger shoot) {
        m_aim = aim;
        m_intake = intake;
        m_conveyor = conveyor;
        m_shooter = shooter;
        // trg_intaking = intake;
        // trg_startShoot = shoot;

        m_state = State.READY;

        configureStateTriggers();
    }

    private void configureStateTriggers() {
        trg_intaking.onTrue(Commands.runOnce(() -> m_state = State.INTAKE));
        trg_frontSensor.onTrue(Commands.runOnce(
            () -> {
                m_state = State.A1;
                timothyEntered = true;
            }));
        // trg_a1.whileTrue(m_intake.run()); // and set aim
        trg_frontSensor.negate().and(trg_timothyEntered).onTrue(Commands.runOnce(() -> readyToCheck = true));
        trg_frontSensor.and(trg_readyToCheck).onTrue(Commands.runOnce(() -> timothyIn = true));
        trg_frontSensor.negate().and(trg_timothyIn)
            .onTrue(Commands.sequence(Commands.runOnce(() -> m_state = State.CONVEY), Commands.waitSeconds(0.25),
                Commands.runOnce(() -> m_state = State.B2)));
        // trg_convey.whileTrue(intake());
        trg_beamBreak.onTrue(Commands.runOnce(
            () -> {
                if (m_state != State.SHOOTING) {
                    m_state = State.C1;
                }
            }));
        trg_beamBreak.onFalse(Commands.runOnce(
            () -> {
                if (m_state != State.D1) {
                    m_state = State.SHOOT_OK;
                }
            }));
        trg_startShoot.onTrue(Commands.runOnce(() -> m_state = State.SHOOT));
        trg_spunUp.and(trg_aimed).onTrue(Commands.runOnce(() -> m_state = State.SHOOTING));
        // trg_shooting.whileTrue(shoot());
        trg_beamBreak.and(trg_shooting).onTrue(Commands.runOnce(() -> {
            m_state = State.D1;
            timothyFieldTrip = true;
        }));
        trg_beamBreak.and(trg_d1).and(trg_timothyFieldTrip).onTrue(Commands.runOnce(() -> m_state = State.D2));
        trg_d2.onTrue(Commands.sequence(Commands.waitSeconds(0.25), Commands.runOnce(() -> m_state = State.READY)));
        // trg_startShoot.onFalse(Commands.runOnce(() -> m_state = State.READY));
    }

    private Command spinUpWait() {
        return Commands.waitSeconds(0.2).andThen(Commands.waitUntil(() -> m_shooter.spinUpFinished()));
    }

    public Command intake() {
        var aimCmd = m_aim.intakeMode();
        var intakeCmd = m_intake.run();
        var conveyorCmd = m_conveyor.run(false);
        var retractCmd = m_conveyor.retract();

        return Commands.sequence(aimCmd, Commands.race(intakeCmd, conveyorCmd), retractCmd);
    }

    public Command intakeShotCycle() {
        var aimCmd = m_aim.toTarget();
        var intakeCmd = m_intake.outtake();
        var conveyorCmd = m_conveyor.run(false);
        var shooterCmd = m_shooter.shoot();

        return Commands.parallel(
            aimCmd,
            shooterCmd,
            Commands.race(
                intakeCmd,
                Commands.sequence(
                    // TODO make this work
                    Commands.waitUntil(() -> spinUpWait().isFinished() && aimCmd.isFinished()),
                    conveyorCmd)));
    }

    public Command subwoofer() {
        var aimCmd = m_aim.subwoofer();
        var shootCmd = m_shooter.shoot();
        var conveyorCmd = m_conveyor.run(true);

        return Commands.parallel(
            shootCmd,
            aimCmd,
            Commands.sequence(
                // TODO make this work correctly
                Commands.parallel(
                    spinUpWait(),
                    Commands.waitUntil(() -> aimCmd.isFinished())),
                conveyorCmd));
    }

    public Command shoot() { // podium angle: 0.007568 rot
        var shootCmd = m_shooter.shoot();
        var conveyorCmd = m_conveyor.run(true);

        return Commands.parallel(
            Commands.print("shooting"),
            shootCmd,
            Commands.sequence(
                Commands.print("waiting for spinup"),
                spinUpWait(),
                Commands.print("conveying"),
                conveyorCmd,
                Commands.print("done")));
    }

    public Command shootFast() {
        var shootCmd = m_shooter.shootFast();
        var conveyorCmd = m_conveyor.run(true);

        return Commands.parallel(
            shootCmd,
            Commands.sequence(
                // TODO make this work correctly
                spinUpWait(),
                conveyorCmd));
    }

    public Command trapShot() {
        var shootCmd = m_shooter.trapShot();
        var conveyorCmd = m_conveyor.run(true);

        return Commands.parallel(
            shootCmd,
            Commands.sequence(
                // TODO make this work correctly
                spinUpWait(),
                conveyorCmd));
    }

    public Command ampShot() {
        var shootCmd = m_shooter.ampShot();
        var conveyorCmd = m_conveyor.run(true);

        return Commands.parallel(
            shootCmd,
            Commands.sequence(
                // TODO make this work correctly
                spinUpWait(),
                conveyorCmd));
    }

    public Command aimAndShoot() {
        var aimCmd = m_aim.toTarget();
        var conveyorCmd = m_conveyor.run(false);
        var shooterCmd = m_shooter.shoot();

        return Commands.parallel(
            aimCmd,
            shooterCmd,
            Commands.sequence(
                Commands.waitUntil(() -> m_shooter.spinUpFinished() && aimCmd.isFinished()),
                conveyorCmd));
    }

    public Command backwardsRun() {
        var shooterCmd = m_shooter.runBackwards();
        var conveyorCmd = m_conveyor.runBackwards();

        return Commands.parallel(shooterCmd, conveyorCmd);
    }

    public void periodic() {
        log_state.accept((double) m_state.getId());
        log_timothyEntered.accept(timothyEntered);
        log_timothyIn.accept(timothyIn);
        log_timothyFieldTrip.accept(timothyFieldTrip);
    }

    public void backToReady() {
        m_state = State.READY;
    }
}
