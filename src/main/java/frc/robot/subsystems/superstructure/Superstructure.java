package frc.robot.subsystems.superstructure;

import static frc.robot.Constants.RobotK.kDbTabName;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.Conveyor;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.BooleanLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

public class Superstructure extends SubsystemBase {
    public final Aim m_aim;
    public final Intake m_intake;
    public final Conveyor m_conveyor;
    public final Shooter m_shooter;

    private State m_state;

    /* note trackers. the note is named timothy. */
    private boolean timothyEntered = false;
    private boolean readyToCheck = false;
    private boolean timothyIn = false;
    private boolean timothyFieldTrip = false;

    public final EventLoop sensorEvtLoop = new EventLoop();

    /** true = driver wants to intake */ 
    private final Trigger trg_driverIntakeReq;
    /** true = driver wants to shoot */
    private final Trigger trg_driverShootReq;

    /** true = has note */ 
    private final Trigger trg_intakeSensor;
    /** true = has note */ 
    private final Trigger trg_shooterSensor;

    private final Trigger trg_spunUp;
    private final Trigger trg_aimed;

    private final Trigger trg_a1 = new Trigger(sensorEvtLoop, () -> m_state == State.INTAKE_TOP_RISING);
    private final Trigger trg_timothyEntered = new Trigger(sensorEvtLoop, () -> timothyEntered);
    private final Trigger trg_readyToCheck = new Trigger(sensorEvtLoop, () -> readyToCheck);
    private final Trigger trg_b2 = new Trigger(sensorEvtLoop, () -> m_state == State.INTAKE_BOT_RISING);
    private final Trigger trg_timothyIn = new Trigger(sensorEvtLoop, () -> timothyIn);
    private final Trigger trg_noteRetracting = new Trigger(sensorEvtLoop, () -> m_state == State.ROLLER_BEAM_RETRACT);
    private final Trigger trg_shooting = new Trigger(sensorEvtLoop, () -> m_state == State.SHOOTING);
    private final Trigger trg_d1 = new Trigger(sensorEvtLoop, () -> m_state == State.D1);
    private final Trigger trg_d2 = new Trigger(sensorEvtLoop, () -> m_state == State.D2);
    private final Trigger trg_timothyFieldTrip = new Trigger(sensorEvtLoop, () -> timothyFieldTrip);
    private final Trigger trg_ready = new Trigger(sensorEvtLoop, () -> m_state == State.READY);

    private final DoubleLogger log_state = WaltLogger.logDouble(kDbTabName, "state", PubSubOption.periodic(0.005));
    private final BooleanLogger log_timothyEntered = WaltLogger.logBoolean(kDbTabName, "timothyEntered", PubSubOption.periodic(0.005));
    private final BooleanLogger log_timothyIn = WaltLogger.logBoolean(kDbTabName, "timothyIn", PubSubOption.periodic(0.005));
    private final BooleanLogger log_timothyFieldTrip = WaltLogger.logBoolean(kDbTabName, "timothyFieldTrip",PubSubOption.periodic(0.005));
    private final BooleanLogger log_readyToCheck = WaltLogger.logBoolean(kDbTabName, "readyToCheck", PubSubOption.periodic(0.005));

    public Superstructure(Aim aim, Intake intake, Conveyor conveyor, Shooter shooter,
        Trigger intaking, Trigger shoot, BooleanSupplier intakeSensor, BooleanSupplier beamBreak) {
        m_aim = aim;
        m_intake = intake;
        m_conveyor = conveyor;
        m_shooter = shooter;

        trg_driverIntakeReq = intaking;
        trg_driverShootReq = shoot;

        trg_intakeSensor = new Trigger(sensorEvtLoop, intakeSensor);
        trg_shooterSensor = new Trigger(sensorEvtLoop, beamBreak);

        trg_spunUp = new Trigger(m_shooter::spinUpFinished);
        trg_aimed = new Trigger(m_aim::aimFinished);

        m_state = State.READY;

        configureStateTriggers();
        // stateTriggersWhileDisabled();
    }

    private void configureStateTriggers() {
        trg_driverIntakeReq.onTrue(Commands.runOnce(() -> m_state = State.INTAKE)
            .alongWith(intake()));
        trg_intakeSensor.and(trg_a1.negate()).onTrue(Commands.runOnce(
            () -> {
                if (!readyToCheck) {
                    m_state = State.INTAKE_TOP_RISING;
                    timothyEntered = true;
                }
            }));
        trg_intakeSensor.negate().and(trg_timothyEntered).onTrue(Commands.runOnce(() -> readyToCheck = true));
        trg_intakeSensor.and(trg_readyToCheck).onTrue(Commands.runOnce(() -> timothyIn = true));
        trg_intakeSensor.negate().and(trg_timothyIn)
            .onTrue(Commands.sequence(Commands.runOnce(() -> m_state = State.INTAKE_TOP_FALLING),
                Commands.waitSeconds(0.25),
                Commands.runOnce(() -> m_state = State.INTAKE_BOT_RISING)));
        trg_b2.and(trg_shooterSensor).onTrue(Commands.runOnce(() -> {
            m_state = State.ROLLER_BEAM_RETRACT;
        }));
        trg_noteRetracting.onTrue(
            Commands.sequence(
                Commands.print("ENTER C1"),
                Commands.parallel(
                    m_intake.stop(),
                    m_conveyor.retract()
                )
            )
        );
        // if (noNote && ) stopConveyor; state = shoot_ok
        // 
        trg_shooterSensor.negate().and(trg_noteRetracting).onTrue(Commands.runOnce(
            () -> {
                if (m_state != State.D1) {
                    m_state = State.SHOOT_OK;
                }
            }));
        trg_driverShootReq.onTrue(Commands.runOnce(() -> m_state = State.SHOT_SPINUP));
        trg_spunUp.and(trg_aimed).onTrue(Commands.runOnce(() -> m_state = State.SHOOTING));
        trg_shooting.or(trg_d1).or(trg_d2).whileTrue(shoot());
        trg_shooterSensor.and(trg_shooting).onTrue(Commands.runOnce(() -> {
            m_state = State.D1;
            timothyFieldTrip = true;
        }));
        trg_d1.onTrue(m_conveyor.stop());
        trg_shooterSensor.and(trg_d1).and(trg_timothyFieldTrip).onTrue(Commands.runOnce(() -> m_state = State.D2));
        trg_d2.onTrue(Commands.sequence(Commands.waitSeconds(0.25),
            Commands.runOnce(() -> m_state = State.READY)));
        // trg_startShoot.onFalse(Commands.runOnce(() -> m_state = State.READY));
        trg_ready.onTrue(Commands.runOnce(
            () -> {
                timothyEntered = false;
                readyToCheck = false;
                timothyIn = false;
                timothyFieldTrip = false;
            }));
    }

    // private void stateTriggersWhileDisabled() {
    // trg_intaking.onTrue(Commands.runOnce(() -> m_state = State.INTAKE));
    // trg_frontSensor.and(trg_a1.negate()).onTrue(Commands.runOnce(
    // () -> {
    // if (!readyToCheck) {
    // m_state = State.A1;
    // timothyEntered = true;
    // System.out.println("a1");
    // }
    // }).ignoringDisable(true));
    // trg_frontSensor.negate().and(trg_timothyEntered)
    // .onTrue(Commands.runOnce(() -> readyToCheck = true).ignoringDisable(true));
    // trg_frontSensor.and(trg_readyToCheck).onTrue(Commands.runOnce(() -> timothyIn
    // = true).ignoringDisable(true));
    // trg_frontSensor.negate().and(trg_timothyIn)
    // .onTrue(
    // Commands.sequence(
    // Commands.runOnce(
    // () -> {
    // m_state = State.CONVEY;
    // System.out.println("conveying");
    // }),
    // Commands.waitSeconds(0.25),
    // Commands.runOnce(
    // () -> {
    // m_state = State.B2;
    // System.out.println("b2");
    // }))
    // .ignoringDisable(true));
    // // trg_b2.onTrue(m_conveyor.start());
    // trg_b2.and(trg_beamBreak).onTrue(Commands.runOnce(
    // () -> {
    // m_state = State.C1;
    // System.out.println("c1");

    // }).ignoringDisable(true));
    // // trg_c1.onTrue(m_conveyor.stop().andThen(m_conveyor.retract()));
    // trg_beamBreak.negate().and(trg_c1).onTrue(Commands.runOnce(
    // () -> {
    // m_state = State.SHOOT_OK;
    // System.out.println("d1");
    // }).ignoringDisable(true));
    // trg_shoot.onTrue(Commands.runOnce(
    // () -> {
    // m_state = State.SHOOT;
    // System.out.println("shoot");
    // }).ignoringDisable(true));
    // // trg_shooting.or(trg_d1).or(trg_d2).whileTrue(shoot());
    // trg_spunUp.and(trg_aimed).onTrue(Commands.runOnce(
    // () -> {
    // m_state = State.SHOOTING;
    // System.out.println("shooting");
    // }).ignoringDisable(true));
    // trg_beamBreak.and(trg_shooting).onTrue(Commands.runOnce(() -> {
    // m_state = State.D1;
    // timothyFieldTrip = true;
    // System.out.println("d1 (timothy field trip wheeee!)");
    // }).ignoringDisable(true));
    // trg_beamBreak.and(trg_d1).and(trg_timothyFieldTrip).onTrue(
    // Commands.runOnce(
    // () -> {
    // m_state = State.D2;
    // System.out.println("d2");
    // }).ignoringDisable(true));
    // trg_d2.onTrue(Commands.sequence(Commands.waitSeconds(0.25), Commands.runOnce(
    // () -> {
    // m_state = State.READY;
    // System.out.println("ready");
    // })).ignoringDisable(true));
    // // trg_startShoot.onFalse(Commands.runOnce(() -> m_state = State.READY));
    // trg_ready.onTrue(Commands.runOnce(
    // () -> {
    // timothyEntered = false;
    // readyToCheck = false;
    // timothyIn = false;
    // timothyFieldTrip = false;
    // }).ignoringDisable(true));
    // }

    private Command spinUpWait() {
        return Commands.waitSeconds(0.2).andThen(Commands.waitUntil(() -> m_shooter.spinUpFinished()));
    }

    public Command intake() {
        var aimCmd = m_aim.intakeMode();
        var intakeCmd = m_intake.run();
        var conveyorCmd = m_conveyor.run(true);
        // var retractCmd = m_conveyor.retract();

        return Commands.sequence(aimCmd, Commands.race(intakeCmd, conveyorCmd));
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
        log_state.accept((double) m_state.idx);
        log_timothyEntered.accept(timothyEntered);
        log_timothyIn.accept(timothyIn);
        log_timothyFieldTrip.accept(timothyFieldTrip);
        log_readyToCheck.accept(readyToCheck);
    }

    public void backToReady() {
        m_state = State.READY;
    }
}
