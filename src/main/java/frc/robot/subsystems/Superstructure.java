package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.IntakeK.kVisiSightId;
import static frc.robot.Constants.RobotK.kDbTabName;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    private final DoubleConsumer m_driverRumbler, m_manipRumbler;

    private final DigitalInput frontVisiSight = new DigitalInput(kVisiSightId);
    private final DigitalInput shooterBeamBreak = new DigitalInput(0);
    private final BooleanSupplier bs_frontVisiSight = () -> frontVisiSight.get();

    private final BooleanSupplier bs_shooterBeamBreak = () -> !shooterBeamBreak.get();
    private final BooleanLogger log_frontVisiSight = WaltLogger.logBoolean("Sensors", "frontVisiSight",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_frontVisiSightIrq = WaltLogger.logBoolean("Sensors", "frontVisiSightIrq",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_shooterBeamBreak = WaltLogger.logBoolean("Sensors", "shooterBeamBreak",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_autonIntakeReq = WaltLogger.logBoolean(kDbTabName, "autonIntakeReq",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_autonShootReq = WaltLogger.logBoolean(kDbTabName, "autonShootReq",
        PubSubOption.sendAll(true));

    /** To be set on any edge from the AsyncIrq callback  */
    private boolean frontVisiSightSeenNote = false;

    private final AsynchronousInterrupt ai_frontVisiSight = new AsynchronousInterrupt(frontVisiSight,
        (Boolean rising, Boolean falling) -> {
            if ((rising || falling) && !frontVisiSightSeenNote) {
                frontVisiSightSeenNote = true;
                log_frontVisiSightIrq.accept(true);
            }
        });

    private NoteState m_state;

    /* note trackers. the note is named timothy. */
    private boolean timothyEntered = false;
    private boolean timothyIn = false;
    private boolean timothyFieldTrip = false;
    private boolean autonIntake = false;
    private boolean autonShoot = false;
    private boolean driverRumbled = false;
    private boolean manipulatorRumbled = false;

    public final EventLoop sensorEventLoop = new EventLoop();

    /** true = driver wants to intake */
    private final Trigger trg_driverIntakeReq;
    /** true = driver wants to shoot */
    private final Trigger trg_driverShootReq;

    private final Trigger trg_autonIntakeReq = new Trigger(() -> autonIntake);
    private final Trigger trg_autonShootReq = new Trigger(() -> autonShoot);

    /** true = has note */
    private final Trigger trg_shooterSensor;
    /** true = has note */
    private final Trigger trg_frontSensorIrq;

    private final Trigger trg_spunUp;
    private final Trigger trg_atAngle;

    private final Trigger trg_intakeReq;
    private final Trigger trg_shootReq;

    private final Trigger trg_timothyFieldTrip = new Trigger(sensorEventLoop,
        () -> timothyFieldTrip);

    private final Trigger stateTrg_idle = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.IDLE);
    private final Trigger stateTrg_noteRetracting = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.ROLLER_BEAM_RETRACT);
    private final Trigger stateTrg_spinUp = new Trigger(sensorEventLoop, () -> m_state == NoteState.SHOT_SPINUP);
    private final Trigger stateTrg_shooting = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.SHOOTING);
    private final Trigger extStateTrg_shooting = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.SHOOTING ||
            m_state == NoteState.LEAVING_BEAM_BREAK ||
            m_state == NoteState.LEFT_BEAM_BREAK);
    private final Trigger stateTrg_leavingBeamBreak = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.LEAVING_BEAM_BREAK);
    private final Trigger stateTrg_leftBeamBreak = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.LEFT_BEAM_BREAK);
    public final Trigger stateTrg_noteReady = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.NOTE_READY);

    private final DoubleLogger log_state = WaltLogger.logDouble(kDbTabName, "state",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_timothyEntered = WaltLogger.logBoolean(kDbTabName, "timothyEntered",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_timothyIn = WaltLogger.logBoolean(kDbTabName, "timothyIn",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_timothyFieldTrip = WaltLogger.logBoolean(kDbTabName, "timothyFieldTrip",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_driverIntakeReq = WaltLogger.logBoolean(kDbTabName, "intakeButton");
    private final BooleanLogger log_aimReady = WaltLogger.logBoolean(kDbTabName, "aimReady");

    public Superstructure(
        Aim aim, Intake intake, Conveyor conveyor, Shooter shooter,
        Trigger intaking, Trigger shooting,
        DoubleConsumer driverRumbler, DoubleConsumer manipRumbler) {
        m_aim = aim;
        m_intake = intake;
        m_conveyor = conveyor;
        m_shooter = shooter;

        m_driverRumbler = driverRumbler;
        m_manipRumbler = manipRumbler;

        trg_driverIntakeReq = intaking;
        trg_driverShootReq = shooting;

        trg_intakeReq = trg_driverIntakeReq.or(trg_autonIntakeReq);
        trg_shootReq = trg_driverShootReq.or(trg_autonShootReq);

        ai_frontVisiSight.setInterruptEdges(true, true);
        ai_frontVisiSight.enable();

        trg_frontSensorIrq = new Trigger(sensorEventLoop, () -> frontVisiSightSeenNote);
        trg_shooterSensor = new Trigger(sensorEventLoop, bs_shooterBeamBreak);

        trg_spunUp = new Trigger(m_shooter::spinUpFinished).debounce(0.04);
        trg_atAngle = new Trigger(m_aim::aimFinished);

        m_state = NoteState.IDLE;

        configureStateTriggers();
    }

    public Command ampShot() {
        var shootCmd = m_shooter.ampShot();
        var conveyorCmd = m_conveyor.runFast();

        return Commands.parallel(shootCmd, conveyorCmd);
    }

    private Command cmdDriverRumble(double intensity, double seconds) {
        if (driverRumbled) {
            return Commands.none();
        }
        return Commands.startEnd(
            () -> {
                m_driverRumbler.accept(intensity);
                driverRumbled = true;
            },
            () -> m_driverRumbler.accept(0)).withTimeout(seconds);
    }

    private Command cmdManipRumble(double intensity, double seconds) {
        if (manipulatorRumbled) {
            return Commands.none();
        }
        return Commands.startEnd(
            () -> {
                m_manipRumbler.accept(intensity);
                manipulatorRumbled = true;
            },
            () -> m_manipRumbler.accept(0)).withTimeout(seconds);
    }

    private Command changeStateCmd(NoteState state) {
        return Commands.runOnce(() -> {
            if (m_state == state) {
                System.out.println("[SUPERSTRUCTURE] IGNORING State Change FROM: " + state + " TO " + m_state.toString());
                return;
            }

            var oldState = m_state;
            m_state = state;
            System.out.println("[SUPERSTRUCTURE] State Change FROM: " + oldState + " TO " + m_state.toString());
        });
    }

    private void configureStateTriggers() {
        // intakeReq && idle
        (trg_intakeReq.and(stateTrg_idle))
            .onTrue(Commands.runOnce(() -> m_state = NoteState.INTAKE)
                .alongWith(
                    // wait until aim is ±50 degrees to intake mode
                    Commands.sequence(
                        m_aim.intakeAngleNearCmd(),
                        Commands.parallel(m_intake.run(), m_conveyor.startSlow()))));

        // !(intakeReq || idle) => !intakeReq && !idle
        (trg_intakeReq.or(trg_frontSensorIrq)).onFalse(changeStateCmd(NoteState.IDLE));

        trg_frontSensorIrq.onTrue(cmdManipRumble(1, 0.5));

        // note in shooter and not shooting
        (trg_shooterSensor.and((extStateTrg_shooting.or(stateTrg_spinUp)).negate()))
            .onTrue(changeStateCmd(NoteState.ROLLER_BEAM_RETRACT));
        stateTrg_noteRetracting.onTrue(
            Commands.parallel(
                m_intake.stop(),
                m_conveyor.retract()));

        // !beamBreak && noteRetracting
        // Post-retract stop
        (trg_shooterSensor.negate().and(stateTrg_noteRetracting))
            .onTrue(
                Commands.sequence(
                    Commands.waitSeconds(0.35),
                    changeStateCmd(NoteState.NOTE_READY),
                    m_conveyor.stop()));

        // manipulator controls this now so idt this is necessary
        // shot requested and note is ready
        // state -> spinup, cmd spinup shooter
        // (trg_shootReq.and(stateTrg_aimed))
        // .onTrue(changeStateCmd(NoteState.SHOT_SPINUP));

        // manipulator controls this now so idt this is necessary (2)
        // ((trg_shootReq.negate()).and(stateTrg_shotSpinup)).onTrue(
        // Commands.sequence(
        // changeStateCmd(NoteState.AIMED), stopEverything()));

        (trg_spunUp.and(trg_atAngle))
            .onTrue(cmdDriverRumble(1, 0.5));

        // if shooter spun up and state spinupping
        // state -> SHOOTING
        (trg_spunUp.and(trg_atAngle).and(trg_shootReq).and(stateTrg_spinUp))
            .onTrue(changeStateCmd(NoteState.SHOOTING));

        // if now shooting, note leaving, or note just left
        // cmd conveyorFast
        // and then if none
        // cmd conveyorStop
        // (stateTrg_shooting.or(stateTrg_leavingBeamBreak).or(stateTrg_leftBeamBreak))
        extStateTrg_shooting
            .onTrue(m_conveyor.runFast())
            .onFalse(m_conveyor.stop());

        // if note in shooter sensor and state shooting
        // state -> LEAVING_BEAM_BREAK, timothy says bye 😃
        (trg_shooterSensor.and(stateTrg_shooting))
            .onTrue(Commands.parallel(
                changeStateCmd(NoteState.LEAVING_BEAM_BREAK),
                Commands.runOnce(() -> timothyFieldTrip = true)));

        // if note not in shooter and state leaving and timothy waving bye 😄
        // state -> LEFT_BEAM_BREAK
        ((trg_shooterSensor.debounce(0.1)).negate().and(stateTrg_leavingBeamBreak).and(trg_timothyFieldTrip))
            .onTrue(changeStateCmd(NoteState.LEFT_BEAM_BREAK));

        // if left beam break
        // wait for 0.5 sec
        // state -> IDLE
        stateTrg_leftBeamBreak
            .onTrue(Commands.sequence(
                Commands.waitSeconds(0.5),
                changeStateCmd(NoteState.IDLE)));

        stateTrg_idle
            .onTrue(Commands.parallel(
                Commands.runOnce(
                    () -> {
                        timothyEntered = false;
                        timothyIn = false;
                        timothyFieldTrip = false;
                        frontVisiSightSeenNote = false;
                        autonIntake = false;
                        autonShoot = false;
                        driverRumbled = false;
                        manipulatorRumbled = false;
                    }),
                stopEverything(), m_aim.intakeAngleNearCmd()));
    }

    public Command stopEverything() {
        var shootCmd = m_shooter.stop();
        var conveyorCmd = m_conveyor.stop();
        var intakeCmd = m_intake.stop();

        return Commands.parallel(shootCmd, conveyorCmd, intakeCmd);
    }

    public Command aimAndSpinUp(Supplier<Measure<Angle>> target, boolean amp) {
        return aimAndSpinUp(target, amp, false);
    }

    public Command aimAndSpinUp(Supplier<Measure<Angle>> target, boolean amp, boolean auton) {
        var aimCmd = m_aim.toAngleUntilAt(target, amp ? Degrees.of(1) : Degrees.of(2)); // TODO make this unmagical 🙁

        var waitForNoteReady = Commands.waitUntil(() -> m_state.idx > NoteState.ROLLER_BEAM_RETRACT.idx)
            .andThen(Commands.print("====NOTE READY===="));

        Command shootCmd;

        if (auton) {
            shootCmd = amp ? m_shooter.ampShot() : m_shooter.subwoofer(stateTrg_idle); 
        } else {
            shootCmd = amp ? m_shooter.ampShot() : m_shooter.subwoofer();
        }

        return Commands.sequence(
            Commands.parallel(
                aimCmd.asProxy().andThen(Commands.print("AimAndSpinUp_AIM_DONE")),
                waitForNoteReady.andThen(Commands.print("AimAndSpinUp_NOTERD_DONE"))
            ),
            Commands.parallel(
                shootCmd.asProxy().andThen(Commands.print("shooty shoot done (no yelling)")),
                changeStateCmd(NoteState.SHOT_SPINUP)
            )
        );
    }


    public Command backwardsRun() {
        var shooterCmd = m_shooter.runBackwards();
        var conveyorCmd = m_conveyor.runBackwards();

        return Commands.parallel(shooterCmd,
            Commands.sequence(Commands.waitUntil(trg_spunUp), conveyorCmd));
    }

    public void fastPeriodic() {
        log_state.accept((double) m_state.idx);
        log_timothyEntered.accept(timothyEntered);
        log_timothyIn.accept(timothyIn);
        log_timothyFieldTrip.accept(timothyFieldTrip);
        log_frontVisiSight.accept(bs_frontVisiSight.getAsBoolean());
        log_shooterBeamBreak.accept(bs_shooterBeamBreak.getAsBoolean());
        log_frontVisiSightIrq.accept(frontVisiSightSeenNote);
        log_driverIntakeReq.accept(trg_driverIntakeReq.getAsBoolean());
        log_autonIntakeReq.accept(autonIntake);
        log_autonShootReq.accept(autonShoot);
        log_aimReady.accept(trg_atAngle.getAsBoolean());
    }

    public void forceStateToNoteReady() {
        timothyEntered = true;
        timothyIn = true;
        autonShoot = false;
        autonIntake = false;
        m_state = NoteState.NOTE_READY;
    }

    public void forceStateToShooting() {
        timothyEntered = true;
        timothyIn = true;
        autonIntake = false;
        autonShoot = true;
        m_state = NoteState.SHOOTING;
    }

    public Command autonIntakeCmd() {
        return runOnce(() -> {
            autonIntake = true;
            autonShoot = false;
        });
    }

    public Command waitUntilIdle() {
        return Commands.waitUntil(() -> isIdle());
    }

    public boolean isIdle() {
        return m_state == NoteState.IDLE;
    }

    public enum NoteState {
        IDLE(0),
        INTAKE(1),
        ROLLER_BEAM_RETRACT(2), // Note hit top beam
        NOTE_READY(3),
        SHOT_SPINUP(4),
        SHOOTING(5),
        LEAVING_BEAM_BREAK(6),
        LEFT_BEAM_BREAK(7);

        public final int idx;

        private NoteState(int idx) {
            this.idx = idx;
        }
    }
}
