package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.IntakeK.kVisiSightId;
import static frc.robot.Constants.RobotK.kDbTabName;

import java.util.function.BooleanSupplier;
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
    private boolean driverShootReq = false;
    private boolean autonIntake = false;
    private boolean autonShoot = false;

    public final EventLoop sensorEventLoop = new EventLoop();

    /** true = driver wants to intake */
    private final Trigger trg_driverIntakeReq;
    /** true = driver wants to shoot */
    private final Trigger trg_driverShootReq = new Trigger(() -> driverShootReq);

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
    private final Trigger stateTrg_shotSpinup = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.SHOT_SPINUP);

    private final DoubleLogger log_state = WaltLogger.logDouble(kDbTabName, "state",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_timothyEntered = WaltLogger.logBoolean(kDbTabName, "timothyEntered",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_timothyIn = WaltLogger.logBoolean(kDbTabName, "timothyIn",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_timothyFieldTrip = WaltLogger.logBoolean(kDbTabName, "timothyFieldTrip",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_drvShootReq = WaltLogger.logBoolean(kDbTabName, "intakeButton");
    private final BooleanLogger log_aimReady = WaltLogger.logBoolean(kDbTabName, "aimReady");

    public Superstructure(Aim aim, Intake intake, Conveyor conveyor, Shooter shooter, Trigger intaking) {
        m_aim = aim;
        m_intake = intake;
        m_conveyor = conveyor;
        m_shooter = shooter;

        trg_driverIntakeReq = intaking;

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

    private void configureStateTriggers() {
        // intakeReq && idle
        (trg_intakeReq.and(stateTrg_idle))
            .onTrue(Commands.runOnce(() -> m_state = NoteState.INTAKE)
                .alongWith(
                    // Wait for aim +/-5deg to intake zone
                    Commands.sequence(
                        m_aim.intakeAngleNearCmd(),
                        Commands.parallel(m_intake.run(), m_conveyor.startSlow()))));
        // !(intakeReq || idle) -> !intakeReq && !idle
        (trg_intakeReq.or(trg_frontSensorIrq)).onFalse(Commands.runOnce(() -> {
            m_state = NoteState.IDLE;
        }));
        (trg_shooterSensor.and((stateTrg_shooting.or(stateTrg_leavingBeamBreak).or(stateTrg_leftBeamBreak)).negate()))
            .onTrue(Commands.runOnce(() -> {
                m_state = NoteState.ROLLER_BEAM_RETRACT;
            }));
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
                    Commands.runOnce(
                        () -> {
                            m_state = NoteState.NOTE_READY;
                        }),
                    m_conveyor.stop()));
        // shot requested and note is ready
        // state -> spinup, cmd spinup shooter
        (trg_shootReq.and(stateTrg_noteReady))
            .onTrue(Commands.sequence(
                Commands.parallel(
                    Commands.runOnce(() -> {
                        m_state = NoteState.SHOT_SPINUP;
                    }),
                    m_shooter.shoot())));

        // if shot not requested and shooter spinupping
        // state -> NOTE_READY, cmd stop everything
        ((trg_shootReq.negate()).and(stateTrg_shotSpinup)).onTrue(
            Commands.sequence(
                Commands.runOnce(() -> {
                    m_state = NoteState.NOTE_READY;
                }), stopEverything()));

        // if shooter spun up and state spinupping
        // state -> SHOOTING
        (trg_spunUp.and(trg_atAngle).and(stateTrg_shotSpinup))
            .onTrue(
                Commands.runOnce(() -> {
                    m_state = NoteState.SHOOTING;
                }));

        // if now shooting, note leaving, or note just left
        // cmd conveyorFast
        // and then if none
        // cmd conveyorStop
        // (stateTrg_shooting.or(stateTrg_leavingBeamBreak).or(stateTrg_leftBeamBreak))
        extStateTrg_shooting
            .onTrue(m_conveyor.runFast())
            .onFalse(m_conveyor.stop());

        // if note in shooter sensor and state shooting
        // state -> LEAVING_BEAM_BREAK, timothy says bye
        (trg_shooterSensor.and(stateTrg_shooting))
            .onTrue(Commands.runOnce(() -> {
                m_state = NoteState.LEAVING_BEAM_BREAK;
                timothyFieldTrip = true;
            }));

        // if note not in shooter and state leaving and timothy waving bye
        // state -> LEFT_BEAM_BREAK
        (trg_shooterSensor.negate().and(stateTrg_leavingBeamBreak).and(trg_timothyFieldTrip))
            .onTrue(Commands.runOnce(() -> m_state = NoteState.LEFT_BEAM_BREAK));

        // if left beam break
        // wait for 0.5 sec
        // state -> IDLE
        stateTrg_leftBeamBreak
            .onTrue(Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> m_state = NoteState.IDLE)));
        stateTrg_idle
            .onTrue(Commands.parallel(
                Commands.runOnce(
                    () -> {
                        timothyEntered = false;
                        timothyIn = false;
                        timothyFieldTrip = false;
                        frontVisiSightSeenNote = false;
                        driverShootReq = false;
                        autonIntake = false;
                        autonShoot = false;
                    }),
                stopEverything(), m_aim.hardStop()));
    }

    public Command stopEverything() {
        var shootCmd = m_shooter.stop();
        var conveyorCmd = m_conveyor.stop();
        var intakeCmd = m_intake.stop();

        return Commands.parallel(shootCmd, conveyorCmd, intakeCmd);
    }

    public Command aimAndShoot(Supplier<Measure<Angle>> target) {
        var aimCmd = m_aim.toAngleUntilAt(target, Degrees.of(2)); // TODO check this and make this unmagical 🙁

        return Commands.sequence(
            Commands.parallel(
                aimCmd,
                Commands.waitUntil(() -> m_state == NoteState.NOTE_READY)),
            Commands.runOnce(() -> {
                driverShootReq = true;
            }));
    }

    public Command stopRequestingToShoot() {
        return Commands.runOnce(() -> driverShootReq = false);
    }

    public Command backwardsRun() {
        var shooterCmd = m_shooter.runBackwards();
        var conveyorCmd = m_conveyor.runBackwards();

        return Commands.parallel(shooterCmd, conveyorCmd);
    }

    public void fastPeriodic() {
        log_state.accept((double) m_state.idx);
        log_timothyEntered.accept(timothyEntered);
        log_timothyIn.accept(timothyIn);
        log_timothyFieldTrip.accept(timothyFieldTrip);
        log_frontVisiSight.accept(bs_frontVisiSight.getAsBoolean());
        log_shooterBeamBreak.accept(bs_shooterBeamBreak.getAsBoolean());
        log_frontVisiSightIrq.accept(frontVisiSightSeenNote);
        log_drvShootReq.accept(trg_driverIntakeReq.getAsBoolean());
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

    public Command autonIntakeCmd() {
        return runOnce(() -> {
            autonIntake = true;
            autonShoot = false;
        });
    }

    public Command waitUntilIdle() {
        return Commands.waitUntil(() -> m_state == NoteState.IDLE);
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
