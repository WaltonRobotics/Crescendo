package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.IntakeK.kVisiSightId;
import static frc.robot.Constants.RobotK.kDbTabName;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SynchronousInterrupt;
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
    private final DigitalInput conveyorBeamBreak = new DigitalInput(0);
    private final DigitalInput shooterBeamBreak = new DigitalInput(1);
    private final BooleanSupplier bs_frontVisiSight = () -> frontVisiSight.get();

    private final BooleanSupplier bs_conveyorBeamBreak = () -> !conveyorBeamBreak.get();
    private final BooleanSupplier bs_shooterBeamBreak = () -> !shooterBeamBreak.get();

    private final BooleanLogger log_frontVisiSight = WaltLogger.logBoolean("Sensors", "frontVisiSight",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_frontVisiSightIrq = WaltLogger.logBoolean("Sensors", "frontVisiSightIrq",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_conveyorBeamBreakIrq = WaltLogger.logBoolean("Sensors", "conveyorBeamBreakIrq",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_conveyorBeamBreak = WaltLogger.logBoolean("Sensors", "conveyorBeamBreak",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_shooterBeamBreakIrq = WaltLogger.logBoolean("Sensors", "shooterBeamBreakIrq",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_shooterBeamBreak = WaltLogger.logBoolean("Sensors", "shooterBeamBreak",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_autonIntakeReq = WaltLogger.logBoolean(kDbTabName, "autonIntakeReq",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_autonShootReq = WaltLogger.logBoolean(kDbTabName, "autonShootReq",
        PubSubOption.sendAll(true)); 

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
    // private final Trigger trg_shooterSensor;
    /** true = has note */
    private final Trigger trg_frontSensorIrq;

    public final Trigger trg_spunUp;
    private final Trigger trg_atAngle;

    private final Trigger trg_intakeReq;
    private final Trigger trg_shootReq;

    private final Trigger trg_timothyFieldTrip = new Trigger(sensorEventLoop,
        () -> timothyFieldTrip);

    public final Trigger stateTrg_idle = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.IDLE);
    private final Trigger stateTrg_intake = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.INTAKE);
    private final Trigger stateTrg_noteRetracting = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.ROLLER_BEAM_RETRACT);
    private final Trigger stateTrg_spinUp = new Trigger(sensorEventLoop, () -> m_state == NoteState.SHOT_SPINUP);
    private final Trigger stateTrg_shootOk = new Trigger(sensorEventLoop, () -> m_state == NoteState.SHOOT_OK);
    private final Trigger stateTrg_shooting = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.SHOOTING);
        private final Trigger stateTrg_leavingBeamBreak = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.LEAVING_BEAM_BREAK);
        private final Trigger stateTrg_leftBeamBreak = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.LEFT_BEAM_BREAK);
    private final Trigger extStateTrg_shooting = stateTrg_shooting.or(stateTrg_leavingBeamBreak).or(stateTrg_leftBeamBreak);
    public final Trigger stateTrg_noteReady = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.NOTE_READY);

    /** To be set on any edge from the AsyncIrq callback  */
    private boolean frontVisiSightSeenNote = false;
    private final AsynchronousInterrupt ai_frontVisiSight = new AsynchronousInterrupt(frontVisiSight,
        (Boolean rising, Boolean falling) -> {
            if ((rising || falling) && !frontVisiSightSeenNote) {
                frontVisiSightSeenNote = true;
                log_frontVisiSightIrq.accept(true);
            }
        });

    private boolean conveyorBeamBreakIrq = false;
    private double conveyorBeamBreakIrqLastRising = 0;
    private double conveyorBeamBreakIrqLastFalling = 0;
    private final SynchronousInterrupt irq_conveyorBeamBreak = new SynchronousInterrupt(conveyorBeamBreak);

    private boolean shooterBeamBreakIrq = false;
    private double shooterBeamBreakIrqLastRising = 0;
    private double shooterBeamBreakIrqLastFalling = 0;
    private final SynchronousInterrupt irq_shooterBeamBreak = new SynchronousInterrupt(shooterBeamBreak);

    private final Trigger irqTrg_conveyorBeamBreak;
    private final Trigger irqTrg_shooterBeamBreak;

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

        irq_conveyorBeamBreak.setInterruptEdges(true, true);
        irq_shooterBeamBreak.setInterruptEdges(true, true);

        trg_frontSensorIrq = new Trigger(sensorEventLoop, () -> frontVisiSightSeenNote);
        
        irqTrg_conveyorBeamBreak = new Trigger(sensorEventLoop, () -> conveyorBeamBreakIrq);
        irqTrg_shooterBeamBreak = new Trigger(sensorEventLoop, () -> shooterBeamBreakIrq);


        trg_spunUp = new Trigger(m_shooter::spinUpFinished).debounce(0.05);
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
        return Commands.startEnd(
            () -> {
                if (!driverRumbled) {
                    m_driverRumbler.accept(intensity);
                    driverRumbled = true;
                }
            },
            () -> m_driverRumbler.accept(0)).withTimeout(seconds);
    }

    private Command cmdManipRumble(double intensity, double seconds) {
        return Commands.startEnd(
            () -> {
                if (!manipulatorRumbled) {
                    m_manipRumbler.accept(intensity);
                    manipulatorRumbled = true;
                }
            },
            () -> m_manipRumbler.accept(0)).withTimeout(seconds);
    }

    private Command changeStateCmd(NoteState state) {
        return Commands.runOnce(() -> {
            if (m_state == state) {
                return;
            }

            var oldState = m_state;
            m_state = state;
        });
    }

    private void configureStateTriggers() {
        irqTrg_conveyorBeamBreak.onTrue(Commands.none());

        // intakeReq && idle
        (trg_intakeReq.and(stateTrg_idle))
            .onTrue(Commands.runOnce(() -> m_state = NoteState.INTAKE));
        
        stateTrg_intake.onTrue(
            // wait until aim is ±50 degrees to intake mode
            Commands.sequence(
                m_aim.intakeAngleNearCmd(),
                Commands.parallel(m_intake.run(), m_conveyor.startSlow())));

        // !(intakeReq || idle) => !intakeReq && !idle
        (trg_intakeReq.or(trg_frontSensorIrq)).onFalse(changeStateCmd(NoteState.IDLE));

        trg_frontSensorIrq
            .onTrue(
                Commands.parallel(
                    cmdDriverRumble(1, 0.5),
                    cmdManipRumble(1, 0.5)
                )
            );

        // note in shooter and not shooting or spinupping
        (irqTrg_conveyorBeamBreak.and((extStateTrg_shooting.or(stateTrg_spinUp)).negate()))
            .onTrue(
                Commands.parallel(
                    changeStateCmd(NoteState.ROLLER_BEAM_RETRACT),
                    Commands.runOnce(() -> driverRumbled = false)
                )
            );

        stateTrg_noteRetracting.onTrue(
            Commands.parallel(
                m_intake.stop(),
                m_conveyor.retract()));

        // !beamBreak && noteRetracting
        // Post-retract stop
        ((irqTrg_conveyorBeamBreak.negate().debounce(0.35)).and(stateTrg_noteRetracting))
            .onTrue(
                Commands.sequence(
                    changeStateCmd(NoteState.NOTE_READY),
                    m_conveyor.stop()));

        // added back shoot ok
        (trg_spunUp.and(trg_atAngle).and(stateTrg_spinUp))
            .onTrue(
                Commands.parallel(
                    cmdDriverRumble(1, 0.5), 
                    changeStateCmd(NoteState.SHOOT_OK)
                )
            );

        // if shooter spun up and asked to shoot
        // state -> SHOOTING
        (stateTrg_shootOk.and(trg_shootReq))
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
        // state -> LEAVING_BEAM_BREAK, timothy says bye :D
        (irqTrg_shooterBeamBreak.and(stateTrg_shooting))
            .onTrue(Commands.parallel(
                changeStateCmd(NoteState.LEAVING_BEAM_BREAK),
                Commands.runOnce(() -> timothyFieldTrip = true)));

        // if note not in shooter and state leaving and timothy waving bye :D
        // state -> LEFT_BEAM_BREAK
        ((irqTrg_shooterBeamBreak.debounce(0.1)).negate().and(stateTrg_leavingBeamBreak).and(trg_timothyFieldTrip))
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

    public Command aimAndSpinUp(Measure<Angle> target, boolean podium) {
        var aimCmd = m_aim.toAngleUntilAt(target, Degrees.of(1)); // TODO make this unmagical :(

        var waitForNoteReady = Commands.waitUntil(() -> m_state.idx > NoteState.ROLLER_BEAM_RETRACT.idx)
            .andThen(Commands.print("====NOTE READY===="));

        var shootCmd = podium ? m_shooter.podium(stateTrg_idle) : m_shooter.subwoofer(stateTrg_idle); 


        return Commands.sequence(
            Commands.parallel(
                aimCmd.asProxy().andThen(Commands.print("AimAndSpinUp_AIM_DONE")),
                Commands.sequence(
                    waitForNoteReady.andThen(Commands.print("AimAndSpinUp_NOTERD_DONE")),
                    changeStateCmd(NoteState.SHOT_SPINUP),
                    Commands.parallel(
                        shootCmd.asProxy().andThen(Commands.print("shooty shoot done (no yelling)"))
                    )
                )
            )
        );
    }

    public Command ampSpinUp(Measure<Angle> target) {
        var waitForNoteReady = Commands.waitUntil(() -> m_state.idx > NoteState.ROLLER_BEAM_RETRACT.idx)
            .andThen(Commands.print("====NOTE READY===="));

        Command shootCmd = m_shooter.ampShot();

        var aimCmd2 = m_aim.toAngleUntilAt(() -> target.plus(Degrees.of(20)), Degrees.of(0));

        return Commands.sequence(
            waitForNoteReady.andThen(Commands.print("AimAndSpinUp_NOTERD_DONE")),
            changeStateCmd(NoteState.SHOT_SPINUP),
            Commands.parallel(
                shootCmd.asProxy().andThen(Commands.print("shooty shoot done (no yelling)")),
                Commands.sequence(
                    Commands.waitUntil(irqTrg_conveyorBeamBreak),
                    aimCmd2.asProxy().andThen(Commands.print("aim"))
                )
            )  
        );
    }

    public Command subwooferSpinUp() {
        var waitForNoteReady = Commands.waitUntil(() -> m_state.idx > NoteState.ROLLER_BEAM_RETRACT.idx)
            .andThen(Commands.print("====NOTE READY===="));

        Command shootCmd = m_shooter.subwoofer();

        return Commands.sequence(
            waitForNoteReady.andThen(Commands.print("AimAndSpinUp_NOTERD_DONE")),
            changeStateCmd(NoteState.SHOT_SPINUP),
            shootCmd.asProxy().andThen(Commands.print("shooty shoot done (no yelling)"))
        );
    }

    public Command backwardsRun() {
        var shooterCmd = m_shooter.runBackwards();
        var conveyorCmd = m_conveyor.runBackwards();

        return Commands.parallel(
            shooterCmd,
            Commands.sequence(
                Commands.waitUntil(trg_spunUp), 
                conveyorCmd
            )
        );
    }

    private void evaluateConveyorIrq() {
        double latestRising = irq_conveyorBeamBreak.getRisingTimestamp();
        double latestFalling = irq_conveyorBeamBreak.getFallingTimestamp();
        boolean risingNew = latestRising > conveyorBeamBreakIrqLastRising;
        if (risingNew) {
            conveyorBeamBreakIrqLastRising = latestRising;
        }

        boolean fallingNew = latestFalling > conveyorBeamBreakIrqLastFalling;
        if (fallingNew) {
            conveyorBeamBreakIrqLastFalling = latestFalling;
        }

        if (latestFalling > latestRising && fallingNew) {
            conveyorBeamBreakIrq = true;
        } else if (latestRising > latestFalling && risingNew) {
            conveyorBeamBreakIrq = false;
        }
    }

    private void evaluateShooterIrq() {
        double latestRising = irq_shooterBeamBreak.getRisingTimestamp();
        double latestFalling = irq_shooterBeamBreak.getFallingTimestamp();
        boolean risingNew = latestRising > shooterBeamBreakIrqLastRising;
        if (risingNew) {
            shooterBeamBreakIrqLastRising = latestRising;
        }

        boolean fallingNew = latestFalling > shooterBeamBreakIrqLastFalling;
        if (fallingNew) {
            shooterBeamBreakIrqLastFalling = latestFalling;
        }

        if (latestFalling > latestRising && fallingNew) {
            shooterBeamBreakIrq = true;
        } else if (latestRising > latestFalling && risingNew) {
            shooterBeamBreakIrq = false;
        }
    }

    public void fastPeriodic() {
        evaluateConveyorIrq();
        evaluateShooterIrq();

        log_state.accept((double) m_state.idx);
        log_timothyEntered.accept(timothyEntered);
        log_timothyIn.accept(timothyIn);
        log_timothyFieldTrip.accept(timothyFieldTrip);
        log_frontVisiSight.accept(bs_frontVisiSight.getAsBoolean());
        log_conveyorBeamBreak.accept(bs_conveyorBeamBreak.getAsBoolean());
        log_shooterBeamBreak.accept(bs_shooterBeamBreak.getAsBoolean());
        log_frontVisiSightIrq.accept(frontVisiSightSeenNote);
        log_conveyorBeamBreakIrq.accept(conveyorBeamBreakIrq);
        log_shooterBeamBreakIrq.accept(shooterBeamBreakIrq);
        log_driverIntakeReq.accept(trg_driverIntakeReq.getAsBoolean());
        log_autonIntakeReq.accept(autonIntake);
        log_autonShootReq.accept(autonShoot);
        log_aimReady.accept(trg_atAngle.getAsBoolean());
    }

    public Command forceStateToNoteReady() {
        return runOnce(() -> {
            timothyEntered = true;
            timothyIn = true;
            autonShoot = false;
            autonIntake = false;
            m_state = NoteState.NOTE_READY;
        });
    }

    public void preloadShootReq() {
        timothyEntered = true;
        timothyIn = true;
        autonIntake = false;
        autonShoot = true;
    }

    public void forceStateToShooting() {
        timothyEntered = true;
        timothyIn = true;
        autonIntake = false;
        autonShoot = true;
        m_state = NoteState.SHOOTING;
    }

    public void forceStateToIdle() {
        m_state = NoteState.IDLE;
    }

    public Command autonShootReq() {
        return runOnce(() -> autonShoot = true);
    }

    public Command autonIntakeCmd() {
        return runOnce(() -> {
            autonIntake = true;
            autonShoot = false;
            m_state = NoteState.INTAKE;
        });
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
        SHOOT_OK(5),
        SHOOTING(6),
        LEAVING_BEAM_BREAK(7),
        LEFT_BEAM_BREAK(8);

        public final int idx;

        private NoteState(int idx) {
            this.idx = idx;
        }
    }
}
