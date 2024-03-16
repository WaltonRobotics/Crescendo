package frc.robot.subsystems;

import static frc.robot.Constants.IntakeK.kVisiSightId;
import static frc.robot.Constants.RobotK.kDbTabName;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SynchronousInterrupt;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.Conveyor;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.*;

public class Superstructure {
    public final Aim m_aim;
    public final Intake m_intake;
    public final Conveyor m_conveyor;
    public final Shooter m_shooter;

    private final DoubleConsumer m_driverRumbler, m_manipRumbler;

    // Only for logging purposes. Edges are handled by the SynchronousInterrupt
    private final DigitalInput frontVisiSight = new DigitalInput(kVisiSightId);
    private final BooleanSupplier bs_frontVisiSight = () -> frontVisiSight.get();
    private final DigitalInput conveyorBeamBreak = new DigitalInput(0);
    private final DigitalInput shooterBeamBreak = new DigitalInput(1);

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

    public final Trigger stateTrg_idle = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.IDLE);
    private final Trigger stateTrg_intake = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.INTAKE);
    private final Trigger stateTrg_noteRetracting = new Trigger(sensorEventLoop,
        () -> m_state == NoteState.ROLLER_BEAM_RETRACT);
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
    // TODO: move to SynchronousInterrupt and handle in fastPeriodic()
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

    private final IntLogger log_state = WaltLogger.logInt(kDbTabName, "state",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_driverIntakeReq = WaltLogger.logBoolean(kDbTabName, "intakeButton");
    private final BooleanLogger log_driverShootReq = WaltLogger.logBoolean(kDbTabName, "shootButton");
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
            if (m_state == state) { return; }
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

        // note in shooter and not shooting
        (irqTrg_conveyorBeamBreak.and((extStateTrg_shooting).negate()))
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
        ((irqTrg_conveyorBeamBreak.negate()).and(stateTrg_noteRetracting))
            .onTrue(
                Commands.sequence(
                    changeStateCmd(NoteState.NOTE_READY),
                    m_conveyor.stop()));

        // added back shoot ok
        (trg_spunUp.and(trg_atAngle).and(stateTrg_noteReady))
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
            .onTrue(changeStateCmd(NoteState.LEAVING_BEAM_BREAK));  

        // if note not in shooter and state leaving and timothy waving bye :D
        // state -> LEFT_BEAM_BREAK
        ((irqTrg_shooterBeamBreak.debounce(0.1)).negate().and(stateTrg_leavingBeamBreak))
            .onTrue(changeStateCmd(NoteState.LEFT_BEAM_BREAK));

        // if left beam break for 0.1 sec
        // state -> IDLE
        (stateTrg_leftBeamBreak.debounce(0.1))
            .onTrue(changeStateCmd(NoteState.IDLE));

        (stateTrg_idle.and(RobotModeTriggers.autonomous().negate()))
            .onTrue(Commands.parallel(
                Commands.runOnce(
                    () -> {
                        frontVisiSightSeenNote = false;
                        autonIntake = false;
                        autonShoot = false;
                        driverRumbled = false;
                        manipulatorRumbled = false;
                    }),
                stopEverything(), m_aim.intakeAngleNearCmd()));

        (stateTrg_idle.and(RobotModeTriggers.autonomous()))
            .onTrue(Commands.parallel(
                Commands.runOnce(
                    () -> {
                        frontVisiSightSeenNote = false;
                        autonIntake = false;
                        autonShoot = false;
                        driverRumbled = false;
                        manipulatorRumbled = false;
                    }),
                autonStop(), m_aim.intakeAngleNearCmd()));
    }

    public Command stopEverything() {
        var shootCmd = m_shooter.stop();
        var conveyorCmd = m_conveyor.stop();
        var intakeCmd = m_intake.stop();

        return Commands.parallel(shootCmd, conveyorCmd, intakeCmd);
    }

    public Command autonStop() {
        var conveyorCmd = m_conveyor.stop();
        var intakeCmd = m_intake.stop();

        return Commands.parallel(conveyorCmd, intakeCmd);
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
        // TODO: intake sensor sync eval
        evaluateConveyorIrq();
        evaluateShooterIrq();

        log_frontVisiSight.accept(bs_frontVisiSight);
        log_conveyorBeamBreak.accept(bs_conveyorBeamBreak);
        log_shooterBeamBreak.accept(bs_shooterBeamBreak);
        log_frontVisiSightIrq.accept(frontVisiSightSeenNote);
        log_conveyorBeamBreakIrq.accept(conveyorBeamBreakIrq);
        log_shooterBeamBreakIrq.accept(shooterBeamBreakIrq);
        log_driverIntakeReq.accept(trg_driverIntakeReq);
        log_driverShootReq.accept(trg_driverShootReq);
        log_autonIntakeReq.accept(autonIntake);
        log_autonShootReq.accept(autonShoot);
        log_aimReady.accept(trg_atAngle);

        sensorEventLoop.poll();

        // log state after, so it represents the event loop changes
        log_state.accept(m_state.idx);
    }

    public Command forceStateToNoteReady() {
        return Commands.runOnce(() -> {
            autonShoot = false;
            autonIntake = false;
            m_state = NoteState.NOTE_READY;
        });
    }

    public Command preloadShootReq() {
        return Commands.runOnce(() -> {
            autonIntake = false;
            autonShoot = true;
        });
    }

    public Command forceStateToShooting() {
        return Commands.runOnce(() -> {
            autonIntake = false;
            autonShoot = true;
            m_state = NoteState.SHOOTING;
        });
    }

    public Command forceStateToIdle() {
        return Commands.runOnce(() -> m_state = NoteState.IDLE);
    }

    public Command autonShootReq() {
        return Commands.runOnce(() -> autonShoot = true);
    }

    public Command autonIntakeCmd() {
        return Commands.runOnce(() -> {
            autonIntake = true;
            autonShoot = false;
            m_state = NoteState.INTAKE;
        });
    }

    public enum NoteState {
        IDLE(0),
        INTAKE(1),
        ROLLER_BEAM_RETRACT(2), // Note hit top beam
        NOTE_READY(3),
        SHOOT_OK(4),
        SHOOTING(5),
        LEAVING_BEAM_BREAK(6),
        LEFT_BEAM_BREAK(7);

        public final int idx;

        private NoteState(int idx) {
            this.idx = idx;
        }
    }
}
