package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static frc.robot.Constants.AimK.kAmpAngle;
import static frc.robot.Constants.AimK.kSubwooferAngle;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Vision;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.Conveyor;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.CommandDoodads;
import frc.util.WaltRangeChecker;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.*;

import static frc.robot.subsystems.Superstructure.NoteState.*;

public class Superstructure {
    private final Aim m_aim;
    private final Intake m_intake;
    private final Conveyor m_conveyor;
    private final Shooter m_shooter;

    private final DoubleConsumer m_driverRumbler, m_manipRumbler;

    // Only for logging purposes. Edges are handled by the SynchronousInterrupt
    private final DigitalInput frontVisiSight = new DigitalInput(kVisiSightId);
    private final BooleanSupplier bs_frontVisiSight = () -> frontVisiSight.get();
    private final DigitalInput conveyorBeamBreak = new DigitalInput(0);
    private final DigitalInput shooterBeamBreak = new DigitalInput(1);

    private final BooleanSupplier bs_conveyorBeamBreak = () -> !conveyorBeamBreak.get();
    private final BooleanSupplier bs_shooterBeamBreak = () -> !shooterBeamBreak.get();

    public final Timer m_autonTimer = new Timer();
    public int shotNumber = 0;

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

    private boolean autonIntake = false;
    private boolean straightThrough = false;
    private boolean autonShoot = false;
    private boolean driverRumbled = false;
    private boolean manipulatorRumbled = false;
    private boolean trapping = false;

    public final EventLoop sensorEventLoop = new EventLoop();
    public final EventLoop stateEventLoop = new EventLoop();

    /** true = driver wants to intake */
    private final Trigger trg_driverIntakeReq;
    /** true = driver wants to shoot */
    private final Trigger trg_driverShootReq;
    private final Trigger trg_driverAmpReq;
    private final Trigger trg_driverTrapReq;

    private final Trigger trg_autonIntakeReq = new Trigger(() -> autonIntake);
    private final Trigger trg_straightThroughReq = new Trigger(() -> straightThrough);
    private final Trigger trg_autonShootReq = new Trigger(() -> autonShoot);

    private final Trigger trg_trap = new Trigger(() -> trapping);

    /** true = has note */
    private final Trigger irqTrg_frontSensor;

    public final Trigger trg_spunUp;
    public final Trigger trg_atAngle;

    private final Trigger trg_subwooferAngle;
    private final Trigger trg_ampAngle;

    private final Trigger trg_intakeReq;
    private final Trigger trg_shootReq;

    public final Trigger stateTrg_idle = new Trigger(stateEventLoop,
        () -> m_state == IDLE);
    public final Trigger stateTrg_intake = new Trigger(stateEventLoop,
        () -> m_state == INTAKE);
    private final Trigger stateTrg_noteRetracting = new Trigger(stateEventLoop,
        () -> m_state == ROLLER_BEAM_RETRACT);
    public final Trigger stateTrg_shootOk = new Trigger(stateEventLoop, () -> m_state == SHOOT_OK);
    public final Trigger stateTrg_shooting = new Trigger(stateEventLoop,
        () -> m_state == SHOOTING);
    private final Trigger stateTrg_leftBeamBreak = new Trigger(stateEventLoop,
        () -> m_state == LEFT_BEAM_BREAK);
        private final Trigger extStateTrg_noteIn = new Trigger(stateEventLoop, () -> m_state.idx > ROLLER_BEAM_RETRACT.idx);
    public final Trigger extStateTrg_shooting = new Trigger(stateEventLoop, () -> m_state.idx > SHOOT_OK.idx);
    public final Trigger stateTrg_noteReady = new Trigger(stateEventLoop,
        () -> m_state == NOTE_READY);
        
    /** To be set on any edge from the AsyncIrq callback  */
    // TODO: move to SynchronousInterrupt and handle in fastPeriodic()
    private boolean frontVisiSightSeenNote = false;
    private final AsynchronousInterrupt ai_frontVisiSight = new AsynchronousInterrupt(frontVisiSight,
    (Boolean rising, Boolean falling) -> {
        if ((rising || falling) && !frontVisiSightSeenNote && stateTrg_intake.getAsBoolean()) {
            frontVisiSightSeenNote = true;
            log_frontVisiSightIrq.accept(true);
        }
    });
        
    private boolean conveyorBeamBreakIrq = false;
    private double conveyorBeamBreakIrqLastRising = 0;
    private double conveyorBeamBreakIrqLastFalling = 0;
    private final SynchronousInterrupt irq_conveyorBeamBreak = new SynchronousInterrupt(conveyorBeamBreak);
    private final BooleanLogger log_conveyorBeamBreakExtended = 
        WaltLogger.logBoolean(kDbTabName, "conveyorBeamBreakExtended", PubSubOption.sendAll(true));
    
    private boolean shooterBeamBreakIrq = false;
    private double shooterBeamBreakIrqLastRising = 0;
    private double shooterBeamBreakIrqLastFalling = 0;
    private final SynchronousInterrupt irq_shooterBeamBreak = new SynchronousInterrupt(shooterBeamBreak);
    private final BooleanLogger log_shooterBeamBreakExtended =
        WaltLogger.logBoolean(kDbTabName, "shooterBeamBreakExtended", PubSubOption.sendAll(true));

    public final Trigger irqTrg_conveyorBeamBreak;
    public final Trigger irqTrg_shooterBeamBreak;

    private final IntLogger log_state = WaltLogger.logInt(kDbTabName, "state",
        PubSubOption.sendAll(true));
    private final BooleanLogger log_driverIntakeReq = WaltLogger.logBoolean(kDbTabName, "intakeButton");
    private final BooleanLogger log_driverShootReq = WaltLogger.logBoolean(kDbTabName, "shootButton");
    private final BooleanLogger log_aimReady = WaltLogger.logBoolean(kDbTabName, "aimReady");

    private Timer timer = new Timer();
        
    public Superstructure(
        Aim aim, Intake intake, Conveyor conveyor, Shooter shooter, Vision vision,
        Trigger intaking, Trigger shooting, Trigger ampShot, Trigger trapShot,
        DoubleConsumer driverRumbler, DoubleConsumer manipRumbler) {
        m_aim = aim;
        m_intake = intake;
        m_conveyor = conveyor;
        m_shooter = shooter;

        m_driverRumbler = driverRumbler;
        m_manipRumbler = manipRumbler;

        trg_driverIntakeReq = intaking;
        trg_driverShootReq = shooting;
        trg_driverAmpReq = ampShot;
        trg_driverTrapReq = trapShot;

        trg_subwooferAngle = new Trigger(() -> aim.getAngle().isNear(kSubwooferAngle, 0.1));
        trg_ampAngle = new Trigger(() -> aim.getAngle().isNear(kAmpAngle, 0.3));

        trg_intakeReq = trg_driverIntakeReq.or(trg_autonIntakeReq).or(trg_straightThroughReq);
        trg_shootReq = trg_driverShootReq.or(trg_autonShootReq);

        ai_frontVisiSight.setInterruptEdges(true, true);
        ai_frontVisiSight.enable();

        irq_conveyorBeamBreak.setInterruptEdges(true, true);
        irq_shooterBeamBreak.setInterruptEdges(true, true);

        irqTrg_frontSensor = new Trigger(sensorEventLoop, () -> frontVisiSightSeenNote);
        
        // initialize (inverted)
        shooterBeamBreakIrq = !shooterBeamBreak.get();

        irqTrg_conveyorBeamBreak = new Trigger(sensorEventLoop, () -> conveyorBeamBreakIrq);
        irqTrg_conveyorBeamBreak
            .onTrue(Commands.runOnce(()-> log_conveyorBeamBreakExtended.accept(true)).ignoringDisable(true));
        irqTrg_conveyorBeamBreak.negate().debounce(0.1)
            .onTrue(Commands.runOnce(() -> log_conveyorBeamBreakExtended.accept(false)).ignoringDisable(true));

        irqTrg_shooterBeamBreak = new Trigger(sensorEventLoop, () -> shooterBeamBreakIrq);
        irqTrg_shooterBeamBreak
            .onTrue(Commands.runOnce(()-> log_shooterBeamBreakExtended.accept(true)).ignoringDisable(true));
        irqTrg_shooterBeamBreak.negate().debounce(0.1)
            .onTrue(Commands.runOnce(() -> log_shooterBeamBreakExtended.accept(false)).ignoringDisable(true));

        trg_spunUp = new Trigger(m_shooter.spinUpFinished()).debounce(0.05);
        trg_atAngle = new Trigger(m_aim.aimFinished());

        m_state = IDLE;

        configureStateTriggers();
        WaltRangeChecker.addIntegerChecker("ShooterBeamBreak", () -> shooterBeamBreakIrq ? 1 : 0, -1, 1, 1, true);

        configureShootTimer();
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
            var oldState = m_state;
            m_state = state;
            System.out.println("changing state from " + oldState + " to " + m_state);
        }).withName("SuperStateChange_To" + state);
    }

    private void configureShootTimer() {
        trg_driverShootReq
            .onTrue(Commands.runOnce(() -> timer.restart()))
            .onFalse(Commands.runOnce(() -> timer.reset()));

        stateTrg_shooting.onTrue(
            Commands.parallel(
                Commands.runOnce(() -> timer.stop()),
                CommandDoodads.printLater(() -> "[SUPERSTRUCTURE] Time from shoot req to shoot: " + timer.get())
            )
        );
    }

    private void configureStateTriggers() {
        irqTrg_conveyorBeamBreak.onTrue(Commands.none());

        // intakeReq && idle
        (trg_intakeReq.and(stateTrg_idle))
            .onTrue(Commands.runOnce(() -> m_state = INTAKE));

        stateTrg_intake.onTrue(Commands.runOnce(() -> trapping = false));
        
        (stateTrg_intake.and(trg_subwooferAngle.or(RobotModeTriggers.autonomous()).and(trg_straightThroughReq.negate())))
            .onTrue(
                Commands.parallel(m_intake.fullPower(), m_conveyor.startSlower())
            );

        (stateTrg_intake.and(trg_subwooferAngle.negate()).and(trg_straightThroughReq.negate()))
            .onTrue(
                Commands.parallel(m_intake.run(), m_conveyor.start()).withName("AutoIntake")
            );

        stateTrg_intake.and(trg_straightThroughReq)
            .onTrue(
                Commands.parallel(m_intake.fullPower(), m_conveyor.fullPower()).withName("FastIntake")
            );

        (irqTrg_conveyorBeamBreak.or(irqTrg_shooterBeamBreak)).and(trg_straightThroughReq)
            .onTrue(
                changeStateCmd(SHOOTING)
            );

        // !(intakeReq || seenNote) => !intakeReq && !seenNote
        (trg_intakeReq.or(irqTrg_frontSensor)).onFalse(changeStateCmd(IDLE));

        irqTrg_frontSensor
            .onTrue(
                Commands.parallel(
                    cmdDriverRumble(1, 0.5),
                    cmdManipRumble(1, 0.5)
                )
            )
        .and(RobotModeTriggers.autonomous())
            .onTrue(
                CommandDoodads.printLater(() -> "note intaken at " + m_autonTimer.get() + " s")
            );

        // note in shooter and not shooting
        (irqTrg_conveyorBeamBreak.and(irqTrg_frontSensor).and(extStateTrg_shooting.negate())).and(RobotModeTriggers.autonomous().negate())
            .onTrue(
                changeStateCmd(ROLLER_BEAM_RETRACT)
            );

        (irqTrg_conveyorBeamBreak.and((extStateTrg_noteIn).negate())).and(RobotModeTriggers.autonomous()).and(trg_straightThroughReq.negate())
            .onTrue(
                Commands.parallel(
                    m_intake.stop(),
                    m_conveyor.stop(),
                    changeStateCmd(NOTE_READY)
                )
            );

        stateTrg_noteRetracting.onTrue(
            Commands.parallel(
                m_intake.stop(),
                m_conveyor.retract()).withName("RetractNote"));

        // !beamBreak && noteRetracting
        // Post-retract stop
        ((irqTrg_conveyorBeamBreak.negate().debounce(0.125)).and(stateTrg_noteRetracting))
            .onTrue(
                Commands.sequence(
                    changeStateCmd(NOTE_READY),
                    m_conveyor.stop()).withName("NoteReady_StopConveyor"));
        
        (stateTrg_noteReady.and(irqTrg_conveyorBeamBreak)).debounce(0.2).and(RobotModeTriggers.teleop())
            .onTrue(
                changeStateCmd(ROLLER_BEAM_RETRACT)
            );

        stateTrg_noteReady.onTrue(Commands.runOnce(() -> driverRumbled = false));

        // added back shoot ok
        (trg_spunUp.and(trg_atAngle).and(stateTrg_noteReady))
            .onTrue(
                Commands.parallel(
                    Commands.sequence(
                        Commands.waitSeconds(0.1), 
                        cmdDriverRumble(1, 0.5)
                    ),
                    changeStateCmd(SHOOT_OK)
                )
            );

        stateTrg_shootOk
        .and(extStateTrg_shooting.negate())
        .and((trg_spunUp.and(trg_atAngle).negate().debounce(0.5)))
        .and(trg_shootReq.negate()).and(RobotModeTriggers.autonomous().negate())
            .onTrue(changeStateCmd(NOTE_READY));

        // if shooter spun up and asked to shoot
        // state -> SHOOTING
        (stateTrg_shootOk.and(trg_shootReq))
            .onTrue(changeStateCmd(SHOOTING));

        // if now shooting, note leaving, or note just left
        // cmd conveyorFast
        // and then if none
        // cmd conveyorStop
        // (stateTrg_shooting.or(stateTrg_leavingBeamBreak).or(stateTrg_leftBeamBreak))
        extStateTrg_shooting
            .onTrue(m_conveyor.runFast());
        
        // if note in shooter sensor and state shooting
        // state -> LEFT_BEAM_BREAK, timothy says bye :D
        (irqTrg_shooterBeamBreak.and(stateTrg_shooting).and(trg_driverAmpReq.negate()).or(stateTrg_shooting.debounce(0.4)))
            .onTrue(changeStateCmd(LEFT_BEAM_BREAK));  

        (irqTrg_shooterBeamBreak.and(irqTrg_conveyorBeamBreak.negate()).and(stateTrg_shooting).and(trg_driverAmpReq))
            .onTrue(changeStateCmd(LEFT_BEAM_BREAK)); 

        (stateTrg_shooting.debounce(0.4).and(RobotModeTriggers.autonomous()))
            .onTrue(changeStateCmd(IDLE));

        // if left beam break for 0.1 sec
        // state -> IDLE
        (stateTrg_leftBeamBreak.debounce(0.1))
            .onTrue(changeStateCmd(IDLE));

        (stateTrg_idle.and(trg_ampAngle).and(trg_trap.negate()))
            .onTrue(
                Commands.parallel(
                    resetFlags(),
                    ampStop()
                ).withName("AmpStop")
            );
        
        (stateTrg_idle.and((trg_ampAngle.negate().or(trg_trap))))
            .onTrue(
                Commands.parallel(
                    resetFlags(),
                    idleStop()
                ).withName("IdleStop")
            );

        trg_driverTrapReq.onTrue(Commands.runOnce(() -> trapping = true));

        irqTrg_shooterBeamBreak.and(extStateTrg_shooting).and(RobotModeTriggers.autonomous())
            .onTrue(CommandDoodads.printLater(() -> {
                return "Shot " + ++shotNumber + " at " + m_autonTimer.get() + "s of 15.3s";
            })
        );
    }

    private Command resetFlags() { 
        return Commands.runOnce(
            () -> {
                frontVisiSightSeenNote = false;
                autonIntake = false;
                straightThrough = false;
                autonShoot = false;
                driverRumbled = false;
                manipulatorRumbled = false;
            }
        ).withName("SuperStateResetFlags");
    }

    public void resetAutonFlags() {
        autonIntake = false;
        straightThrough = false;
        autonShoot = false;
    }

    public boolean noteReadyOrGreater() {
        return extStateTrg_noteIn.getAsBoolean();
    }

    public Command idleStop() {
        var conveyorCmd = m_conveyor.stop();
        var intakeCmd = m_intake.stop();

        return Commands.parallel(
            conveyorCmd,
            intakeCmd
        ).withName("IdleStop");
    }

    public Command ampStop() {
        var shootCmd = m_shooter.stop();
        var aimCmd = m_aim.toAngleUntilAt(kSubwooferAngle);
        var conveyorCmd = m_conveyor.stop();
        var intakeCmd = m_intake.stop();

        return Commands.parallel(
            shootCmd, 
            aimCmd, 
            conveyorCmd, 
            intakeCmd
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

        log_driverIntakeReq.accept(trg_driverIntakeReq);
        log_driverShootReq.accept(trg_driverShootReq);
        log_autonIntakeReq.accept(autonIntake);
        log_autonShootReq.accept(autonShoot);
        log_aimReady.accept(trg_atAngle);
        
        sensorEventLoop.poll();
        log_frontVisiSight.accept(bs_frontVisiSight);
        log_conveyorBeamBreak.accept(bs_conveyorBeamBreak);
        log_shooterBeamBreak.accept(bs_shooterBeamBreak);
        log_frontVisiSightIrq.accept(frontVisiSightSeenNote);
        log_conveyorBeamBreakIrq.accept(conveyorBeamBreakIrq);
        log_shooterBeamBreakIrq.accept(irqTrg_shooterBeamBreak.getAsBoolean());

        stateEventLoop.poll();

        // log state after, so it represents the event loop changes
        log_state.accept(m_state.idx);
    }

    public Command forceStateToNoteReady() {
        return Commands.parallel(
            changeStateCmd(NOTE_READY),
            Commands.runOnce(() -> {
                autonShoot = false;
                autonIntake = false;
            })
        );
    }


    public Command autonShootReq() {
        return Commands.runOnce(() -> {
            autonIntake = false;
            autonShoot = true;
        });
    }

    public Command forceStateToShooting() {
        return changeStateCmd(SHOOTING);
    }

    public Command forceStateToIntake() {
        return Commands.parallel(
            resetFlags(),
            changeStateCmd(INTAKE)
        );
    }

    public Command autonIntakeReq() {
        return Commands.runOnce(() -> {
            autonIntake = true;
            straightThrough = false;
            autonShoot = false;
        });
    }

    public Command straightThroughReq() {
        return Commands.runOnce(() -> {
            autonIntake = false;
            straightThrough = true;
            autonShoot = false;
        });
    }

    public Command ampShot(Measure<Angle> target) {
        var waitForNoteReady = Commands.waitUntil(() -> m_state.idx > ROLLER_BEAM_RETRACT.idx)
            .andThen(Commands.print("====NOTE READY===="));
        
        var shoot = m_shooter.ampShot();

        var aimCmd = m_aim.toAngleUntilAt(() -> target.plus(Degrees.of(15)), Degrees.of(0));

        return Commands.parallel(
            Commands.print("shoot"),
            shoot,
            Commands.sequence(
            waitForNoteReady.andThen(Commands.print("AimAndSpinUp_NOTERD_DONE")),
                Commands.sequence(
                    Commands.waitUntil(irqTrg_conveyorBeamBreak),
                    Commands.print("going to aim"),
                    aimCmd.asProxy().andThen(Commands.print("aim"))
            )
        ));
    }

    public Command runEverything() {
        return Commands.parallel(
            print("running everything"),
            // m_shooter.subwoofer(),
            m_conveyor.runFast().asProxy(),
            m_intake.fullPower().asProxy()
        );
    }

    public enum NoteState {
        IDLE(0),
        INTAKE(1),
        ROLLER_BEAM_RETRACT(2), // Note hit top beam
        NOTE_READY(3),
        SHOOT_OK(4),
        SHOOTING(5),
        LEFT_BEAM_BREAK(6);

        public final int idx;

        private NoteState(int idx) {
            this.idx = idx;
        }
    }
}
