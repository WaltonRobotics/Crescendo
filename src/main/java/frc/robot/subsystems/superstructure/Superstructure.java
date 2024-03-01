package frc.robot.subsystems.superstructure;

import static frc.robot.Constants.IntakeK.kVisiSightId;
import static frc.robot.Constants.RobotK.kDbTabName;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
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

    private final DigitalInput frontVisiSight = new DigitalInput(kVisiSightId);
	private final DigitalInput shooterBeamBreak = new DigitalInput(0);
	private final BooleanSupplier bs_frontVisiSight = () -> frontVisiSight.get();
	
	private final BooleanSupplier bs_shooterBeamBreak = () -> !shooterBeamBreak.get();
	private final BooleanLogger log_frontVisiSight = 
		WaltLogger.logBoolean("Sensors", "frontVisiSight", PubSubOption.periodic(0.005));
    private final BooleanLogger log_frontVisiSightIrq = 
		WaltLogger.logBoolean("Sensors", "frontVisiSightIrq", PubSubOption.sendAll(true));
	private final BooleanLogger log_shooterBeamBreak = 
		WaltLogger.logBoolean("Sensors", "shooterBeamBreak", PubSubOption.periodic(0.005));

    /** To be set on any edge from the AsyncIrq callback  */
    private boolean frontVisiSight_SeenNote = false;

    private final AsynchronousInterrupt ai_frontVisiSight = new AsynchronousInterrupt(frontVisiSight, (Boolean rising, Boolean falling) -> {
		if ((rising || falling) && !frontVisiSight_SeenNote) {
            frontVisiSight_SeenNote = true;
            log_frontVisiSightIrq.accept(true);
        }
        // TODO: log correctly somehow
	});

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
    private final Trigger trg_shooterSensor;
    
    /** true = has note */ 
    private final Trigger trg_frontSensorIrq;

    private final Trigger trg_spunUp;
    private final Trigger trg_aimed;

    private final Trigger trg_noteRetracting = new Trigger(sensorEvtLoop, () -> m_state == State.ROLLER_BEAM_RETRACT);
    private final Trigger trg_shooting = new Trigger(sensorEvtLoop, () -> m_state == State.SHOOTING);
    private final Trigger trg_d1 = new Trigger(sensorEvtLoop, () -> m_state == State.D1);
    private final Trigger trg_d2 = new Trigger(sensorEvtLoop, () -> m_state == State.D2);
    private final Trigger trg_timothyFieldTrip = new Trigger(sensorEvtLoop, () -> timothyFieldTrip);
    private final Trigger trg_idle = new Trigger(sensorEvtLoop, () -> m_state == State.IDLE);

    private final DoubleLogger log_state = WaltLogger.logDouble(kDbTabName, "state", PubSubOption.periodic(0.005));
    private final BooleanLogger log_timothyEntered = WaltLogger.logBoolean(kDbTabName, "timothyEntered", PubSubOption.periodic(0.005));
    private final BooleanLogger log_timothyIn = WaltLogger.logBoolean(kDbTabName, "timothyIn", PubSubOption.periodic(0.005));
    private final BooleanLogger log_timothyFieldTrip = WaltLogger.logBoolean(kDbTabName, "timothyFieldTrip",PubSubOption.periodic(0.005));
    private final BooleanLogger log_readyToCheck = WaltLogger.logBoolean(kDbTabName, "readyToCheck", PubSubOption.periodic(0.005));
    private final BooleanLogger log_intakeBtn = WaltLogger.logBoolean(kDbTabName, "intakeButton");

    public Superstructure(Aim aim, Intake intake, Conveyor conveyor, Shooter shooter,
        Trigger intaking, Trigger shoot) {
        m_aim = aim;
        m_intake = intake;
        m_conveyor = conveyor;
        m_shooter = shooter;

        trg_driverIntakeReq = intaking;
        trg_driverShootReq = shoot;

		ai_frontVisiSight.setInterruptEdges(true, true);
		ai_frontVisiSight.enable();
            
        trg_frontSensorIrq = new Trigger(sensorEvtLoop, () -> { return frontVisiSight_SeenNote; });
        trg_shooterSensor = new Trigger(sensorEvtLoop, bs_shooterBeamBreak);

        trg_spunUp = new Trigger(m_shooter::spinUpFinished);
        trg_aimed = new Trigger(m_aim::aimFinished);

        m_state = State.IDLE;

        configureStateTriggers();
        // stateTriggersWhileDisabled();
    }

    private void configureStateTriggers() {
        (trg_driverIntakeReq.and(trg_idle)).or(trg_frontSensorIrq).onTrue(Commands.runOnce(() -> m_state = State.INTAKE)
            .alongWith(intake()));
        trg_frontSensorIrq.and(trg_driverIntakeReq).onFalse(Commands.runOnce(() -> m_state = State.IDLE));
        trg_shooterSensor.onTrue(Commands.runOnce(() -> {
            m_state = State.ROLLER_BEAM_RETRACT;
        }));
        trg_noteRetracting.onTrue(
            Commands.sequence(
                Commands.print("ENTER C1"),
                Commands.parallel(
                    m_intake.stop(),
                    m_conveyor.retract()
                )));
        trg_shooterSensor.negate().and(trg_noteRetracting).onTrue(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                m_conveyor.stop(),
                Commands.runOnce(
                    () -> {
                        m_state = State.SHOOT_OK;
                    })));
        trg_driverShootReq.onTrue(Commands.parallel(Commands.runOnce(() -> m_state = State.SHOT_SPINUP), subwooferNoConveyor()));
        trg_spunUp.and(trg_aimed).onTrue(Commands.runOnce(() -> m_state = State.SHOOTING));
        trg_shooting.or(trg_d1).or(trg_d2).whileTrue(m_conveyor.run(true));
        trg_shooterSensor.and(trg_shooting).onTrue(Commands.runOnce(() -> {
            m_state = State.D1;
            timothyFieldTrip = true;
        }));
        trg_d1.onTrue(m_conveyor.stop());
        trg_shooterSensor.and(trg_d1).and(trg_timothyFieldTrip).onTrue(Commands.runOnce(() -> m_state = State.D2));
        trg_d2.onTrue(Commands.sequence(Commands.waitSeconds(0.25),
            Commands.runOnce(() -> m_state = State.IDLE)));
        trg_idle.onTrue(Commands.parallel(
            Commands.runOnce(
            () -> {
                timothyEntered = false;
                readyToCheck = false;
                timothyIn = false;
                timothyFieldTrip = false;
                frontVisiSight_SeenNote = false;
            }), 
            stopEverything()));
    }

    private Command spinUpWait() {
        return Commands.waitSeconds(0.2).andThen(Commands.waitUntil(() -> m_shooter.spinUpFinished()));
    }

    public Command intake() {
        var aimCmd = m_aim.intakeMode();
        var intakeCmd = m_intake.run();
        var conveyorCmd = m_conveyor.start();
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

    public Command subwooferNoConveyor() {
        var aimCmd = m_aim.subwoofer();
        var shootCmd = m_shooter.shoot();

        return Commands.parallel(
            shootCmd,
            aimCmd);
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

    public Command stopEverything() {
        var shootCmd = m_shooter.stop();
        var conveyorCmd = m_conveyor.stop();
        var intakeCmd = m_intake.stop();

        return Commands.parallel(shootCmd, conveyorCmd, intakeCmd);
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

    public void fastPeriodic() {
        log_state.accept((double) m_state.idx);
        log_timothyEntered.accept(timothyEntered);
        log_timothyIn.accept(timothyIn);
        log_timothyFieldTrip.accept(timothyFieldTrip);
        log_readyToCheck.accept(readyToCheck);
        log_frontVisiSight.accept(bs_frontVisiSight.getAsBoolean());
        log_shooterBeamBreak.accept(bs_shooterBeamBreak.getAsBoolean());
        log_frontVisiSightIrq.accept(frontVisiSight_SeenNote);
        log_intakeBtn.accept(trg_driverIntakeReq.getAsBoolean());
    }

    public void backToIdle() {
        m_state = State.IDLE;
    }
}
