package frc.robot.subsystems.shooter;

import static frc.robot.Constants.RobotK.kSimInterval;
import static frc.robot.Constants.ShooterK.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterK.ShooterConfigs;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.BooleanLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

import static frc.robot.Constants.ShooterK.FlywheelSimK.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.kCanbus;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final TalonFX m_left = new TalonFX(kLeftId, kCanbus);
    private final TalonFX m_right = new TalonFX(kRightId, kCanbus);
    private final VelocityVoltage m_request = new VelocityVoltage(0);
    private final VoltageOut m_voltage = new VoltageOut(0);
    private final CoastOut m_coast = new CoastOut();

    private double m_spinAmt = kSpinAmt;
    private double m_shotTime = 1.5;

    private Measure<Velocity<Angle>> m_leftTarget = RotationsPerSecond.of(0);
    private Measure<Velocity<Angle>> m_rightTarget = RotationsPerSecond.of(0);
    private final Supplier<Measure<Velocity<Angle>>> m_leftTargetSupp = () -> m_leftTarget;
    // unused thing was bothering me.
    // private final Supplier<Measure<Velocity<Angle>>> m_rightTargetSupp = () ->
    // m_rightTarget;
    private boolean m_spunUp = false;
    private boolean m_leftOk = false;
    private boolean m_rightOk = false;

    private final DoubleLogger log_leftTargetRpm = WaltLogger.logDouble(kDbTabName, "leftTargetRpm");
    private final DoubleLogger log_rightTargetRpm = WaltLogger.logDouble(kDbTabName, "rightTargetRpm");
    private final DoubleLogger log_spinAmt = WaltLogger.logDouble(kDbTabName, "spinAmt");
    private final DoubleLogger log_shotTime = WaltLogger.logDouble(kDbTabName, "shotTime");

    private final DoubleLogger log_leftTarget = WaltLogger.logDouble(kDbTabName, "leftTarget");
    private final DoubleLogger log_rightTarget = WaltLogger.logDouble(kDbTabName, "rightTarget");

    private final DoubleLogger log_leftError = WaltLogger.logDouble(kDbTabName, "leftError");
    private final DoubleLogger log_rightError = WaltLogger.logDouble(kDbTabName, "rightError");

    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kDbTabName, "spunUp");
    private final BooleanLogger log_leftOk = WaltLogger.logBoolean(kDbTabName, "leftOk");
    private final BooleanLogger log_rightOk = WaltLogger.logBoolean(kDbTabName, "rightOk");

    private boolean found = false;
    private final FlywheelSim m_flywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), kGearRatio, kMoi);

    // private final SysIdRoutine m_currentSysId = makeTorqueCurrentSysIdRoutine(
    // Amps.of(8).per(Second),
    // Amps.of(35),
    // Seconds.of(25));

    private final SysIdRoutine m_sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second),
            Volts.of(7),
            Seconds.of(15),
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
            m_left.setControl(m_voltage.withOutput(volts.in(Volts)));
            m_right.setControl(m_voltage.withOutput(volts.in(Volts)));
        }, null, this));

    // private SysIdRoutine makeTorqueCurrentSysIdRoutine(
    // final Measure<Velocity<Current>> currentRampRate,
    // final Measure<Current> stepCurrent,
    // final Measure<Time> timeout) {
    // var cfg = new SysIdRoutine.Config(
    // // we need to lie to SysId here, because it only takes voltage instead of
    // // current
    // Volts.per(Second).of(currentRampRate.baseUnitMagnitude()),
    // Volts.of(stepCurrent.baseUnitMagnitude()),
    // timeout,
    // state -> SignalLogger.writeString("state", state.toString()));
    // var mech = new SysIdRoutine.Mechanism(
    // (voltageMeasure) -> {
    // m_left.setControl(m_current.withOutput(voltageMeasure.in(Volts)));
    // m_right.setControl(m_current.withOutput(voltageMeasure.in(Volts)));
    // }, null, this);

    // return new SysIdRoutine(cfg, mech);
    // }

    public Shooter() {
        m_right.getConfigurator().apply(ShooterConfigs.kRightConfigs);
        m_left.getConfigurator().apply(ShooterConfigs.kLeftConfigs);
        m_left.setInverted(true);

        m_right.setControl(m_coast);
        m_left.setControl(m_coast);
    }

    public Command stop() {
        return runOnce(() -> {
            m_right.setControl(m_coast);
            m_left.setControl(m_coast);
        });
    }


    public Command toVelo(Supplier<Measure<Velocity<Angle>>> velo, BooleanSupplier idle) {
        Runnable spin = () -> {
            var velMeas = velo.get();
            m_rightTarget = velMeas.times(m_spinAmt);
            m_leftTarget = velMeas;
            var right = m_rightTarget.in(RotationsPerSecond);
            var left = m_leftTarget.in(RotationsPerSecond);

            // withSlot(0) to use slot 0 PIDFF gains for powerful shots
            m_right.setControl(m_request.withVelocity(right).withSlot(0));
            m_left.setControl(m_request.withVelocity(left).withSlot(0));
        };

        Consumer<Boolean> stopSpin = (interrupted) -> {
            boolean isAuton = !idle.getAsBoolean();
            System.out.println("toVelo STOPPED. int: " + interrupted + ", auton: " + isAuton);
            m_rightTarget = RotationsPerSecond.of(0);
            m_leftTarget = RotationsPerSecond.of(0);
            m_right.setControl(m_request.withVelocity(0).withSlot(0));
            m_left.setControl(m_request.withVelocity(0).withSlot(0));
            m_right.setControl(m_coast);
            m_left.setControl(m_coast);
        };

        return new FunctionalCommand(spin, () -> {}, stopSpin, idle, this)
            .withName("ShooterToVelo");
    }

    public Command toVelo(Supplier<Measure<Velocity<Angle>>> velo, BooleanSupplier idle, double spinAmt) {
        int slot;
        if (spinAmt == 1) slot = 1;
        else slot = 0;
        Runnable spin = () -> {
            var velMeas = velo.get();
            m_rightTarget = velMeas.times(spinAmt);
            m_leftTarget = velMeas;
            var right = m_rightTarget.in(RotationsPerSecond);
            var left = m_leftTarget.in(RotationsPerSecond);

            // withSlot(0) to use slot 0 PIDFF gains for powerful shots
            m_right.setControl(m_request.withVelocity(right).withSlot(slot));
            m_left.setControl(m_request.withVelocity(left).withSlot(slot));
        };

        Consumer<Boolean> stopSpin = (interrupted) -> {
            boolean isAuton = !idle.getAsBoolean();
            System.out.println("toVelo STOPPED. int: " + interrupted + ", auton: " + isAuton);
            m_rightTarget = RotationsPerSecond.of(0);
            m_leftTarget = RotationsPerSecond.of(0);
            m_right.setControl(m_request.withVelocity(0).withSlot(slot));
            m_left.setControl(m_request.withVelocity(0).withSlot(slot));
            m_right.setControl(m_coast);
            m_left.setControl(m_coast);
        };

        return new FunctionalCommand(spin, () -> {}, stopSpin, idle, this)
            .withName("ShooterToVelo");
    }

    private Command toVeloNoSpin(Supplier<Measure<Velocity<Angle>>> velo) {
        return toVelo(velo, () -> false, 1);
    }

    public Command increaseRpm() {
        return Commands.runOnce(() -> {
            m_leftTarget = m_leftTarget.plus(Rotations.per(Minute).of(100));
        });
    }

    public Command decreaseRpm() {
        return Commands.runOnce(() -> {
            m_leftTarget = m_leftTarget.minus(Rotations.per(Minute).of(100));
        });
    }

    public Command moreSpin() {
        return Commands.runOnce(() -> m_spinAmt -= 0.05);
    }

    public Command lessSpin() {
        return Commands.runOnce(() -> m_spinAmt += 0.05);
    }

    public Command subwoofer() {
        return toVelo(() -> Rotations.per(Minute).of(kSubwooferRpm), () -> false).withName("ShooterToVelo_SubwooferTele");
    }

    public Command farShot() {
        return toVelo(() -> Rotations.per(Minute).of(8000), () -> false).withName("ShooterToVelo_FarShotTele");
    }

    public Command farShotNoSpin() {
        return toVeloNoSpin(() -> Rotations.per(Minute).of(8000)).withName("ShooterToVelo_FarShotNoSpin");
    }

    public Command subwoofer(BooleanSupplier idle) {
        return toVelo(() -> Rotations.per(Minute).of(kSubwooferRpm), idle).withName("ShooterToVelo_SubwooferAuton");
    }

    public Command podium(BooleanSupplier idle) {
        return toVelo(() -> Rotations.per(Minute).of(kPodiumRpm), idle);
    }

    public Command ampShot() {
        return toVeloNoSpin(() -> Rotations.per(Minute).of(kAmpRpm));
    }

    public Command trapShot() {
        return toVeloNoSpin(() -> Rotations.per(Minute).of(kTrapRpm));
    }

    public boolean spinUpFinished() {
        if (m_leftTarget.baseUnitMagnitude() == 0) {
            m_leftOk = false;
            m_rightOk = false;
            return false;
        }
        var leftMeas = m_leftTargetSupp.get();
        var tolerance = leftMeas.gte(RotationsPerSecond.of(40)) ? kBigShootTolerance : kAmpTolerance;

        var leftCleMeas = RotationsPerSecond.of(m_left.getClosedLoopError().getValueAsDouble());
        var rightCleMeas = RotationsPerSecond.of(m_right.getClosedLoopError().getValueAsDouble());

        m_leftOk = leftCleMeas.lte(tolerance);
        m_rightOk = rightCleMeas.lte(tolerance);
        m_spunUp = m_leftOk && m_rightOk;
        return m_spunUp;
    }

    private void rawRun(double dutyCycle) {
        m_left.set(dutyCycle * m_spinAmt);
        m_right.set(dutyCycle);
    }

    public Command runBackwards() {
        return runEnd(
            () -> {
                rawRun(-0.5);
            }, () -> {
                m_left.set(0);
                m_right.set(0);
            });
    }

    public void periodic() {
        log_leftTargetRpm.accept(m_leftTarget.in(Rotations.per(Minute)));
        log_rightTargetRpm.accept(m_rightTarget.in(Rotations.per(Minute)));
        log_spinAmt.accept(m_spinAmt);
        log_shotTime.accept(m_shotTime);
        log_leftTarget.accept(m_left.getClosedLoopReference().getValueAsDouble());
        log_rightTarget.accept(m_right.getClosedLoopReference().getValueAsDouble());

        log_leftError.accept(m_left.getClosedLoopError().getValueAsDouble());
        log_rightError.accept(m_right.getClosedLoopError().getValueAsDouble());
        log_spunUp.accept(m_spunUp);
        log_leftOk.accept(m_leftOk);
        log_rightOk.accept(m_rightOk);
    }

    public void simulationPeriodic() {
        m_right.getSimState().setSupplyVoltage(12);
        var volts = m_right.getSimState().getMotorVoltage();
        // TODO: check voltage
        m_flywheelSim.setInputVoltage(volts);

        if (m_flywheelSim.getAngularVelocityRPM() >= m_leftTarget.in(Rotations.per(Minute)) && !found) {
            found = true;
        }

        m_flywheelSim.update(kSimInterval);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysId.dynamic(direction);
    }
}
