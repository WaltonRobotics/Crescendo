package frc.robot.subsystems.shooter;

import static frc.robot.Constants.RobotK.kSimInterval;
import static frc.robot.Constants.ShooterK.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterK.ShooterConfigs;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.BooleanLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

import static frc.robot.Constants.ShooterK.FlywheelSimK.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.kCanbus;
import static frc.robot.Constants.kStickDeadband;
import static frc.robot.Robot.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final TalonFX m_left = new TalonFX(kLeftId, kCanbus);
    private final TalonFX m_right = new TalonFX(kRightId, kCanbus);
    private final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0);
    private final TorqueCurrentFOC m_current = new TorqueCurrentFOC(0);
    private final CoastOut m_coast = new CoastOut();

    private double m_spinAmt = kSpinAmt;
    private double m_shotTime = 1.5;

    private double m_leftTarget = 7000;
    // private double m_rightTarget = 7000 * m_spinAmt;
    private final Supplier<Measure<Velocity<Angle>>> m_leftTargetSupp = () -> Rotations.per(Minute).of(m_leftTarget);
    private boolean m_spunUp = false;
    private boolean m_leftOk = false;
    private boolean m_rightOk = false;
    // private final Supplier<Measure<Velocity<Angle>>> m_rightTargetSupp = () ->
    // Rotations.per(Minute).of(m_rightTarget);

    // private LoggedTunableNumber m_tunableRpm = new
    // LoggedTunableNumber("targetRpm", m_leftTarget);
    // private LoggedTunableNumber m_tunableSpin = new LoggedTunableNumber("spin",
    // m_spinAmt);
    // private LoggedTunableNumber m_tunableShotTime = new
    // LoggedTunableNumber("shotTime", m_shotTime);

    private final DoubleLogger log_targetRpm = WaltLogger.logDouble(kDbTabName, "targetRpm");
    private final DoubleLogger log_spinAmt = WaltLogger.logDouble(kDbTabName, "spinAmt");
    private final DoubleLogger log_shotTime = WaltLogger.logDouble(kDbTabName, "shotTime");

    private final DoubleLogger log_leftTarget = WaltLogger.logDouble(kDbTabName, "leftTarget");
    private final DoubleLogger log_rightTarget = WaltLogger.logDouble(kDbTabName, "rightTarget");

    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kDbTabName, "spunUp");
    private final BooleanLogger log_leftOk = WaltLogger.logBoolean(kDbTabName, "leftOk");
    private final BooleanLogger log_rightOk = WaltLogger.logBoolean(kDbTabName, "rightOk");

    private double time = 0;
    private boolean found = false;
    private final FlywheelSim m_flywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), kGearRatio, kMoi);

    private final SysIdRoutine m_currentSysId = makeTorqueCurrentSysIdRoutine(
        Amps.of(4).per(Second),
        Amps.of(20),
        Seconds.of(25));

    private SysIdRoutine makeTorqueCurrentSysIdRoutine(
        final Measure<Velocity<Current>> currentRampRate,
        final Measure<Current> stepCurrent,
        final Measure<Time> timeout) {
        var cfg = new SysIdRoutine.Config(
            // we need to lie to SysId here, because it only takes voltage instead of
            // current
            Volts.per(Second).of(currentRampRate.baseUnitMagnitude()),
            Volts.of(stepCurrent.baseUnitMagnitude()),
            timeout,
            state -> SignalLogger.writeString("state", state.toString()));
        var mech = new SysIdRoutine.Mechanism(
            (voltageMeasure) -> {
                m_left.setControl(m_current.withOutput(voltageMeasure.in(Volts)));
                m_right.setControl(m_current.withOutput(voltageMeasure.in(Volts)));
            }, null, this);

        return new SysIdRoutine(cfg, mech);
    }

    public Shooter() {
        m_right.getConfigurator().apply(ShooterConfigs.kRightConfigs);
        m_left.getConfigurator().apply(ShooterConfigs.kLeftConfigs);
        m_left.setInverted(true);
    }

    public Command stop() {
        return runOnce(() -> {
            m_right.setControl(m_coast);
            m_left.setControl(m_coast);
        });
    }

    private Command toVelo(Supplier<Measure<Velocity<Angle>>> velo) {
        return runEnd(
            () -> {
                var velMeas = velo.get();
                // m_rightTarget = velMeas.in(Rotations.per(Minute)) * m_spinAmt;
                m_leftTarget = velMeas.in(Rotations.per(Minute));
                // var right = Rotations.per(Minute).of(m_rightTarget).in(RotationsPerSecond);
                // var left = Rotations.per(Minute).of(m_leftTarget).in(RotationsPerSecond);
                m_right.setControl(m_request.withVelocity(velMeas.in(RotationsPerSecond) * kSpinAmt));
                m_left.setControl(m_request.withVelocity(velMeas.in(RotationsPerSecond)));

            }, () -> {
                // m_rightTarget = 0;
                m_leftTarget = 0;
                m_right.setControl(m_coast);
                m_left.setControl(m_coast);
            });
    }

    public Command increaseRpm() {
        return Commands.runOnce(
            () -> {
                m_leftTarget += 100;
            });
    }

    public Command decreaseRpm() {
        return Commands.runOnce(
            () -> {
                m_leftTarget -= 100;
            });
    }

    public Command shoot() {
        return toVelo(() -> Rotations.per(Minute).of(7300));
    }

    // That's not really that fast.
    public Command shootFast() {
        return toVelo(() -> Rotations.per(Minute).of(6000));
    }

    public Command ampShot() {
        // TODO figure out the angle that the arm was at
        return toVelo(() -> Rotations.per(Minute).of(1250)); // TODO make this a constant
    }

    public Command trapShot() {
        // It Doesn't Work Very Well. (2 out of like 50 times!!!)
        return toVelo(() -> Rotations.per(Minute).of(6000)); // TODO make this a constant
    }

    public Command spinUp() {
        return toVelo(m_leftTargetSupp)
            .until(() -> spinUpFinished());
    }

    public boolean spinUpFinished() {
        if (m_leftTarget == 0) {
            return false;
        }
        var left = m_leftTargetSupp.get().in(RotationsPerSecond);
        var right = m_leftTargetSupp.get().in(RotationsPerSecond) * kSpinAmt;
        var tolerance = 2;
        m_leftOk = MathUtil.isNear(
            left, m_left.getVelocity().getValueAsDouble(), tolerance);
        m_rightOk = MathUtil.isNear(
            right, m_right.getVelocity().getValueAsDouble(), tolerance);
        m_spunUp = m_leftOk && m_rightOk;
        return m_leftOk && m_rightOk;
    }

    private void rawRun(double dutyCycle) {
        m_left.set(dutyCycle * m_spinAmt);
        m_right.set(dutyCycle);
    }

    public Command run() {
        return runEnd(
            () -> {
                rawRun(0.25);
            }, () -> {
                m_left.set(0);
                m_right.set(0);
            });
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

    // idk i just wrote a couple methods cuz they might be able to be used but i'm
    // prob gonna have to redo everything anyway ^-^ tehe

    public Pose2d getAdjustedPose(Supplier<Pose3d> robotPose, DoubleSupplier x, DoubleSupplier y,
        DoubleSupplier omega) {
        var xPower = MathUtil.applyDeadband(x.getAsDouble(), kStickDeadband);
        var yPower = MathUtil.applyDeadband(y.getAsDouble(), kStickDeadband);
        var omegaPower = MathUtil.applyDeadband(omega.getAsDouble(), kStickDeadband);
        var futureXVelo = xPower * kMaxSpeed;
        var futureYVelo = yPower * kMaxSpeed;
        var futureOmegaVelo = omegaPower * kMaxAngularRate;
        var curPose = robotPose.get().toPose2d();
        var adjustedPose = curPose.plus(new Transform2d(futureXVelo * m_shotTime, futureYVelo * m_shotTime,
            Rotation2d.fromRadians(futureOmegaVelo * m_shotTime)));
        return adjustedPose;
    }

    public void setTargetRpmWhileMoving(Supplier<Pose3d> robotPose, Supplier<ChassisSpeeds> chassisSpeeds) {
        var pose = robotPose.get().getTranslation();
        var speakerToRobotAngle = pose.minus(speakerPose).toTranslation2d().getAngle();
        var speeds = chassisSpeeds.get();
        var linearVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        var tangentialVelocity = linearVelocity.rotateBy(speakerToRobotAngle.unaryMinus());
        var radialComponent = tangentialVelocity.getX();
        var tangentialComponent = tangentialVelocity.getY();
        var rawDist = pose.getDistance(speakerPose);
        var realShotTime = m_shotTime * rawDist;
        // sorry it was less typing
        var shotSpeed = MathUtil.clamp(rawDist / realShotTime + radialComponent, 0, Integer.MAX_VALUE);
        var effectiveDist = Math.hypot(tangentialComponent, shotSpeed);
        m_leftTarget = effectiveDist * kRpmFactor;
    }

    public void periodic() {
        // m_leftTarget = m_tunableRpm.get();
        // m_rightTarget = m_leftTarget * m_spinAmt;
        // m_spinAmt = m_tunableSpin.get();
        // m_shotTime = m_tunableShotTime.get();

        log_targetRpm.accept(m_leftTarget);
        log_spinAmt.accept(m_spinAmt);
        log_shotTime.accept(m_shotTime);
        log_leftTarget.accept(m_left.getClosedLoopReference().getValueAsDouble());
        log_rightTarget.accept(m_right.getClosedLoopReference().getValueAsDouble());
        log_spunUp.accept(m_spunUp);
        log_leftOk.accept(m_leftOk);
        log_rightOk.accept(m_rightOk);
    }

    public void simulationPeriodic() {
        m_right.getSimState().setSupplyVoltage(12);
        var volts = m_right.getSimState().getMotorVoltage();
        // TODO: check voltage
        m_flywheelSim.setInputVoltage(volts);

        if (m_flywheelSim.getAngularVelocityRPM() >= m_leftTarget && !found) {
            System.out.println("found it at " + time + " seconds");
            found = true;
        }

        time += kSimInterval;
        m_flywheelSim.update(kSimInterval);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_currentSysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_currentSysId.dynamic(direction);
    }
}
