package frc.robot.subsystems.shooter;

import static frc.robot.Constants.RobotK.kSimInterval;
import static frc.robot.Constants.ShooterK.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.util.LoggedTunableNumber;

import static frc.robot.Constants.ShooterK.FlywheelSimK.*;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.kCanbus;
import static frc.robot.Constants.kStickDeadband;
import static frc.robot.Robot.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final TalonFX m_left = new TalonFX(kLeftId, kCanbus);
    private final TalonFX m_right = new TalonFX(kRightId, kCanbus);
    private final VelocityVoltage m_request = new VelocityVoltage(0);
    private final VoltageOut m_voltage = new VoltageOut(0);

    private LoggedTunableNumber m_tunableRpm = new LoggedTunableNumber("targetRpm", 3000);
    private LoggedTunableNumber m_tunableSpin = new LoggedTunableNumber("spin", kSpinAmt);
    private LoggedTunableNumber m_tunableShotTime = new LoggedTunableNumber("shotTime", 1.5);

    private double m_targetRpm;
    private double m_spinAmt;
    private double m_shotTime;

    private double time = 0;
    private boolean found = false;
    private final FlywheelSim m_flywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), kGearRatio, kMoi);

    private Measure<Velocity<Angle>> m_targetVelo = Rotations.per(Minute).of(0);

    private final SysIdRoutine m_sysId = new SysIdRoutine(
        new SysIdRoutine.Config(null,
            Volts.of(7),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
            m_left.setControl(m_voltage.withOutput(volts.in(Volts)));
            m_right.setControl(m_voltage.withOutput(volts.in(Volts)));
        }, null, this));

    public Shooter() {
        m_targetRpm = 3000;
        m_spinAmt = kSpinAmt;
        m_shotTime = 1.5;

        TalonFXConfiguration rightConfigs = new ShooterConfigs().kRightConfigs;
        TalonFXConfiguration leftConfigs = new ShooterConfigs().kLeftConfigs;
        m_right.getConfigurator().apply(rightConfigs);
        m_left.getConfigurator().apply(leftConfigs);

        m_left.setInverted(true);
    }

    public Command stop() {
        return toVelo(RotationsPerSecond.of(0));
    }

    private Command toVelo(Measure<Velocity<Angle>> velo) {
        var setTargetCmd = runOnce(() -> {
            m_targetVelo = velo;
        });
        var toTargetCmd = run(() -> {
            m_right.setControl(m_request.withVelocity(m_targetVelo.in(RotationsPerSecond) * kSpinAmt));
            m_left.setControl(m_request.withVelocity(m_targetVelo.in(RotationsPerSecond)));
        });
        return setTargetCmd.andThen(toTargetCmd);
    }

    public Command shoot() {
        // TODO make the LoggedTunableNumber thing work
        return toVelo(Rotations.per(Minute).of(m_targetRpm));
    }

    public Command trapShot() {
        // It Doesn't Work Very Well. (2 out of like 50 times!!!)
        return toVelo(Rotations.per(Minute).of(6000)); // TODO make this a constant
    }

    public Command spinUp() {
        return toVelo(Rotations.per(Minute).of(m_targetRpm))
            .until(() -> spinUpFinished());
    }

    public boolean spinUpFinished() {
        return m_left.getVelocity().getValueAsDouble() * 60 == m_targetRpm;
    }

    private void rawRun(double dutyCycle) {
        m_left.set(dutyCycle * m_spinAmt);
        m_right.set(dutyCycle);
    }

    public Command run() {
        return runEnd(() -> {
            rawRun(0.25);
        },
            () -> {
                m_left.set(0);
                m_right.set(0);
            });
    }

    // idk i just wrote a couple methods cuz they might be able to be used but i'm
    // prob gonna have to redo everything anyway

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
        m_targetRpm = effectiveDist * kRpmFactor;
    }

    public void periodic() {
        m_targetRpm = m_tunableRpm.get();
        m_spinAmt = m_tunableSpin.get();
        m_shotTime = m_tunableShotTime.get();
    }

    public void simulationPeriodic() {
        m_right.getSimState().setSupplyVoltage(12);
        var volts = m_right.getSimState().getMotorVoltage();
        // TODO: check voltage
        m_flywheelSim.setInputVoltage(volts);

        if (m_flywheelSim.getAngularVelocityRPM() >= m_targetRpm && !found) {
            System.out.println("found it at " + time + " seconds");
            found = true;
        }

        time += kSimInterval;
        m_flywheelSim.update(kSimInterval);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysId.dynamic(direction);
    }
}
