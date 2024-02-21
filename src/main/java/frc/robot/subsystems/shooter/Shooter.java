package frc.robot.subsystems.shooter;

import static frc.robot.Constants.RobotK.kSimInterval;
import static frc.robot.Constants.ShooterK.*;

import com.ctre.phoenix6.controls.VelocityVoltage;
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
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CtreConfigs;

import static frc.robot.Constants.ShooterK.FlywheelSimK.*;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.kCanbus;
import static frc.robot.Constants.kStickDeadband;
import static frc.robot.Robot.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final TalonFX m_left = new TalonFX(kLeftId, kCanbus);
    private final TalonFX m_right = new TalonFX(kRightId, kCanbus);
    private final VelocityVoltage m_request = new VelocityVoltage(0);

    // beam break on port 0

    private double m_targetRpm;

    private double time = 0;
    private boolean found = false;
    private final FlywheelSim m_flywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), kGearRatio, kMoi);

    private Measure<Velocity<Angle>> m_targetVelo = Rotations.per(Minute).of(0);

    private double shotTime;

    public Shooter() {
        SmartDashboard.putNumber("shotTime", 1.5);
        shotTime = SmartDashboard.getNumber("shotTime", 1.5);
        m_targetRpm = 1000;

        CtreConfigs configs = CtreConfigs.get();
        m_left.getConfigurator().apply(configs.m_leftShooterConfigs);
        m_right.getConfigurator().apply(configs.m_leftShooterConfigs);

        m_left.setInverted(true);
    }

    // TODO check the numbers for all of this they're just dummy values for now
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
        return toVelo(Rotations.per(Minute).of(1000));
    }

    public Command spinUp() {
        // TODO make work outside of simulation
        return toVelo(Rotations.per(Minute).of(1000))
            .until(() -> m_flywheelSim.getAngularVelocityRPM() >= m_targetRpm);
    }

    public Command slowShot() {
        return toVelo(Rotations.per(Minute).of(500));
    }

    private void rawRun(double dutyCycle) {
        m_left.set(dutyCycle * 0.8);
        m_right.set(dutyCycle);
    }

    public Command runMotors() {
        return runEnd(() -> {
            rawRun(0.5);
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
        var futureXVelo = xPower * maxSpeed;
        var futureYVelo = yPower * maxSpeed;
        var futureOmegaVelo = omegaPower * maxAngularRate;
        var curPose = robotPose.get().toPose2d();
        var adjustedPose = curPose.plus(new Transform2d(futureXVelo * shotTime, futureYVelo * shotTime,
            Rotation2d.fromRadians(futureOmegaVelo * shotTime)));
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
        var realShotTime = shotTime * rawDist;
        // sorry it was less typing
        var shotSpeed = MathUtil.clamp(rawDist / realShotTime + radialComponent, 0, Integer.MAX_VALUE);
        var effectiveDist = Math.hypot(tangentialComponent, shotSpeed);
        m_targetRpm = effectiveDist * kRpmFactor;
    }

    public void periodic() {
        // will change the value when testing
        shotTime = SmartDashboard.getNumber("shotTime", 1.5);

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
}
