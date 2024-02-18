package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterK.kLeftId;
import static frc.robot.Constants.ShooterK.kRightId;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.ShooterK.FlywheelSimK.*;
import static frc.robot.Constants.kStickDeadband;
import static frc.robot.Robot.speakerPose;
import static frc.robot.Robot.maxAngularRate;
import static frc.robot.Robot.maxSpeed;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
    private final TalonFX m_left = new TalonFX(kLeftId);
    private final TalonFX m_right = new TalonFX(kRightId);

    private double m_targetRpm;

    private double time = 0;
    private boolean found = false;
    private final FlywheelSim m_flywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), kGearRatio, kMoi);

    private double shotTime;

    public Shooter() {
        SmartDashboard.putNumber("shotTime", 1.5);
        shotTime = SmartDashboard.getNumber("shotTime", 1.5);
        m_targetRpm = 100;
    }

    public Command shoot() {
        return run(() -> {
            m_right.set(1);
            m_left.set(1);
        });
    }

    public Command shoot(double speed) {
        return run(() -> {
            m_right.set(speed);
            m_left.set(speed);
        });
    }

    public Command spinUp() {
        return run(() -> {
            m_right.set(1);
            m_left.set(1);
        }).until(() -> m_flywheelSim.getAngularVelocityRPM() >= m_targetRpm); // TODO make work outside of simulation
    }

    public Command slowShot() {
        return run(() -> {
            m_right.set(0.5);
            m_left.set(0.5);
        });
    }

    // idk i just wrote a couple methods cuz they might be able to be used but i'm
    // prob gonna have to redo everything anyway

    public Pose2d getAdjustedPose(Swerve swerve, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        var xPower = MathUtil.applyDeadband(x.getAsDouble(), kStickDeadband);
        var yPower = MathUtil.applyDeadband(y.getAsDouble(), kStickDeadband);
        var omegaPower = MathUtil.applyDeadband(omega.getAsDouble(), kStickDeadband);
        var futureXVelo = xPower * maxSpeed;
        var futureYVelo = yPower * maxSpeed;
        var futureOmegaVelo = omegaPower * maxAngularRate;
        var curPose = swerve.getPose3d().toPose2d();
        var adjustedPose = curPose.plus(new Transform2d(futureXVelo * shotTime, futureYVelo * shotTime,
            Rotation2d.fromRadians(futureOmegaVelo * shotTime)));
        return adjustedPose;
    }

    public void setTargetRpmWhileMoving(Swerve swerve) {
        var robotPose = swerve.getPose3d().getTranslation();
        var speakerToRobotAngle = robotPose.minus(speakerPose).toTranslation2d().getAngle();
        var speeds = swerve.getState().speeds;
        var linearVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        var tangentialVelocity = linearVelocity.rotateBy(speakerToRobotAngle.unaryMinus());
        var radialComponent = tangentialVelocity.getX();
        var tangentialComponent = tangentialVelocity.getY();
        var rawDist = robotPose.getDistance(speakerPose);
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

        time += kInterval;
        m_flywheelSim.update(kInterval);
    }
}
