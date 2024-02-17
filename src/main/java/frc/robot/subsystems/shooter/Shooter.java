package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterK.kLeftId;
import static frc.robot.Constants.ShooterK.kRightId;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.ShooterK.FlywheelSimK.*;
import static frc.robot.Robot.*;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final TalonFX m_left = new TalonFX(kLeftId);
    private final TalonFX m_right = new TalonFX(kRightId);

    private double time = 0;
    private boolean found = false;
    private final FlywheelSim m_flywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), kGearRatio, kMoi);

    private final Supplier<Pose3d> m_robotPoseSupplier;

    public Shooter(Supplier<Pose3d> robotPoseSupplier) {
        m_robotPoseSupplier = robotPoseSupplier;
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
        }).until(() -> m_flywheelSim.getAngularVelocityRPM() >= kTargetRpm); // TODO make work outside of simulation
    }

    public Command slowShot() {
        return run(() -> {
            m_right.set(0.5);
            m_left.set(0.5);
        });
    }

    // TODO figure out What To Do with this
    public double getEffectiveDist(Swerve swerve) {
        var currentPose = m_robotPoseSupplier.get();
        var curTranslation = currentPose.getTranslation();
        var speakerToRobotAngle = curTranslation.minus(speakerPose).toTranslation2d().getAngle();
        var linearFieldVelocity = new Translation2d(swerve.getState().speeds.vxMetersPerSecond,
            swerve.getState().speeds.vyMetersPerSecond)
                .rotateBy(swerve.getPose3d().getRotation().toRotation2d());
        var tangentialVelocity = linearFieldVelocity.rotateBy(speakerToRobotAngle.unaryMinus());

        double radialComponent = tangentialVelocity.getX();
        double tangentialComponent = tangentialVelocity.getY();
        double shotTime = 1.05;
        double rawDistToGoal = curTranslation.getDistance(speakerPose);
        double shotSpeed = rawDistToGoal / shotTime + radialComponent;
        shotSpeed = MathUtil.clamp(shotSpeed, 0, shotSpeed);
        Rotation2d goalHeading = new Pose2d(currentPose.toPose2d().getTranslation().unaryMinus(),
            currentPose.toPose2d().getRotation().unaryMinus())
                .transformBy(new Transform2d(speakerPose.toTranslation2d(), new Rotation2d()))
                .getTranslation()
                .getAngle();
        goalHeading = goalHeading.plus(new Rotation2d(shotSpeed, tangentialComponent));

        double effectiveDist = shotTime * Math.hypot(tangentialComponent, shotSpeed);

        return effectiveDist;
    }

    public void simulationPeriodic() {
        m_right.getSimState().setSupplyVoltage(12);
        var volts = m_right.getSimState().getMotorVoltage();
        // TODO: check voltage
        m_flywheelSim.setInputVoltage(volts);

        if (m_flywheelSim.getAngularVelocityRPM() >= kTargetRpm && !found) {
            System.out.println("found it at " + time + " seconds");
            found = true;
        }

        time += kInterval;
        m_flywheelSim.update(kInterval);
    }
}
