package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterK.kLeftId;
import static frc.robot.Constants.ShooterK.kRightId;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterK.FlywheelSimK.*;

public class Shooter extends SubsystemBase {
    private final TalonFX m_left = new TalonFX(kLeftId);
    private final TalonFX m_right = new TalonFX(kRightId);

    private double time = 0;
    private boolean found = false;
    private final FlywheelSim m_flywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), kGearRatio, kMoi);

    public Command shoot() {
        return run(() -> {
            m_right.set(1);
            m_left.set(1);
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
