package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterK.kLeftId;
import static frc.robot.Constants.ShooterK.kRightId;
import static frc.robot.Constants.ShooterK.FlywheelSimK.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX m_left = new TalonFX(kLeftId);
    private final TalonFX m_right = new TalonFX(kRightId);

    // things for simulation
    // TODO: check motor number
    private final FlywheelSim m_flywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), kGearRatio, kMOI);

    public Command shoot() {
        return run(() -> {
            m_right.set(1);
            m_left.set(1);
        });
    }

    public Command slowShot() {
        return run(() -> {
            m_right.set(0.5);
            m_left.set(0.5);
        });
    }
}
