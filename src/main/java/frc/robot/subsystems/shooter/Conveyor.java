package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ConveyorK.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
    private final CANSparkMax m_conveyor = new CANSparkMax(kConveyorId, MotorType.kBrushless);

    public Command runFast() {
        var go = runEnd(() -> {
            m_conveyor.set(1);
        }, () -> {
            m_conveyor.set(0);
        });

        return go;
    }

    public Command retract() {
        return runEnd(() -> m_conveyor.set(-0.1), () -> m_conveyor.set(0));
    }

    public Command startSlow() {
        return run(() -> m_conveyor.set(0.35));
    }

    public Command runBackwards() {
        var go = runEnd(() -> {
            m_conveyor.set(-0.25);
        }, () -> {
            m_conveyor.set(0);
        });

        return go;
    }

    public Command stop() {
        return run(() -> m_conveyor.set(0));
    }
}
