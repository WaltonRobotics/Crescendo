package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ConveyorK.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
    private final CANSparkMax m_conveyor = new CANSparkMax(kConveyorId, MotorType.kBrushless);

    public Command convey() {
        return run(() -> {
            m_conveyor.set(0.5);
        });
    }
}
