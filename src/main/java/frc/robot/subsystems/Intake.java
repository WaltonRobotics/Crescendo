package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeK.*;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(kIntakeId, MotorType.kBrushless);
    private final CANSparkMax m_feeder = new CANSparkMax(kFeederId, MotorType.kBrushless);

    public Intake() {
        m_feeder.follow(m_motor);
    }

    public Command intake() {
        return Commands.run(
            () -> {
                m_motor.set(-1);
            });
    }
}
