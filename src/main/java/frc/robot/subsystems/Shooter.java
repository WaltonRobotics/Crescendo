package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public TalonFX m_motor = new TalonFX(11);

    /**
     * this is just a basic shooter thing it will be changed later
     */
    public Command shoot() {
        return Commands.run(
                () -> m_motor.set(1),
                this).withTimeout(1);
    }
}
