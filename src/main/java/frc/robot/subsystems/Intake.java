package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public TalonFX m_motor = new TalonFX(10);

    /**
     * this is just a basic intake thing it will be changed later
     */
    public Command intake() {
        return Commands.run(
            () -> m_motor.set(-1),
            this).withTimeout(1);
    }
}
