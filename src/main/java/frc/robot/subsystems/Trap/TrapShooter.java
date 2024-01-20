package frc.robot.subsystems.trap;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Constants.TrapK.TrapShooterK.*;

public class TrapShooter extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(kMotorCanId, MotorType.kBrushless);

    // TODO: add a constructor
    public TrapShooter() {

    }

    /**
     * @return a command to outtake the note for one second
     */
    public Command trapShoot() {
        return Commands.run(
            () -> m_motor.set(1)).withTimeout(1);
    }
}
