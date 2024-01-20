package frc.robot.subsystems.trap;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Constants.TrapK.TrapShootK.*;

public class TrapShoot extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(kMotorCANID, MotorType.kBrushless);
    private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(kAbsoluteEncoderPort);

    // TODO: add a constructor
    public TrapShoot() {

    }
    
    /**
     * @return a command to outtake the note for one second
     */
    public Command trapShoot() {
        return Commands.run(
            () -> m_motor.set(1)
        ).withTimeout(1);
    }
}
