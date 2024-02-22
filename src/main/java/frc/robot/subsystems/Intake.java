package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.IntakeK.*;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(kIntakeId, MotorType.kBrushless);
    private final CANSparkMax m_feeder = new CANSparkMax(kFeederId,
        MotorType.kBrushless);

    private final Trigger m_sightTrigger;

    public Intake(DigitalInput visiSight) {
        m_feeder.follow(m_motor, true);
        m_sightTrigger = new Trigger(visiSight::get);
        m_sightTrigger.whileTrue(outtake()); // FIXME
    }

    public Command outtake() {
        return runEnd(
            () -> {
                m_motor.set(-1);
            },
            () -> {
                m_motor.set(0);
            });
    }

    public Command run() {
        return runEnd(
            () -> {
                m_motor.set(1);
            },
            () -> {
                m_motor.set(0);
            });
    }
}
