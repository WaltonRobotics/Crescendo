package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.IntakeK.*;

public class Intake extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(kIntakeId);
    private final CANSparkMax m_feeder = new CANSparkMax(kFeederId,
        MotorType.kBrushless);

    public final Trigger m_sightTrigger;

    public Intake(DigitalInput visiSight) {
        m_sightTrigger = new Trigger(visiSight::get);
    }

    public Command outtake() {
        return runEnd(
            () -> {
                m_motor.set(-1);
                m_feeder.set(-1);
            },
            () -> {
                m_motor.set(0);
                m_feeder.set(0);
            });
    }

    public Command run() {
        return runEnd(
            () -> {
                m_motor.set(1);
                m_feeder.set(1);
            },
            () -> {
                m_motor.set(0);
                m_feeder.set(0);
            });
    }
}
