package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

import static frc.robot.Constants.ClimberK.*;

public class Climber extends SubsystemBase {
    private final CANSparkMax m_right = new CANSparkMax(kRightId, MotorType.kBrushless); 
    private final CANSparkMax m_left = new CANSparkMax(kLeftId, MotorType.kBrushless);

    private final DoubleLogger log_leftPos = WaltLogger.logDouble("Climber", "leftPos");
    private final DoubleLogger log_rightPos = WaltLogger.logDouble("Climber", "rightPos");

    public Climber() {
        m_right.setInverted(false);
        m_left.setInverted(true);

        m_right.setIdleMode(IdleMode.kBrake);
        m_left.setIdleMode(IdleMode.kBrake);
    }
  
    public Command retractBoth() {
        return runEnd(() -> {
            m_right.set(kRetractDutyCycle);
            m_left.set(kRetractDutyCycle);
        }, 
        () -> {
            m_right.set(0);
            m_left.set(0);
        });
    }

    public Command retractLeft() {
        return runEnd(() -> {
            m_left.set(kRetractSingleDutyCycle);
        }, 
        () -> {
            m_left.set(0);
        });
    }

    public Command retractRight() {
        return runEnd(() -> {
            m_right.set(kRetractSingleDutyCycle);
        }, 
        () -> {
            m_right.set(0);
        });
    }

    public Command extendBoth() {
        return runEnd(() -> {
            // max is negative, therefore >
            boolean rightAtMax = m_right.getEncoder().getPosition() > kMaxExtensionPos;
            boolean leftAtMax = m_left.getEncoder().getPosition() > kMaxExtensionPos;
            m_right.set(rightAtMax ? 0 : kExtendDutyCycle);
            m_left.set(leftAtMax ? 0 : kExtendDutyCycle);
        }, 
        () -> {
            m_right.set(0);
            m_left.set(0);
        });
    }

    public Command extendBothOverride() {
        return runEnd(() -> {
            m_right.set(kExtendDutyCycle);
            m_left.set(kExtendDutyCycle);
        }, 
        () -> {
            m_right.set(0);
            m_left.set(0);
        });
    }

    @Override
    public void periodic() {
        log_leftPos.accept(m_left.getEncoder().getPosition());
        log_rightPos.accept(m_right.getEncoder().getPosition());
    }
}  
