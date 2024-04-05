package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.BooleanLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

import static frc.robot.Constants.ClimberK.*;

import java.util.function.BooleanSupplier;

public class Climber extends SubsystemBase {
    private final CANSparkMax m_right = new CANSparkMax(kRightId, MotorType.kBrushless); 
    private final CANSparkMax m_left = new CANSparkMax(kLeftId, MotorType.kBrushless);

    private final DoubleLogger log_leftPos = WaltLogger.logDouble("Climber", "leftPos");
    private final DoubleLogger log_rightPos = WaltLogger.logDouble("Climber", "rightPos");

    private final BooleanLogger log_leftAtLimit = WaltLogger.logBoolean("Climber", "leftAtLimit");
    private final BooleanLogger log_rightAtLimit = WaltLogger.logBoolean("Climber", "rightAtLimit");

    public Climber() {
        m_right.setInverted(false);
        m_left.setInverted(true);

        m_right.setIdleMode(IdleMode.kBrake);
        m_left.setIdleMode(IdleMode.kBrake);

        m_right.getEncoder().setPosition(0);
        m_left.getEncoder().setPosition(0);
    }

    public Command retractBoth(BooleanSupplier overrideSup) {
        return runEnd(() -> {
            boolean override = overrideSup.getAsBoolean();
            boolean rightAtMax = m_right.getEncoder().getPosition() >= 0 && !override;
            boolean leftAtMax = m_left.getEncoder().getPosition() >= 0 && !override;
            m_right.set(rightAtMax ? 0 : kRetractDutyCycle);
            m_left.set(leftAtMax ? 0 : kRetractDutyCycle);
        }, 
        () -> {
            m_right.set(0);
            m_left.set(0);
        });
    }

    public Command retractLeft(BooleanSupplier overrideSup) {
        return runEnd(() -> {
            boolean override = overrideSup.getAsBoolean();
            boolean atMax = m_left.getEncoder().getPosition() >= 0 && !override;
            m_left.set(atMax ? 0 : kRetractSingleDutyCycle);
        }, 
        () -> {
            m_left.set(0);
        });
    }

    public Command retractRight(BooleanSupplier overrideSup) {
        return runEnd(() -> {
            boolean override = overrideSup.getAsBoolean();
            boolean atMax = m_right.getEncoder().getPosition() >= 0 && !override;
            m_right.set(atMax ? 0 : kRetractSingleDutyCycle);
        }, 
        () -> {
            m_right.set(0);
        });
    }

    public Command extendBoth(BooleanSupplier overrideSup) {
        return runEnd(() -> {
            // max is negative, therefore >
            boolean override = overrideSup.getAsBoolean();
            boolean rightAtMax = m_right.getEncoder().getPosition() <= kMaxExtensionPos && !override;
            boolean leftAtMax = m_left.getEncoder().getPosition() <= kMaxExtensionPos && !override;
            m_right.set(rightAtMax ? 0 : kExtendDutyCycle);
            m_left.set(leftAtMax ? 0 : kExtendDutyCycle);
            log_rightAtLimit.accept(rightAtMax);
            log_leftAtLimit.accept(leftAtMax);
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
