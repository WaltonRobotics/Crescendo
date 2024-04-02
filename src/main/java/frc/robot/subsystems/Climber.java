package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberK.*;

public class Climber extends SubsystemBase {
    private final CANSparkMax m_right = new CANSparkMax(kRightId, MotorType.kBrushless); 
    private final CANSparkMax m_left = new CANSparkMax(kLeftId, MotorType.kBrushless);

    public Climber() {
        m_right.setInverted(false);
        m_left.setInverted(true);

        m_right.setIdleMode(IdleMode.kBrake);
        m_left.setIdleMode(IdleMode.kBrake);
    }
    
    public Command climb() {
        return runEnd(() -> {
            m_right.set(0.5);
            m_left.set(0.5);
        }, 
        () -> {
            m_right.set(0);
            m_left.set(0);
        });
    }

    public Command moveLeft() {
        return runEnd(() -> {
            m_left.set(0.5);
        }, 
        () -> {
            m_left.set(0);
        });
    }

    public Command reverseLeft() {
        return runEnd(() -> {
            m_left.set(-0.5);
        }, 
        () -> {
            m_left.set(0);
        });
    }

    public Command moveRight() {
        return runEnd(() -> {
            m_right.set(0.5);
        }, 
        () -> {
            m_right.set(0);
        });
    }

    public Command reverseRight() {
        return runEnd(() -> {
            m_right.set(-0.5);
        }, 
        () -> {
            m_right.set(0);
        });
    }

    public Command release() {
        return runEnd(() -> {
            m_right.set(-0.5);
            m_left.set(-0.5);
        }, 
        () -> {
            m_right.set(0);
            m_left.set(0);
        });
    }
}  
