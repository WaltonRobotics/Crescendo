package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeK.*;

public class Intake extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(kIntakeId);
    private final CANSparkMax m_feeder = new CANSparkMax(kFeederId,
        MotorType.kBrushless);

    private final VoltageOut m_voltsFoc = new VoltageOut(0).withEnableFOC(true);

    public Intake() {
        m_motor.getConfigurator().apply(IntakeConfigs.kConfigs);
    }

    private void runMainRollers(double volts) {
        m_motor.setControl(m_voltsFoc.withOutput(volts));
    }

    public Command outtake() {
        return runEnd(
            () -> {
                runMainRollers(-12);
                m_feeder.set(0.5);
            },
            () -> {
                runMainRollers(0);
                m_feeder.set(0);
            });
    }

    public Command run() {
        return runEnd(
            () -> {
                runMainRollers(12);
                m_feeder.set(-0.5);
            },
            () -> {
                runMainRollers(0);
                m_feeder.set(0);
            });
    }

    public Command start() {
        return run(() -> {
            runMainRollers(12);
            m_feeder.set(-0.5);
        });
    }

    public Command stop() {
        return run(() -> {
            runMainRollers(0);
            m_feeder.set(0);
        });
    }
}
