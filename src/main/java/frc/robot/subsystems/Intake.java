package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

import static frc.robot.Constants.IntakeK.*;

public class Intake extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(kIntakeId);
    private final CANSparkMax m_feeder = new CANSparkMax(kFeederId,
        MotorType.kBrushless);

    private final VoltageOut m_voltsFoc = new VoltageOut(0).withEnableFOC(true);

    private final DoubleLogger log_statorCurrent = WaltLogger.logDouble("Intake", "statorCurrent");
    private final DoubleLogger log_supplyCurrent = WaltLogger.logDouble("Intake", "supplyCurrent");
    private final DoubleLogger log_outputVoltage = WaltLogger.logDouble("Intake", "outputVoltage");
    private final DoubleLogger log_supplyVoltage = WaltLogger.logDouble("Intake", "supplyVoltage");

    private final DoubleLogger log_middleStatorCurrent = WaltLogger.logDouble("Intake/MiddleRoller", "statorCurrent");
    private final DoubleLogger log_middleOutputVoltage = WaltLogger.logDouble("Intake/MiddleRoller", "outputVoltage");
    private final DoubleLogger log_middleSupplyVoltage = WaltLogger.logDouble("Intake/MiddleRoller", "supplyVoltage");

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
            }).withName("IntakeRun");
    }

    public Command start() {
        return run(() -> {
            runMainRollers(12);
            m_feeder.set(-0.5);
        }).withName("IntakeStart");
    }

    public Command stop() {
        return run(() -> {
            runMainRollers(0);
            m_feeder.set(0);
        }).withName("IntakeStop");
    }

    @Override
    public void periodic() {
        log_statorCurrent.accept(m_motor.getStatorCurrent().getValueAsDouble());
        log_supplyCurrent.accept(m_motor.getSupplyCurrent().getValueAsDouble());
        log_outputVoltage.accept(m_motor.getMotorVoltage().getValueAsDouble());
        log_supplyVoltage.accept(m_motor.getSupplyVoltage().getValueAsDouble());

        log_middleStatorCurrent.accept(m_feeder.getOutputCurrent());
        log_middleOutputVoltage.accept(m_feeder.getAppliedOutput());
        log_middleSupplyVoltage.accept(m_feeder.getBusVoltage());
    }
}
