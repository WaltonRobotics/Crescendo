package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ConveyorK.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.BooleanLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

public class Conveyor extends SubsystemBase {
    private final CANSparkMax m_conveyor = new CANSparkMax(kConveyorId, MotorType.kBrushless);

    private final DoubleLogger log_statorCurrent = WaltLogger.logDouble("Conveyor", "statorCurrent");
    private final DoubleLogger log_outputVoltage = WaltLogger.logDouble("Conveyor", "outputVoltage");
    private final DoubleLogger log_supplyVoltage = WaltLogger.logDouble("Conveyor", "supplyVoltage");

    public final Trigger trg_currentSpike = new Trigger(() -> {
        return m_conveyor.getOutputCurrent() >= 15;
    }).debounce(0.085);

    private final BooleanLogger log_currentSpike = WaltLogger.logBoolean("Conveyor", "currentSpike", PubSubOption.sendAll(true));
    
    public Command runFast() {
        var go = runEnd(() -> {
            m_conveyor.set(1);
        }, () -> {
            m_conveyor.set(0);
        });

        return go.withName("ConveyorRunFast");
    }

    public Command retract() {
        return runEnd(() -> m_conveyor.set(-0.2), () -> m_conveyor.set(0)).withName("ConveyorRetract");
    }

    public Command start() {
        return run(() -> m_conveyor.set(0.4)).withName("ConveyorRun");
    }

    public Command startSlower() {
        return run(() -> m_conveyor.set(0.2)).withName("ConveyorRunSlow");
    }

    public Command fullPower() {
        return run(() -> m_conveyor.set(1));
    }

    public Command runBackwards() {
        var go = runEnd(() -> {
            m_conveyor.set(-0.25);
        }, () -> {
            m_conveyor.set(0);
        });

        return go;
    }

    public Command stop() {
        return run(() -> m_conveyor.set(0)).withName("ConveyorStop");
    }

    @Override
    public void periodic() {
        log_statorCurrent.accept(m_conveyor.getOutputCurrent());
        log_outputVoltage.accept(m_conveyor.getAppliedOutput());
        log_supplyVoltage.accept(m_conveyor.getBusVoltage());
        log_currentSpike.accept(trg_currentSpike);
    }
}
