package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ConveyorK.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.BooleanLogger;

public class Conveyor extends SubsystemBase {
    private final CANSparkMax m_conveyor = new CANSparkMax(kConveyorId, MotorType.kBrushless);
    private final DigitalInput m_beamBreak = new DigitalInput(0);

    private final Trigger m_note = new Trigger(() -> m_beamBreak.get()).negate();

    private final BooleanLogger log_beamBreak = WaltLogger.logBoolean(kDbTabName, "beamBreak");

    public Command run(boolean ignoreSensor) {
        var go = runEnd(() -> {
            m_conveyor.set(0.35);
        }, () -> {
            m_conveyor.set(0);
        });

        if (!ignoreSensor) {
            go = go.until(m_note);
        }

        return go;
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
        return run(() -> m_conveyor.set(0));
    }

    public void periodic() {
        log_beamBreak.accept(m_beamBreak.get());
    }
}
