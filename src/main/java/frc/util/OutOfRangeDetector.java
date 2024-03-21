package frc.util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OutOfRangeDetector {
    private Trigger trg_outOfRange;
    BooleanSupplier outOfRange;
    
    public OutOfRangeDetector() {
        trg_outOfRange = new Trigger(outOfRange);
    }

    public void detectOutOfRange(DoubleSupplier val, double min, double max, String valName, double debounceSecs) {
        outOfRange = () -> val.getAsDouble() < min || val.getAsDouble() > max;
        trg_outOfRange.debounce(debounceSecs)
            .onTrue(
                Commands.print(valName + " (" + val.getAsDouble() + ") is out of range D:"));
    }
}
