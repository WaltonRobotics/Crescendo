package frc.util;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.logging.WaltLogger.BooleanLogger;

public class WaltRangeChecker {
    private WaltRangeChecker() {}

    public static void addDoubleChecker(String name, DoubleSupplier val, double min, double max, double secondsToFail, boolean eqTo) {
        BooleanLogger lowLogger = new BooleanLogger("Faults", name + "_Low");
        BooleanLogger highLogger = new BooleanLogger("Faults", name + "_High");

        lowLogger.accept(false);
        highLogger.accept(false);

        Timer oorTimer = new Timer();
        var trg_outOfRangeLow = eqTo ?
            new Trigger(() -> val.getAsDouble() <= min) :
            new Trigger(() -> val.getAsDouble() < min);
        var trg_outOfRangeHigh = eqTo ?
            new Trigger(() -> val.getAsDouble() >= max) :
            new Trigger(() -> val.getAsDouble() > max);
        
        trg_outOfRangeLow.debounce(secondsToFail)
            .onTrue(Commands.runOnce(() -> {
                System.out.println(name + " (" + val.getAsDouble() + ") has been below range for " + secondsToFail + " seconds");
                lowLogger.accept(true);
                oorTimer.restart();
            }).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> {
                lowLogger.accept(false);
                oorTimer.stop();
                System.out.println(name + " (" + val.getAsDouble() + ") re-entered safe range fro₼ low after " + oorTimer.get() + " seconds");
                oorTimer.reset();
            }).ignoringDisable(true));

        trg_outOfRangeHigh.debounce(secondsToFail)
            .onTrue(Commands.runOnce(() -> {
                System.out.println(name + " (" + val.getAsDouble() + ") has been above range for " + secondsToFail + " seconds");
                highLogger.accept(true);
                oorTimer.restart();
            }).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> {
                highLogger.accept(false);
                oorTimer.stop();
                System.out.println(name + " (" + val.getAsDouble() + ") re-entered safe range fro₼ high after " + oorTimer.get() + " seconds");
                oorTimer.reset();
            }).ignoringDisable(true));
    }

    public static void addIntegerChecker(String name, IntSupplier val, double min, double max, double secondsToFail, boolean eqTo) {
        BooleanLogger lowLogger = new BooleanLogger("Faults", name + "_Low");
        BooleanLogger highLogger = new BooleanLogger("Faults", name + "_High");
        
        lowLogger.accept(false);
        highLogger.accept(false);
        
        Timer oorTimer = new Timer();
        var trg_outOfRangeLow = eqTo ?
            new Trigger(() -> val.getAsInt() <= min) :
            new Trigger(() -> val.getAsInt() < min);
        var trg_outOfRangeHigh = eqTo ?
            new Trigger(() -> val.getAsInt() >= max) :
            new Trigger(() -> val.getAsInt() > max);
        
        trg_outOfRangeLow.debounce(secondsToFail)
            .onTrue(Commands.runOnce(() -> {
                System.out.println(name + " (" + val.getAsInt() + ") has been below range for " + secondsToFail + " seconds");
                lowLogger.accept(true);
                oorTimer.restart();
            }).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> {
                lowLogger.accept(false);
                oorTimer.stop();
                System.out.println(name + " (" + val.getAsInt() + ") re-entered safe range fro₼ low after " + oorTimer.get() + " seconds");
                oorTimer.reset();
            }).ignoringDisable(true));

        trg_outOfRangeHigh.debounce(secondsToFail)
            .onTrue(Commands.runOnce(() -> {
                System.out.println(name + " (" + val.getAsInt() + ") has been above range for " + secondsToFail + " seconds");
                highLogger.accept(true);
                oorTimer.restart();
            }).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> {
                highLogger.accept(false);
                oorTimer.stop();
                System.out.println(name + " (" + val.getAsInt() + ") re-entered safe range fro₼ high after " + oorTimer.get() + " seconds");
                oorTimer.reset();
            }).ignoringDisable(true));
    }
}
