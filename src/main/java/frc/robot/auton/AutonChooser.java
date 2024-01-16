package frc.robot.auton;

import java.util.EnumMap;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonChooser {
    public enum AutonOption {
        DO_NOTHING("0 - do nothing"), ONE_METER("0 - one meter"), SIMPLE_THING("0 - simple thing"), THREE_PC(
            "3 - speaker"), FIVE_PC("5 - speaker");

        public final String description;

        AutonOption(String description) {
            this.description = description;
        }

        @Override
        public String toString() {
            return name() + ": " + description;
        }
    }

    private AutonChooser() {
        // utility class, do not create instances!
    }

    private static EnumMap<AutonOption, Command> autonChooserMap = new EnumMap<>(AutonOption.class);
    private static EnumMap<AutonOption, Optional<Pose2d>> autonInitPoseMap = new EnumMap<>(AutonOption.class);
    private static final SendableChooser<AutonOption> autonNTChooser = new SendableChooser<AutonOption>();

    static {
        SmartDashboard.putData("Auton Chooser", autonNTChooser);
    }

    public static void assignAutonCommand(AutonOption auton, Command command, Pose2d holonomicStartPose) {
        autonChooserMap.put(auton, command);
        autonInitPoseMap.put(auton, Optional.ofNullable(holonomicStartPose));

        autonNTChooser.addOption(auton.description, auton);
    }

    public static void assignAutonCommand(AutonOption auton, Command command) {
        assignAutonCommand(auton, command, null);
    }

    public static void setDefaultAuton(AutonOption auton) {
        autonNTChooser.setDefaultOption(auton.description, auton);
    }

    public static Command getAuton(AutonOption auton) {
        return autonChooserMap.computeIfAbsent(auton, a -> Commands.print("========WARNING: Empty Auton!!!========")
            .withName("InvalidAuton"));
    }

    public static Optional<Pose2d> getAutonInitPose(AutonOption auton) {
        Optional<Pose2d> result = autonInitPoseMap.computeIfAbsent(auton, a -> Optional.empty());
        return result;
    }

    public static Command getChosenAutonCmd() {
        return getAuton(autonNTChooser.getSelected());
    }

    public static Optional<Pose2d> getChosenAutonInitPose() {
        var selected = AutonChooser.autonNTChooser.getSelected();

        if (selected == null) {
            return Optional.empty();
        }

        return getAutonInitPose(selected);
    }
}
