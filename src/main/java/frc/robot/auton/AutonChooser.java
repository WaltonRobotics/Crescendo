package frc.robot.auton;

import static edu.wpi.first.units.Units.Degrees;

import java.util.EnumMap;
import java.util.Optional;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonChooser {
    public enum AutonOption {
        // TODO fix startAngle
        DO_NOTHING("0 - do nothing", new ChoreoTrajectory(), Degrees.of(0)),
        ONE_METER("0 - one meter", Trajectories.oneMeter, Degrees.of(0)),
        SIMPLE_THING("0 - simple thing", Trajectories.simpleThing, Degrees.of(0)),
        THREE_PC("3 - speaker", Trajectories.threePc, Degrees.of(30)),
        FOUR_PC("4 - speaker", Trajectories.fourPc, Degrees.of(30)),
        FIVE_PC("5 - speaker", Trajectories.fivePc, Degrees.of(30));

        public final String m_description;
        public final ChoreoTrajectory m_traj;
        public final Measure<Angle> m_startAngle;

        AutonOption(String description, ChoreoTrajectory traj, Measure<Angle> startAngle) {
            m_description = description;
            m_traj = traj;
            m_startAngle = startAngle;
        }

        @Override
        public String toString() {
            return name() + ": " + m_description;
        }
    }

    private AutonChooser() {
        // utility class, do not create instances!
    }

    private static EnumMap<AutonOption, Command> autonChooserMap = new EnumMap<>(AutonOption.class);
    private static EnumMap<AutonOption, Optional<Pose2d>> autonInitPoseMap = new EnumMap<>(AutonOption.class);
    private static final SendableChooser<AutonOption> autonNTChooser = new SendableChooser<AutonOption>();

    static {
        SmartDashboard.putData("AutonChooser", autonNTChooser);
    }

    public static void assignAutonCommand(AutonOption auton, Command command, Pose2d holonomicStartPose) {
        autonChooserMap.put(auton, command);
        autonInitPoseMap.put(auton, Optional.ofNullable(holonomicStartPose));

        autonNTChooser.addOption(auton.m_description, auton);
    }

    public static void assignAutonCommand(AutonOption auton, Command command) {
        assignAutonCommand(auton, command, null);
    }

    public static void setDefaultAuton(AutonOption auton) {
        autonNTChooser.setDefaultOption(auton.m_description, auton);
    }

    public static Command getAuton(AutonOption auton) {
        return autonChooserMap.computeIfAbsent(auton, a -> Commands.print("warning: empty auton!")
            .withName("InvalidAuton"));
    }

    public static Optional<Pose2d> getAutonInitPose(AutonOption auton) {
        Optional<Pose2d> result = autonInitPoseMap.computeIfAbsent(auton, a -> Optional.empty());
        return result;
    }

    public static ChoreoTrajectory getTrajectory(AutonOption auton) {
        return auton.m_traj;
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

    public static ChoreoTrajectory getChosenTrajectory() {
        var selected = AutonChooser.autonNTChooser.getSelected();
        return selected.m_traj;
    }

    public static Measure<Angle> getChosenAutonAimInit() {
        var selected = AutonChooser.autonNTChooser.getSelected();
        return selected.m_startAngle;
    }
}
