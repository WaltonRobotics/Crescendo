package frc.robot.auton;

import com.pathplanner.lib.path.PathPlannerPath;

public class Paths {
    public static final PathPlannerPath ampSide1 = PathPlannerPath.fromChoreoTrajectory("amp_side.1");
    public static final PathPlannerPath ampSide2 = PathPlannerPath.fromChoreoTrajectory("amp_side.2");
    public static final PathPlannerPath ampSide3 = PathPlannerPath.fromChoreoTrajectory("amp_side.3");
    public static final PathPlannerPath ampSide4 = PathPlannerPath.fromChoreoTrajectory("amp_side.4");
    public static final PathPlannerPath ampSide5 = PathPlannerPath.fromChoreoTrajectory("amp_side.5");

    public static final PathPlannerPath ampSideAlt1 = PathPlannerPath.fromChoreoTrajectory("5_pc_skip_shot.1");
    public static final PathPlannerPath ampSideAlt2 = PathPlannerPath.fromChoreoTrajectory("5_pc_skip_shot.2");
    public static final PathPlannerPath ampSideAlt3 = PathPlannerPath.fromChoreoTrajectory("5_pc_skip_shot.3");
    public static final PathPlannerPath ampSideAlt4 = PathPlannerPath.fromChoreoTrajectory("5_pc_skip_shot.4");
    public static final PathPlannerPath ampSideAlt5 = PathPlannerPath.fromChoreoTrajectory("5_pc_skip_shot.5");

    public static final PathPlannerPath ampSideSkip1 = PathPlannerPath.fromChoreoTrajectory("5_pc_first_note.1"); // what a terrible name

    public static final PathPlannerPath sourceSide1 = PathPlannerPath.fromChoreoTrajectory("source_side.1");
    public static final PathPlannerPath sourceSide2 = PathPlannerPath.fromChoreoTrajectory("source_side.2");
    public static final PathPlannerPath sourceSide3 = PathPlannerPath.fromChoreoTrajectory("source_side.3");

    public static final PathPlannerPath sourceSideAlt1 = PathPlannerPath.fromChoreoTrajectory("source_side_alt.1");

    public static final PathPlannerPath g28Counter1 = PathPlannerPath.fromChoreoTrajectory("9500_counter.1");
    public static final PathPlannerPath g28Counter2 = PathPlannerPath.fromChoreoTrajectory("9500_counter.2");

    public static final PathPlannerPath veryAmp1 = PathPlannerPath.fromChoreoTrajectory("very_amp.1");
    public static final PathPlannerPath veryAmp2 = PathPlannerPath.fromChoreoTrajectory("very_amp.2");
    public static final PathPlannerPath veryAmp3 = PathPlannerPath.fromChoreoTrajectory("very_amp.3");

    public static final PathPlannerPath close1 = PathPlannerPath.fromChoreoTrajectory("close.1");
    public static final PathPlannerPath close2 = PathPlannerPath.fromChoreoTrajectory("close.2");
    public static final PathPlannerPath close3 = PathPlannerPath.fromChoreoTrajectory("close.3");
}