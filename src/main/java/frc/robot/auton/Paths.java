package frc.robot.auton;

import com.pathplanner.lib.path.PathPlannerPath;

public class Paths {
    public static final PathPlannerPath ampSide1 = PathPlannerPath.fromChoreoTrajectory("amp_side.1");
    public static final PathPlannerPath ampSide2 = PathPlannerPath.fromChoreoTrajectory("amp_side.2");
    public static final PathPlannerPath ampSide3 = PathPlannerPath.fromChoreoTrajectory("amp_side.3");
    public static final PathPlannerPath ampSide4 = PathPlannerPath.fromChoreoTrajectory("amp_side.4");

    public static final PathPlannerPath sourceSide1 = PathPlannerPath.fromChoreoTrajectory("source_side.1");
    public static final PathPlannerPath sourceSide2 = PathPlannerPath.fromChoreoTrajectory("source_side.2");
    public static final PathPlannerPath sourceSide3 = PathPlannerPath.fromChoreoTrajectory("source_side.3");

    public static final PathPlannerPath sourceSideAlt1 = PathPlannerPath.fromChoreoTrajectory("source_side_alt.1");
}