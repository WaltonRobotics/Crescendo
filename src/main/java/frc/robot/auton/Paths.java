package frc.robot.auton;

import com.pathplanner.lib.path.PathPlannerPath;

public class Paths {
    public static final PathPlannerPath ampSide1 = PathPlannerPath.fromChoreoTrajectory("amp_side.1");
    public static final PathPlannerPath ampSide2 = PathPlannerPath.fromChoreoTrajectory("amp_side.2");
    public static final PathPlannerPath ampSide3 = PathPlannerPath.fromChoreoTrajectory("amp_side.3");

    public static final PathPlannerPath center3 = PathPlannerPath.fromChoreoTrajectory("clear_3_center");
}
