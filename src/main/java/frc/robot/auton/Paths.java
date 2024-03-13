package frc.robot.auton;

import com.pathplanner.lib.path.PathPlannerPath;

public class Paths {
    public static final PathPlannerPath ampSide1 = PathPlannerPath.fromChoreoTrajectory("amp_side.1");
    public static final PathPlannerPath ampSide2 = PathPlannerPath.fromChoreoTrajectory("amp_side.2");
    public static final PathPlannerPath ampSide3 = PathPlannerPath.fromChoreoTrajectory("amp_side.3");
    public static final PathPlannerPath ampSide4 = PathPlannerPath.fromChoreoTrajectory("amp_side.4");
    public static final PathPlannerPath ampSideToAmp = PathPlannerPath.fromChoreoTrajectory("amp_3pc_amp");

    public static final PathPlannerPath ampStart1 = PathPlannerPath.fromChoreoTrajectory("amp_start.1");
    public static final PathPlannerPath ampStart2 = PathPlannerPath.fromChoreoTrajectory("amp_start.2");
    public static final PathPlannerPath ampStart3 = PathPlannerPath.fromChoreoTrajectory("amp_start.3");

    public static final PathPlannerPath center1 = PathPlannerPath.fromChoreoTrajectory("clear_3_center.1");
    public static final PathPlannerPath center2 = PathPlannerPath.fromChoreoTrajectory("clear_3_center.2");
    public static final PathPlannerPath center3 = PathPlannerPath.fromChoreoTrajectory("clear_3_center.3");

    public static final PathPlannerPath clearCenter1 = PathPlannerPath.fromChoreoTrajectory("clear_center.1");
    public static final PathPlannerPath clearCenter2 = PathPlannerPath.fromChoreoTrajectory("clear_center.2");
    public static final PathPlannerPath clearCenter3 = PathPlannerPath.fromChoreoTrajectory("clear_center.3");
    public static final PathPlannerPath clearCenter4 = PathPlannerPath.fromChoreoTrajectory("clear_center.4");
    public static final PathPlannerPath clearCenter5 = PathPlannerPath.fromChoreoTrajectory("clear_center.5");
}
