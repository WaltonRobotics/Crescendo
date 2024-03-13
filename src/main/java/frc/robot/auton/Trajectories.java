package frc.robot.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

public class Trajectories {
	public static final ChoreoTrajectory ampSide = Choreo.getTrajectory("amp_side");
	public static final ChoreoTrajectory ampSideToAmp = Choreo.getTrajectory("amp_3pc_amp");

	public static final ChoreoTrajectory ampStart = Choreo.getTrajectory("amp_start");
	
	public static final ChoreoTrajectory clear3Center = Choreo.getTrajectory("clear_3_center");
	
	public static final ChoreoTrajectory clearCenter = Choreo.getTrajectory("clear_center");
}
