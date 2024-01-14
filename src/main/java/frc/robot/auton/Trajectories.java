package frc.robot.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

public class Trajectories {
	public static final ChoreoTrajectory oneMeter = Choreo.getTrajectory("oneMeter");
	public static final ChoreoTrajectory simpleThing = Choreo.getTrajectory("simpleThing");
	public static final ChoreoTrajectory fivePc = Choreo.getTrajectory("5pc");
	public static final ChoreoTrajectory[] threePc = {
		Choreo.getTrajectory("3pc1"),
		Choreo.getTrajectory("3pc2")
	};
}
