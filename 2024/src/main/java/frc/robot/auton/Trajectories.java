package frc.robot.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

public class Trajectories {
	public static final ChoreoTrajectory ampSide = Choreo.getTrajectory("amp_side");
	public static final ChoreoTrajectory sourceSide = Choreo.getTrajectory("source_side");
	public static final ChoreoTrajectory g28Counter = Choreo.getTrajectory("9500_counter");
	public static final ChoreoTrajectory veryAmp = Choreo.getTrajectory("very_amp");
	public static final ChoreoTrajectory close = Choreo.getTrajectory("close");
}