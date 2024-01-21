package com.pathplanner.lib.path;

// import java.io.BufferedReader;
// import java.io.File;
// import java.io.FileReader;
// import java.util.ArrayList;
// import java.util.List;
// import java.util.Optional;

// import org.json.simple.JSONArray;
// import org.json.simple.JSONObject;
// import org.json.simple.parser.JSONParser;

// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.path.PathPoint;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Filesystem;

public class PathPlannerDeShitter {
  //   /**
  //  * Load a Choreo trajectory as a PathPlannerPath
  //  *
  //  * @param trajectoryName The name of the Choreo trajectory to load. This should be just the name
  //  *     of the trajectory. The trajectories must be located in the "deploy/choreo" directory.
  //  * @return PathPlannerPath created from the given Choreo trajectory file
  //  */
  // public static PathPlannerPath fromChoreoTrajectory(String trajectoryName) {
  //   try (BufferedReader br =
  //       new BufferedReader(
  //           new FileReader(
  //               new File(Filesystem.getDeployDirectory(), "choreo/" + trajectoryName + ".traj")))) {
  //     StringBuilder fileContentBuilder = new StringBuilder();
  //     String line;
  //     while ((line = br.readLine()) != null) {
  //       fileContentBuilder.append(line);
  //     }

  //     String fileContent = fileContentBuilder.toString();
  //     JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

  //     List<PathPlannerTrajectory.State> trajStates = new ArrayList<>();
  //     for (var s : (JSONArray) json.get("samples")) {
  //       JSONObject sample = (JSONObject) s;
  //       PathPlannerTrajectory.State state = new PathPlannerTrajectory.State();

  //       double time = ((Number) sample.get("timestamp")).doubleValue();
  //       double xPos = ((Number) sample.get("x")).doubleValue();
  //       double yPos = ((Number) sample.get("y")).doubleValue();
  //       double rotationRad = ((Number) sample.get("heading")).doubleValue();
  //       double xVel = ((Number) sample.get("velocityX")).doubleValue();
  //       double yVel = ((Number) sample.get("velocityY")).doubleValue();
  //       double angularVelRps = ((Number) sample.get("angularVelocity")).doubleValue();

  //       state.timeSeconds = time;
  //       state.velocityMps = Math.hypot(xVel, yVel);
  //       state.accelerationMpsSq = 0.0; // Not encoded, not needed anyway
  //       state.headingAngularVelocityRps = 0.0; // Not encoded, only used for diff drive anyway
  //       state.positionMeters = new Translation2d(xPos, yPos);
  //       state.heading = new Rotation2d(xVel, yVel);
  //       state.targetHolonomicRotation = new Rotation2d(rotationRad);
  //       state.holonomicAngularVelocityRps = Optional.of(angularVelRps);
  //       state.curvatureRadPerMeter = 0.0; // Not encoded, only used for diff drive anyway
  //       state.constraints =
  //           new PathConstraints(
  //               Double.POSITIVE_INFINITY,
  //               Double.POSITIVE_INFINITY,
  //               Double.POSITIVE_INFINITY,
  //               Double.POSITIVE_INFINITY);

  //       trajStates.add(state);
  //     }

  //     PathPlannerPath path =
  //         new PathPlannerPath(
  //             new PathConstraints(
  //                 Double.POSITIVE_INFINITY,
  //                 Double.POSITIVE_INFINITY,
  //                 Double.POSITIVE_INFINITY,
  //                 Double.POSITIVE_INFINITY),
  //             new GoalEndState(
  //                 trajStates.get(trajStates.size() - 1).velocityMps,
  //                 trajStates.get(trajStates.size() - 1).targetHolonomicRotation,
  //                 true));

  //     List<PathPoint> pathPoints = new ArrayList<>();
  //     for (var state : trajStates) {
  //       pathPoints.add(new PathPoint(state.positionMeters));
  //     }

  //     path.allPoints = pathPoints;
  //     path.isChoreoPath = true;
  //     path.choreoTrajectory = new PathPlannerTrajectory(trajStates);

  //     return path;
  //   } catch (Exception e) {
  //     e.printStackTrace();
  //     return null;
  //   }
  // }
}
