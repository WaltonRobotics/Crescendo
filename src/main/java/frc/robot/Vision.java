package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldK;
import frc.util.AllianceFlipUtil;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.DoubleLogger;
import frc.util.logging.WaltLogger.Pose3dLogger;
import frc.util.logging.WaltLogger.Transform3dLogger;

import static frc.robot.Constants.FieldK.kTagLayout;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class Vision {

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.5, 1.5, 6.24);
    public static final Matrix<N3, N1> kMultipleTagStdDevs = VecBuilder.fill(0.3, 0.3, 3.14);

    public static final double kMaxPoseHeight = 0.405;
    public static final double kMaxPoseAngle = 0.3;

    // Total of 16 AprilTags
    // https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf (page 35 and more)
    // Tag Locations (1-16) | Source: 1,2,9,10 | Speaker: 3,4,7,8 | Amp: 5,6 | Stage 11,12,13,14,15,16
    // First half of locations are on red side, second half on blue side
    // (ex. source: 1,2 is red, 9,10)

    // Unsure if getFiducialId() returns the tags from 1-16 or 0-15 (assuming 0-15)
    public static final double[] TAG_WEIGHTS = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};


    public static final record PhotonMeasurement (PhotonTrackedTarget target, double latencyMilliseconds) {}
    public static final record VisionMeasurement2d (Integer id, Double yaw, Double pitch, Double area) {}
    public static final record VisionMeasurement3d (EstimatedRobotPose estimate, Matrix<N3, N1> stdDevs) {}
    public static final record VisMeas3dEx (boolean hasTarget, Optional<VisionMeasurement3d> measOpt) {}
    
    // private final Matrix<N3, N1> kDefaultStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    private final PhotonCamera m_shooterCam = new PhotonCamera("ShooterCam");
    private final PhotonCamera m_frontCam = new PhotonCamera("FrontCam");
    private final Transform3d m_frontCam_robotToCam = new Transform3d(
        Units.inchesToMeters(9.095), Units.inchesToMeters(11.212), Units.inchesToMeters(10.739), 
        new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(0 - 20), Units.degreesToRadians(-20)));
    public final PhotonPoseEstimator m_frontCam_poseEstimator = 
        new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_frontCam, m_frontCam_robotToCam);

    private final Pose3dLogger log_frontCamOnRobot = WaltLogger.logPose3d("Vision", "frontCamOffset");
    private final DoubleLogger log_shooterYaw = WaltLogger.logDouble("Vision", "shooterYaw");
    private final Transform3dLogger log_speakerTag = WaltLogger.logTransform3d("Vision", "speakerTag");
    private final Pose3dLogger log_frontCamRawEstimate = WaltLogger.logPose3d("Vision", "frontCamRawEstimate");
    private final Pose3dLogger log_frontCamFilteredEstimate = WaltLogger.logPose3d("Vision", "frontCamFilteredEstimate");

    public Vision() {
        m_frontCam_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        log_frontCamOnRobot.accept(new Pose3d().plus(m_frontCam_robotToCam));
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(
        Pose2d estimatedPose, PhotonPipelineResult pipelineResult) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = pipelineResult.getTargets();
        int numTags = 0;
        double avgDist = 0;
        double avgWeight = 0;
        for (var tgt : targets) {
            var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
            avgWeight += TAG_WEIGHTS[tgt.getFiducialId() - 1];
        }
        if (numTags == 0) return estStdDevs;

        avgDist /= numTags;
        avgWeight /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultipleTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        estStdDevs = estStdDevs.times(avgWeight);

        return estStdDevs;
    }

    public static int getMiddleSpeakerId() {
        boolean red = AllianceFlipUtil.shouldFlip();
        return red ? 4 : 7;
    }

    public static Pose3d getMiddleSpeakerTagPose() {
        return kTagLayout.getTagPose(getMiddleSpeakerId()).get();
    }

    private Optional<EstimatedRobotPose> filterEstimation(EstimatedRobotPose estPose) {
        return Optional.of(estPose);

        // if (
        //     FieldK.inField(estPose.estimatedPose) &&
        //     estPose.estimatedPose.getZ() < kMaxPoseHeight
        // ) {
        //     return Optional.of(estPose);
        // }
        // return Optional.empty();
    }

    public VisMeas3dEx getFrontCamPoseEst() {
        var result = m_frontCam.getLatestResult();
        var estimateOpt = m_frontCam_poseEstimator.update();
        if (estimateOpt.isEmpty()) return new VisMeas3dEx(result.hasTargets(), Optional.empty());
        log_frontCamRawEstimate.accept(estimateOpt.get().estimatedPose);
        if (FieldK.inField(estimateOpt.get().estimatedPose) && estimateOpt.get().estimatedPose.getZ() >= 0) {
            var filtered = estimateOpt.get();
            var stdDevs = getEstimationStdDevs(filtered.estimatedPose.toPose2d(), result);
            log_frontCamFilteredEstimate.accept(filtered.estimatedPose);
            return new VisMeas3dEx(true, Optional.of(new VisionMeasurement3d(filtered, stdDevs)));
        }

        return new VisMeas3dEx(false, Optional.empty());
    }

    public Supplier<Optional<List<VisionMeasurement2d>>> shooterDataSupplier() {
        return () -> {
            var result = m_shooterCam.getLatestResult();
            if (result.hasTargets()) {
                var targets = result.getTargets();
                var measurements = new ArrayList<VisionMeasurement2d>();
                
                for (PhotonTrackedTarget t : targets) {
                    measurements.add(new VisionMeasurement2d(t.getFiducialId(), t.getYaw(), t.getPitch(), t.getArea()));
                }

                return Optional.of(measurements);
            }
            return Optional.empty();
        };
    }

    public Supplier<Optional<PhotonMeasurement>> speakerTargetSupplier() {
        return () -> {
            var result = m_shooterCam.getLatestResult();
            if (result.hasTargets()) {
                for (var target : result.targets) {
                    if(target.getFiducialId() == getMiddleSpeakerId()) {
                        log_shooterYaw.accept(target.getYaw());
                        log_speakerTag.accept(target.getBestCameraToTarget());
                        var msmt = new PhotonMeasurement(target, result.getLatencyMillis());
                        return Optional.of(msmt);
                    }
                }
            }
            return Optional.empty();
        };
    }

}