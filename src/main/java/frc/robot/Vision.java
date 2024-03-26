package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.util.AllianceFlipUtil;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class Vision {
    public static final record VisionMeasurement2d (Integer id, Double yaw, Double pitch, Double area) {}
    public static final record VisionMeasurement3d (Pose2d measure, double latency, Matrix<N3, N1> stdDevs) {}
    
    private final Matrix<N3, N1> kDefaultStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    private final PhotonCamera m_shooterCam = new PhotonCamera("ShooterCam");

    private final DoubleLogger log_shooterYaw = WaltLogger.logDouble("Vision", "shooterYaw");

    public Vision() {}

    private int getMiddleSpeakerId() {
        boolean red = AllianceFlipUtil.shouldFlip();
        return red ? 4 : 7;
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

    public Supplier<Optional<PhotonTrackedTarget>> speakerTargetSupplier() {
        return () -> {
            var result = m_shooterCam.getLatestResult();
            if (result.hasTargets()) {
                for (var target : result.targets) {
                    if(target.getFiducialId() == getMiddleSpeakerId()) {
                        log_shooterYaw.accept(target.getYaw());
                        return Optional.of(target);
                    }
                }
            }
            return Optional.empty();
        };
    }



    public void run() {
        // TODO do things in here
    }
}
