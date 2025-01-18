package frc.util;

import edu.wpi.first.math.geometry.Pose2d;

public class AdvantageScopeUtil {
    public static double[] toDoubleArr(Pose2d pose) {
        var t = pose.getTranslation();
        var r = pose.getRotation().getDegrees();
        return new double[] {
            t.getX(), t.getY(), r
        };
    }
}
