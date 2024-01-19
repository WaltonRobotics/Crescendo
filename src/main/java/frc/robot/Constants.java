package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    public static final double kFieldLength = 16.54; // meters
    public static final double kFieldWidth = 8.21; // meters

    public class AutoConstants {
        // TODO: check these values they're just copied from bubbles
        public static final double kPX = 3.25; // 8
        public static final double kPY = 3.25;
        public static final double kPTheta = 5.15; // 1
        public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(kPX),
            new PIDConstants(kPTheta),
            5,
            Math.sqrt(Math.pow(10.3125, 12.375)),
            new ReplanningConfig());
    }

    public static final class TrapK {
        public static final class TrapEleK {
// TODO: fix
            public static final int kExtendCANID = 21;
            public static final int kLowerLimitSwitchPort = 20;

            public static final double kMaxVelocity = 3.25; // Meters Per Second
            public static final double kMaxAcceleration = 2.75; // Meters Per Second Squared

            public static final double kMaxHeight = 6; // Rotations? from base
            public static final double kMinHeight = 0; // Rotations? from base
        
            public static final double kP = 22;
            public static final double kPHold = .7;
            public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration); 
        }

        public static final class TrapTiltK {
            public static final int kMotorCANID = 22;
            public static final int kAbsoluteEncoderPort = 23;

            // TODO: adjust values for PID and constants
            public static final double kP = 2;
            public static final double kPHold = .7;
            public static final double kDHold = 0;
            public static final double kHoldKs = .705;
            public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVelocity,
                kMaxAcceleration);

            public static final double kAbsMaxDegree = 35; // stolen from chargedup, MUST adjust
        }

        public static final class TrapShootK {

        }
    }
}
