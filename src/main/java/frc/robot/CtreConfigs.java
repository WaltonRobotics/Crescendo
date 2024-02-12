package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.AimK;

public class CtreConfigs {
    public class Container {
        public static final CtreConfigs instance = new CtreConfigs();
    }

    public static CtreConfigs get() {
        return Container.instance;
    }

    public final TalonFXConfiguration m_aimConfigs = new TalonFXConfiguration();
    public final CANcoderConfiguration m_cancoderConfigs = new CANcoderConfiguration();

    public CtreConfigs() {
        m_cancoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        m_cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        m_cancoderConfigs.MagnetSensor.MagnetOffset = 0.25;

        // it feels like these should be switched but that doesn't work???
        m_aimConfigs.Feedback.SensorToMechanismRatio = 1;
        m_aimConfigs.Feedback.RotorToSensorRatio = AimK.kGearRatio;
        m_aimConfigs.Feedback.FeedbackRemoteSensorID = AimK.kCancoderId;
        m_aimConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        
        var slot0Configs = m_aimConfigs.Slot0;
        slot0Configs.kP = AimK.kP;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        var motionMagicConfigs = m_aimConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicExpo_kV = AimK.kV;
        motionMagicConfigs.MotionMagicExpo_kA = 0.12;
    }
}
