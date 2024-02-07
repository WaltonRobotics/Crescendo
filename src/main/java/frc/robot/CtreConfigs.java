package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import frc.robot.Constants.AimK;

public class CtreConfigs {
    public class Container {
        public static final CtreConfigs INSTANCE = new CtreConfigs();
    }

    public static CtreConfigs get() {
        return Container.INSTANCE;
    }

    public final TalonFXConfiguration m_aimConfigs = new TalonFXConfiguration();

    public CtreConfigs() {
        m_aimConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_aimConfigs.Feedback.SensorToMechanismRatio = AimK.kGearRatio;
        var slot0Configs = m_aimConfigs.Slot0;
        // slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        // slot0Configs.kS = kS;
        // slot0Configs.kV = kV;
        // slot0Configs.kA = kA;
        slot0Configs.kP = AimK.kP;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        // slot0Configs.kG = kG;

        var motionMagicConfigs = m_aimConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        // motionMagicConfigs.MotionMagicAcceleration = kAcceleration;
        motionMagicConfigs.MotionMagicExpo_kV = AimK.kV;
        motionMagicConfigs.MotionMagicExpo_kA = 0.12;
    }
}
