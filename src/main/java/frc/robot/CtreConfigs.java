package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import frc.robot.Constants.*;

public class CtreConfigs {
    public class Container {
        public static final CtreConfigs instance = new CtreConfigs();
    }

    public static CtreConfigs get() {
        return Container.instance;
    }

    public final TalonFXConfiguration m_aimConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration m_shooterConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration m_rightShooterConfigs = new TalonFXConfiguration();

    public CtreConfigs() {
        var aimSlot0Configs = m_aimConfigs.Slot0;
        aimSlot0Configs.kP = AimK.kP;
        aimSlot0Configs.kI = 0;
        aimSlot0Configs.kD = 0;

        m_aimConfigs.Feedback = m_aimConfigs.Feedback
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(AimK.kGearRatio);

        m_aimConfigs.MotorOutput = m_aimConfigs.MotorOutput
            .withPeakForwardDutyCycle(0.25)
            .withPeakReverseDutyCycle(0.25);

        m_aimConfigs.CurrentLimits = m_aimConfigs.CurrentLimits
            .withStatorCurrentLimit(15)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(8)
            .withSupplyCurrentLimitEnable(true);

        var motionMagicConfigs = m_aimConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicExpo_kV = AimK.kV;
        motionMagicConfigs.MotionMagicExpo_kA = AimK.kA;

        var shooterSlot0Configs = m_shooterConfigs.Slot0;
        shooterSlot0Configs.kP = ShooterK.kP;
        shooterSlot0Configs.kI = 0;
        shooterSlot0Configs.kD = 0;

        m_shooterConfigs.Feedback = m_shooterConfigs.Feedback.withSensorToMechanismRatio(ShooterK.kGearRatio);
    }
}
