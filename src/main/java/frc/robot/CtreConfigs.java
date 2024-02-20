package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants.AimK;

public class CtreConfigs {
    public class Container {
        public static final CtreConfigs instance = new CtreConfigs();
    }

    public static CtreConfigs get() {
        return Container.instance;
    }

    public final TalonFXConfiguration m_aimConfigs = new TalonFXConfiguration();

    public CtreConfigs() {
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
