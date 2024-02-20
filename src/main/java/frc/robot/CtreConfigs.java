package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    public CtreConfigs() {
        var aimSlot0Configs = m_aimConfigs.Slot0;
        aimSlot0Configs.kP = AimK.kP;
        aimSlot0Configs.kI = 0;
        aimSlot0Configs.kD = 0;

        var motionMagicConfigs = m_aimConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicExpo_kV = AimK.kV;
        motionMagicConfigs.MotionMagicExpo_kA = AimK.kA;

        var shooterSlot0Configs = m_shooterConfigs.Slot0;
        shooterSlot0Configs.kP = ShooterK.kP;
        shooterSlot0Configs.kI = 0;
        shooterSlot0Configs.kD = 0;
    }
}
