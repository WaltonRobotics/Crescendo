package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FieldK.*;
import static frc.robot.Constants.ShooterK.*;

public class Shooter extends SubsystemBase {
    private final TalonFX m_right = new TalonFX(kRightId);
    private final TalonFX m_left = new TalonFX(kLeftId);
    private final Follower m_follower = new Follower(m_right.getDeviceID(), true);

    private final CANSparkMax m_conveyor = new CANSparkMax(kConveyorId, MotorType.kBrushless);

    private final TalonFX m_aim = new TalonFX(kAimId);
    // private final PIDController m_aimController = new PIDController(kP, 0, 0);
    private final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    private double m_targetAngle;
    private Translation3d m_speakerPose;

    public Shooter() {
        m_left.setControl(m_follower);
        ctreConfigs();
        // TODO fix conversion
        m_targetAngle = m_aim.getPosition().getValueAsDouble();
    }

    private void ctreConfigs() {
        var talonFxConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFxConfigs.Slot0;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        slot0Configs.kA = kA;
        slot0Configs.kP = kP;
        slot0Configs.kI = 0;
        slot0Configs.kD = kD;

        var motionMagicConfigs = talonFxConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicExpo_kV = 0.12;
        motionMagicConfigs.MotionMagicExpo_kA = 0.1;

        m_aim.getConfigurator().apply(talonFxConfigs);
    }

    public Command shoot() {
        return run(() -> {
            m_right.set(1);
            m_conveyor.set(0.5);
        });
    }

    public Command toAngle(double degrees) {
        // var setupCmd = runOnce(() -> m_aimController.setSetpoint(degrees));
        // var moveCmd = run(() -> {
        // double effort =
        // m_aimController.calculate(m_aim.getPosition().getValueAsDouble());
        // m_aim.setVoltage(effort);
        // })
        // .until(() -> m_aimController.atSetpoint())
        // .finallyDo(() -> m_aim.set(0));

        // return Commands.sequence(setupCmd, moveCmd);
        return run(() -> m_aim.setControl(m_request.withPosition(degrees / 360)));
    }

    public void getSpeakerPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            m_speakerPose = kBlueSpeakerPose;
        } else {
            m_speakerPose = kRedSpeakerPose;
        }
    }

    public Command aimAtSpeaker(Swerve swerve) {
        var getAngleCmd = run(() -> {
            var translation = swerve.getPose().getTranslation();
            var poseToSpeaker = m_speakerPose.minus(translation);
            m_targetAngle = Math.atan((poseToSpeaker.getZ()) / (poseToSpeaker.getX()));
        });
        var toAngleCmd = toAngle(Math.toDegrees(m_targetAngle));

        return Commands.repeatingSequence(
            getAngleCmd,
            toAngleCmd);
    }
}
