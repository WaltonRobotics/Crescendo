package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Field.*;
import static frc.robot.Constants.Shooter.*;

public class Shooter extends SubsystemBase {
    private final TalonFX m_right = new TalonFX(kRightId);
    private final TalonFX m_left = new TalonFX(kLeftId);
    private final Follower m_follower = new Follower(m_right.getDeviceID(), true);

    private final CANSparkMax m_conveyor = new CANSparkMax(kConveyorId, MotorType.kBrushless);

    private final CANSparkMax m_aim = new CANSparkMax(kConveyorId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_aim.getEncoder();
    private final PIDController m_aimController = new PIDController(kPAim, 0, 0);

    private double m_targetAngle;
    private Translation3d m_speakerPose;

    public Shooter() {
        m_left.setControl(m_follower);
        m_encoder.setPositionConversionFactor(kConversion);
        m_targetAngle = m_encoder.getPosition();
    }

    public Command shoot() {
        var shooterCmd = run(() -> m_right.set(1));
        var conveyorCmd = run(() -> m_conveyor.set(0.5));

        return Commands.parallel(shooterCmd, conveyorCmd);
    }

    public Command toAngle(double degrees) {
        var setupCmd = runOnce(() -> m_aimController.setSetpoint(degrees));
        var moveCmd = run(() -> {
            double effort = m_aimController.calculate(m_encoder.getPosition());
            m_aim.setVoltage(effort);
        })
            .until(() -> m_aimController.atSetpoint())
            .finallyDo(() -> m_aim.set(0));

        return Commands.sequence(setupCmd, moveCmd);
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
            Translation3d poseToSpeaker = m_speakerPose.minus(translation);
            m_targetAngle = Math.atan((poseToSpeaker.getZ()) / (poseToSpeaker.getX()));
        });
        var toAngleCmd = toAngle(Math.toDegrees(m_targetAngle));

        return Commands.repeatingSequence(
            getAngleCmd,
            toAngleCmd);
    }
}
