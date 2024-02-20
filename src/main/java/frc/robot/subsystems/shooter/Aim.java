package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CtreConfigs;
import frc.robot.auton.AutonChooser;
import frc.util.AllianceFlipUtil;

import static frc.robot.Constants.FieldK.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.AimK.*;
import static frc.robot.Constants.FieldK.SpeakerK.*;
import static frc.robot.Robot.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Aim extends SubsystemBase {
    private final Supplier<Pose3d> m_robotPoseSupplier;

    private final TalonFX m_aim = new TalonFX(kAimId);
    private final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    private final DCMotor m_aimGearbox = DCMotor.getFalcon500(1);
    private final SingleJointedArmSim m_aimSim = new SingleJointedArmSim(
        m_aimGearbox, kGearRatio, 0.761, Units.inchesToMeters(19.75),
        kMinAngle.in(Radians), kMaxAngle.in(Radians), true, Units.degreesToRadians(0));

    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_aimPivot = m_mech2d.getRoot("aimPivot", 30, 30);
    private final MechanismLigament2d m_aim2d = m_aimPivot.append(
        new MechanismLigament2d(
            "aim2d",
            30,
            Units.radiansToDegrees(m_aimSim.getAngleRads()),
            10,
            new Color8Bit(Color.kHotPink)));

    private Measure<Angle> m_targetAngle;
    private Translation3d m_ampPose;

    private final DigitalInput m_home = new DigitalInput(kHomeSwitch);
    private final Trigger m_homeTrigger = new Trigger(m_home::get).negate();

    private final Trigger m_atStart = new Trigger(
        () -> m_aim.getPosition().getValueAsDouble() == AutonChooser.getChosenAutonAimInit().in(Rotations));

    public Aim(Supplier<Pose3d> robotPoseSupplier) {
        m_robotPoseSupplier = robotPoseSupplier;

        CtreConfigs configs = CtreConfigs.get();
        m_aim.getConfigurator().apply(configs.m_aimConfigs);

        m_targetAngle = Degrees.of(0);

        SmartDashboard.putData("Mech2d", m_mech2d);

        // TODO check this value
        m_homeTrigger.onTrue(Commands.runOnce(() -> m_aim.setPosition(0))
            .ignoringDisable(true));
        m_atStart.onTrue(Commands.runOnce(() -> m_aim.setNeutralMode(NeutralModeValue.Brake))
            .ignoringDisable(true));
    }

    private Command toAngle(Measure<Angle> angle) {
        return run(() -> {
            m_aim.setControl(m_request.withPosition(angle.in(Rotations)));
        });
    }

    public Command teleop(DoubleSupplier power) {
        return run(() -> {
            double powerVal = MathUtil.applyDeadband(power.getAsDouble(), 0.1);
            m_targetAngle = m_targetAngle.plus(Degrees.of(powerVal * 1.2));
            m_targetAngle = Degrees
                .of(MathUtil.clamp(m_targetAngle.magnitude(), kMinAngle.magnitude(), kMaxAngle.magnitude()));

            m_aim.setControl(m_request.withPosition(m_targetAngle.in(Rotations)));

            stageMode();

            SmartDashboard.putNumber("aimSpeed", m_aim.get());
            SmartDashboard.putNumber("aimPos",
                Units.rotationsToDegrees(m_aim.getPosition().getValueAsDouble()));
        });
    }

    public Command goTo90() {
        return runOnce(() -> {
            m_targetAngle = Degrees.of(90);

            m_aim.setControl(m_request.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public Command goTo30() {
        return runOnce(() -> {
            m_targetAngle = Degrees.of(30);

            m_aim.setControl(m_request.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public Command toTarget() {
        return run(() -> m_aim.setControl(m_request.withPosition(m_targetAngle.in(Rotations))));
    }

    public void setAimTarget() {
        var translation = AllianceFlipUtil.apply(m_robotPoseSupplier.get().getTranslation());
        var poseToSpeaker = speakerPose.plus(translation);
        m_targetAngle = Radians.of(Math.atan((poseToSpeaker.getZ()) / (poseToSpeaker.getX())));
    }

    /**
     * if the robot is to the right of the speaker center, shoot to the left corner
     * of the speaker and vice versa
     * 
     * @return a custom target angle based on the position of the robot
     */
    public Command changeSpeakerTarget() {
        var blueCenter = kFieldLayout.getTagPose(kBlueSpeakerId).get();
        var blueRight = kFieldLayout.getTagPose(kBlueSpeakerRightId).get();

        var diff = blueCenter.getY() - m_robotPoseSupplier.get().getTranslation().getY(); // find robot pose relative to
                                                                                          // the middle of the speaker
        var rightTrans = blueRight.getTranslation(); // translation if the robot is to the left
        var lefTrans = new Translation3d(
            blueCenter.getX(),
            blueCenter.getY() + (blueCenter.getY() - blueRight.getY()),
            blueCenter.getZ());

        var poseToSpeaker = speakerPose.minus(m_robotPoseSupplier.get().getTranslation());

        if (diff < -23.125) {
            var poseToSpeakerRight = rightTrans.minus(m_robotPoseSupplier.get().getTranslation());
            return runOnce(() -> {
                m_targetAngle = Radians.of(Math.atan((poseToSpeakerRight.getZ()) / (poseToSpeakerRight.getX())));
            });
        } else if (diff > 23.125) {
            var poseToSpeakerLeft = lefTrans.minus(m_robotPoseSupplier.get().getTranslation());
            return runOnce(() -> {
                m_targetAngle = Radians.of(Math.atan((poseToSpeakerLeft.getZ()) / (poseToSpeakerLeft.getX())));
            });
        }
        return runOnce(() -> {
            m_targetAngle = Radians.of(Math.atan((poseToSpeaker.getZ()) / (poseToSpeaker.getX())));
        });
    }

    public void aimAtAmp() {
        var translation = m_robotPoseSupplier.get().getTranslation();
        var poseToAmp = m_ampPose.minus(translation);
        m_targetAngle = Radians.of(Math.atan((poseToAmp.getZ()) / (poseToAmp.getX())));
    }

    public Command stageMode() {
        Pose3d pose = m_robotPoseSupplier.get();
        if (pose.getY() > kBlueStageClearanceRight && pose.getY() < kBlueStageClearanceLeft) {
            if (pose.getX() > kBlueStageClearanceDs && pose.getX() < kBlueStageClearanceCenter) {
                return toAngle(kStageClearance);
            } else if (pose.getX() > kRedStageClearanceDs && pose.getX() < kRedStageClearanceCenter) {
                return toAngle(kStageClearance);
            }
        }
        return Commands.none();
    }

    public void coast() {
        m_aim.setNeutralMode(NeutralModeValue.Coast);
    }

    public void simulationPeriodic() {
        m_aim.getSimState().setSupplyVoltage(12);
        var volts = m_aim.getSimState().getMotorVoltage();
        m_aimSim.setInputVoltage(volts);

        double angle = Units.radiansToRotations(m_aimSim.getAngleRads());
        double velocity = Units.radiansToRotations(m_aimSim.getVelocityRadPerSec());
        m_aim.getSimState().setRawRotorPosition(angle);
        m_aim.getSimState().setRotorVelocity(velocity);

        m_aim2d.setAngle(Units.rotationsToDegrees(
            m_aim.getPosition().getValueAsDouble())); // TODO: make this render correctly with the real robot too

        SmartDashboard.putNumber("simVoltage", volts);
        SmartDashboard.putNumber("simVelo", m_aimSim.getVelocityRadPerSec());
        SmartDashboard.putNumber("simAngle", angle);
        SmartDashboard.putNumber("targetAngle", m_targetAngle.in(Degrees));

        m_aimSim.update(0.020);
    }
}
