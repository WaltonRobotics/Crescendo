package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.CtreConfigs;
import frc.robot.subsystems.Swerve;
import frc.util.AllianceFlipUtil;

import static frc.robot.Constants.FieldK.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.AimK.*;
import static frc.robot.Constants.FieldK.SpeakerK.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Aim extends SubsystemBase {
    private final Supplier<Pose3d> m_robotPoseSupplier;

    private final TalonFX m_aim = new TalonFX(kAimId);
    private final CANcoder m_cancoder = new CANcoder(kCancoderId);
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
    private Translation3d m_speakerPose;
    private Translation3d m_ampPose;

    public Aim(Supplier<Pose3d> robotPoseSupplier) {
        m_robotPoseSupplier = robotPoseSupplier;

        CtreConfigs configs = CtreConfigs.get();
        m_aim.getConfigurator().apply(configs.m_aimConfigs);
        m_cancoder.getConfigurator().apply(configs.m_cancoderConfigs);

        m_targetAngle = Degrees.of(0);

        SmartDashboard.putData("mech2d", m_mech2d);
    }

    private Command toAngle(Measure<Angle> angle) {
        return run(() -> {
            m_aim.setControl(m_request.withPosition(angle.in(Rotations)));
        }).until(() -> m_cancoder.getPosition().getValueAsDouble() == angle.in(Rotations));
    }

    public void getSpeakerPose() {
        m_speakerPose = AllianceFlipUtil.apply(SpeakerK.kBlueCenterOpening);
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

    public void aimAtSpeaker() {
        var translation = AllianceFlipUtil.apply(m_robotPoseSupplier.get().getTranslation());
        System.out.println("robot z: " + translation.getZ());
        System.out.println("robot x: " + translation.getX());
        var poseToSpeaker = m_speakerPose.plus(translation);
        m_targetAngle = Radians.of(Math.atan((poseToSpeaker.getZ()) / (poseToSpeaker.getX())));
        System.out.println("z: " + poseToSpeaker.getZ());
        System.out.println("x: " + poseToSpeaker.getX());
        System.out.println("target angle: " + m_targetAngle.in(Degrees));
    }

    /**
     *  if the robot is to the right of the speaker center, shoot to the left corner of the speaker and vice versa
     *  @return a custom target angle based on the position of the robot
     */
    public Command aimSpeakerDynamic() {
        var blueCenter = kFieldLayout.getTagPose(kBlueSpeakerId).get();
        var blueRight = kFieldLayout.getTagPose(kBlueSpeakerRightId).get();

        var diff = blueCenter.getY() - m_robotPoseSupplier.get().getTranslation().getY(); // find robot pose relative to the middle of the speaker
        var rightTrans = blueRight.getTranslation(); // translation if the robot is to the left
        var lefTrans = new Translation3d(
            blueCenter.getX(), 
            blueCenter.getY() + (blueCenter.getY() - blueRight.getY()), 
            blueCenter.getZ());

        var poseToSpeaker = m_speakerPose.minus(m_robotPoseSupplier.get().getTranslation());

        if(diff < -23.125) {
            var poseToSpeakerRight = rightTrans.minus(m_robotPoseSupplier.get().getTranslation());
            return runOnce(() -> {
                m_targetAngle = Radians.of(Math.atan((poseToSpeakerRight.getZ()) / (poseToSpeakerRight.getX())));
            });
        } else if(diff > 23.125) {
            var poseToSpeakerLeft = lefTrans.minus(m_robotPoseSupplier.get().getTranslation());
            return runOnce(() -> {
                m_targetAngle = Radians.of(Math.atan((poseToSpeakerLeft.getZ()) / (poseToSpeakerLeft.getX())));
            });
        }
        return runOnce(() -> {
            m_targetAngle = Radians.of(Math.atan((poseToSpeaker.getZ()) / (poseToSpeaker.getX())));
        });
    }

    public Command shootOnTheMove(Swerve swerve) {
        return runOnce(() -> {
            var translation = m_robotPoseSupplier.get().getTranslation();
            var poseToSpeaker = m_speakerPose.minus(translation);
            var offsetPose = poseToSpeaker.plus(new Translation3d(
                // TODO figure out how long it takes to shoot and multiply
                swerve.getState().speeds.vxMetersPerSecond, swerve.getState().speeds.vyMetersPerSecond, 0));
            m_targetAngle = Radians.of(Math.atan((offsetPose.getZ()) / (offsetPose.getX())));

            m_aim.setControl(m_request.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public Command aimAtAmp() {
        var getTarget = runOnce(() -> {
            var translation = m_robotPoseSupplier.get().getTranslation();
            var poseToAmp = m_ampPose.minus(translation);
            m_targetAngle = Radians.of(Math.atan((poseToAmp.getZ()) / (poseToAmp.getX())));
        });
        var toTarget = runOnce(() -> m_aim.setControl(m_request.withPosition(m_targetAngle.in(Rotations))));

        return Commands.sequence(
            getTarget,
            toTarget);
    }

    public int getTrapId() {
        double center;
        double right;
        double left;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            Translation3d centerTag = kFieldLayout.getTagPose(kBlueCenterTrapId).get().getTranslation();
            Translation3d rightTag = kFieldLayout.getTagPose(kBlueRightTrapId).get().getTranslation();
            Translation3d leftTag = kFieldLayout.getTagPose(kBlueLeftTrapId).get().getTranslation();

            center = centerTag.getDistance(m_robotPoseSupplier.get().getTranslation());
            right = rightTag.getDistance(m_robotPoseSupplier.get().getTranslation());
            left = leftTag.getDistance(m_robotPoseSupplier.get().getTranslation());

            if (center < right && center < left) {
                return kBlueCenterTrapId;
            } else if (right < center && right < left) {
                return kBlueRightTrapId;
            } else {
                return kBlueLeftTrapId;
            }
        } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) {
            Translation3d centerTag = kFieldLayout.getTagPose(kRedCenterTrapId).get().getTranslation();
            Translation3d rightTag = kFieldLayout.getTagPose(kRedRightTrapId).get().getTranslation();
            Translation3d leftTag = kFieldLayout.getTagPose(kRedLeftTrapId).get().getTranslation();

            center = centerTag.getDistance(m_robotPoseSupplier.get().getTranslation());
            right = rightTag.getDistance(m_robotPoseSupplier.get().getTranslation());
            left = leftTag.getDistance(m_robotPoseSupplier.get().getTranslation());

            if (center < right && center < left) {
                return kRedCenterTrapId;
            } else if (right < center && right < left) {
                return kRedRightTrapId;
            } else {
                return kRedLeftTrapId;
            }
        }
        return 0;
    }

    public Command aimAtTrap() {
        var getAngleCmd = run(() -> {
            var translation = m_robotPoseSupplier.get();
            int trapId = getTrapId();
            if (trapId == 0) {
                return; // or something like that i just don't want robot code to crash 😭
            }
            var poseToTrap = kFieldLayout.getTagPose(trapId).get().minus(translation);
            m_targetAngle = Radians.of(Math.atan((poseToTrap.getZ()) / (poseToTrap.getX())));
        });
        var toAngleCmd = toAngle(m_targetAngle);

        return Commands.repeatingSequence(
            getAngleCmd,
            toAngleCmd);
    }

    public Command stageMode() {
        Pose3d pose = m_robotPoseSupplier.get();
        if (pose.getY() > kBlueStageClearanceRight && pose.getY() < kBlueStageClearanceLeft) {
            if (pose.getX() > kBlueStageClearanceDS && pose.getX() < kBlueStageClearanceCenter) {
                return toAngle(kStageClearance);
            } else if (pose.getX() > kRedStageClearanceDS && pose.getX() < kRedStageClearanceCenter) {
                return toAngle(kStageClearance);
            }
        }
        return Commands.none();
    }

    public void simulationPeriodic() {
        m_aim.getSimState().setSupplyVoltage(12);
        var volts = m_aim.getSimState().getMotorVoltage();
        m_aimSim.setInputVoltage(volts);

        double angle = Units.radiansToRotations(m_aimSim.getAngleRads());
        double velocity = Units.radiansToRotations(m_aimSim.getVelocityRadPerSec());
        m_aim.getSimState().setRawRotorPosition(angle);
        m_aim.getSimState().setRotorVelocity(velocity);
        m_cancoder.getSimState().setRawPosition(angle);
        m_cancoder.getSimState().setVelocity(velocity);

        m_aim2d.setAngle(Units.rotationsToDegrees(
            m_aim.getPosition().getValueAsDouble())); // TODO: make this render correctly with the real robot too

        SmartDashboard.putNumber("cancoderPos",
            m_cancoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("simVoltage", volts);
        SmartDashboard.putNumber("simVelo", m_aimSim.getVelocityRadPerSec());
        SmartDashboard.putNumber("simAngle", angle);
        SmartDashboard.putNumber("targetAngle", m_targetAngle.in(Degrees));

        m_aimSim.update(0.020);
    }
}
