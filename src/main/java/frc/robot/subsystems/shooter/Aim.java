package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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

import static frc.robot.Constants.FieldK.*;
import static frc.robot.Constants.AimK.*;

import java.util.function.Supplier;

public class Aim extends SubsystemBase {
    private final Supplier<Pose3d> m_robotPoseSupplier;

    private final TalonFX m_aim = new TalonFX(kAimId);
    // private final CANcoder m_cancoder = new CANcoder(kCancoderId);
    private final TalonFXSimState m_talonFxSim = m_aim.getSimState();
    // private final MotionMagicExpoVoltage m_request = new
    // MotionMagicExpoVoltage(0);
    private final PositionVoltage m_request = new PositionVoltage(0)
        .withSlot(0)
        .withUpdateFreqHz(100)
        .withEnableFOC(true)
        .withFeedForward(0);

    private final DCMotor m_aimGearbox = DCMotor.getFalcon500(1);
    private final SingleJointedArmSim m_aimSim = new SingleJointedArmSim(
        m_aimGearbox, kGearRatio, 0.761, Units.inchesToMeters(19.75),
        Units.degreesToRadians(0), Units.degreesToRadians(200), true, Units.degreesToRadians(0));

    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_aimPivot = m_mech2d.getRoot("AimPivot", 30, 30);
    private final MechanismLigament2d m_aim2d = m_aimPivot.append(
        new MechanismLigament2d(
            "aim",
            30,
            Units.radiansToDegrees(m_aimSim.getAngleRads()),
            10,
            new Color8Bit(Color.kHotPink)));

    private double m_targetAngle;
    private Translation3d m_speakerPose;

    public Aim(Supplier<Pose3d> robotPoseSupplier) {
        m_robotPoseSupplier = robotPoseSupplier;
        ctreConfigs();
        m_targetAngle = 0;
        SmartDashboard.putData("Mech2d", m_mech2d);
    }

    private void ctreConfigs() {
        var talonFxConfigs = new TalonFXConfiguration();
        talonFxConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        talonFxConfigs.Feedback.SensorToMechanismRatio = kGearRatio;
        var slot0Configs = talonFxConfigs.Slot0;
        // slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        // slot0Configs.kS = kS;
        // slot0Configs.kV = kV;
        // slot0Configs.kA = kA;
        slot0Configs.kP = kP;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        // slot0Configs.kG = kG;

        // var motionMagicConfigs = talonFxConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        // // motionMagicConfigs.MotionMagicAcceleration = kAcceleration;
        // motionMagicConfigs.MotionMagicExpo_kV = kV;
        // motionMagicConfigs.MotionMagicExpo_kA = 0.12;

        m_aim.getConfigurator().apply(talonFxConfigs);
    }

    private Command toAngle(double degrees) {
        return run(() -> {
            m_aim.setControl(m_request.withPosition(degrees / 360.0));
        });
    }

    public void getSpeakerPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            m_speakerPose = kBlueSpeakerPose;
        } else {
            m_speakerPose = kRedSpeakerPose;
        }
    }

    // public Command teleopCmd(DoubleSupplier power) {
    // return run(() -> {
    // double powerVal = MathUtil.applyDeadband(power.getAsDouble(), 0.1);
    // m_targetAngle += powerVal * 1.2;
    // m_targetAngle = MathUtil.clamp(m_targetAngle, 30, 80);
    // if (powerVal == 0) {
    // m_aim.setControl(m_holdRequest.withPosition(m_targetAngle / 360.0));
    // } else {
    // m_aim.setControl(m_request.withPosition(m_targetAngle / 360.0));
    // }
    // SmartDashboard.putNumber("aim", m_aim.get());
    // SmartDashboard.putNumber("aim position",
    // m_aim.getPosition().getValueAsDouble());
    // });
    // }

    public Command goTo90() {
        return runOnce(() -> {
            m_targetAngle = 90;
            m_aim.setControl(m_request.withPosition(m_targetAngle / 360.0));
        });
    }

    public Command goTo30() {
        return runOnce(() -> {
            m_targetAngle = 30;
            m_aim.setControl(m_request.withPosition(m_targetAngle / 360.0));
        });
    }

    public Command aimAtSpeaker() {
        var getAngleCmd = run(() -> {
            var translation = m_robotPoseSupplier.get().getTranslation();
            var poseToSpeaker = m_speakerPose.minus(translation);
            m_targetAngle = Math.atan((poseToSpeaker.getZ()) / (poseToSpeaker.getX()));
        });
        var toAngleCmd = toAngle(Math.toDegrees(m_targetAngle));

        return Commands.repeatingSequence(
            getAngleCmd,
            toAngleCmd);
    }

    public void simulationPeriodic() {
        m_talonFxSim.setSupplyVoltage(12);
        var volts = m_talonFxSim.getMotorVoltage();
        m_aimSim.setInputVoltage(volts);
        m_aimSim.update(0.020);
        var angle = Units.radiansToDegrees(m_aimSim.getAngleRads());
        m_aim2d.setAngle(angle); // TODO: make this render correctly with real robot too
        m_talonFxSim.setRawRotorPosition((angle / 360.0) * kGearRatio);
        m_talonFxSim.setRotorVelocity(Units.radiansToRotations(m_aimSim.getVelocityRadPerSec()) * kGearRatio);

        SmartDashboard.putNumber("sim voltage", volts);
        SmartDashboard.putNumber("sim velocity", m_aimSim.getVelocityRadPerSec());
        SmartDashboard.putNumber("sim angle", angle);
        SmartDashboard.putNumber("target", m_targetAngle);
    }
}
