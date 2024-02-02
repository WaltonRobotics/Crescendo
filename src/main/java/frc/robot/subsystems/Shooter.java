package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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
import static frc.robot.Constants.ShooterK.*;

import java.util.function.DoubleSupplier;

// TODO separate into two classes
public class Shooter extends SubsystemBase {
    private final TalonFX m_right = new TalonFX(kRightId);
    private final TalonFX m_left = new TalonFX(kLeftId);

    private final CANSparkMax m_conveyor = new CANSparkMax(kConveyorId, MotorType.kBrushless);

    private final TalonFX m_aim = new TalonFX(kAimId);
    private final TalonFXSimState m_talonFxSim = m_aim.getSimState();
    private final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    private final DCMotor m_aimGearbox = DCMotor.getFalcon500(1);
    private final SingleJointedArmSim m_aimSim = new SingleJointedArmSim(
        m_aimGearbox, 0.5, 0.761, 0.5017,
        Math.PI / 6, Units.degreesToRadians(80), true, Math.PI / 6);

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

    public Shooter() {
        ctreConfigs();
        m_targetAngle = 30;
        SmartDashboard.putData("Mech2d", m_mech2d);
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
            // TODO check values and stuff
            m_right.set(1);
            m_left.set(1);
            m_conveyor.set(0.5);
        });
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

    public Command teleopCmd(DoubleSupplier power) {
        return run(() -> {
            double powerVal = MathUtil.applyDeadband(power.getAsDouble(), 0.1);
            m_targetAngle += powerVal * 1.2;
            m_aim.setControl(m_request.withPosition(m_targetAngle / 360.0));
            SmartDashboard.putNumber("aim", m_aim.get());
            SmartDashboard.putNumber("aim position", m_aim.getPosition().getValueAsDouble());
        });
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

    public void simulationPeriodic() {
        m_talonFxSim.setSupplyVoltage(12);
        var volts = m_talonFxSim.getMotorVoltage();
        m_aimSim.setInput(volts);
        SmartDashboard.putNumber("sim voltage", volts);
        m_aimSim.update(0.020);
        var angle = Units.radiansToDegrees(m_aimSim.getAngleRads());
        m_aim2d.setAngle(angle);
        m_talonFxSim.setRawRotorPosition(angle / 360.0);
        m_talonFxSim.setRotorVelocity(
            Units.radiansToRotations(m_aimSim.getVelocityRadPerSec()));
        SmartDashboard.putNumber("sim angle", angle);
        SmartDashboard.putNumber("target", m_targetAngle);
    }
}
