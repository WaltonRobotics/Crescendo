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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CtreConfigs;
import frc.util.AllianceFlipUtil;

import static frc.robot.Constants.FieldK.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.kCanbus;
import static frc.robot.Constants.AimK.*;
import static frc.robot.Constants.FieldK.SpeakerK.*;
import static frc.robot.Constants.RobotK.kSimInterval;
import static frc.robot.Robot.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Aim extends SubsystemBase {
    private final Supplier<Pose3d> m_robotPoseSupplier;

    private final TalonFX m_motor = new TalonFX(kAimId, kCanbus);
    private final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    private final DCMotor m_aimGearbox = DCMotor.getFalcon500(1);
    private final SingleJointedArmSim m_aimSim = new SingleJointedArmSim(
        m_aimGearbox, kGearRatio, 0.761, Units.inchesToMeters(19.75),
        kMinAngle.in(Radians), kMaxAngle.in(Radians), true, Units.degreesToRadians(0));

    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_aimPivot = m_mech2d.getRoot("aimPivot", 30, 30);
    private final MechanismLigament2d m_aim2d = m_aimPivot.append(
        new MechanismLigament2d(
            "Aim2d",
            30,
            Units.radiansToDegrees(m_aimSim.getAngleRads()),
            10,
            new Color8Bit(Color.kHotPink)));

    private Measure<Angle> m_targetAngle;
    private Translation3d m_ampPose;

    private final DigitalInput m_home = new DigitalInput(kHomeSwitch);
    private final Trigger m_homeTrigger = new Trigger(m_home::get).negate();

    private boolean m_isCoast;

    // TODO check
    // private final Trigger m_atStart = new Trigger(
    // () -> m_motor.getPosition().getValueAsDouble() ==
    // Units.degreesToRotations(40));

    public Aim(Supplier<Pose3d> robotPoseSupplier) {
        m_robotPoseSupplier = robotPoseSupplier;
        SmartDashboard.putBoolean("shooterCoast", m_isCoast);
        CtreConfigs configs = CtreConfigs.get();
        m_motor.getConfigurator().apply(configs.m_aimConfigs);

        m_targetAngle = Degrees.of(180 - 0);

        SmartDashboard.putData("Mech2d", m_mech2d);

        // TODO check this value
        m_homeTrigger.onTrue(Commands.runOnce(() -> m_motor.setPosition(kInitAngle.in(Rotations)))
            .ignoringDisable(true));
        // m_atStart.onTrue(Commands.runOnce(() ->
        // m_motor.setNeutralMode(NeutralModeValue.Brake))
        // .ignoringDisable(true));

    }

    private Command toAngle(Measure<Angle> angle) {
        return run(() -> {
            m_motor.setControl(m_request.withPosition(angle.in(Rotations)));
        });
    }

    public void setCoast(boolean coast) {
        m_motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    public Command teleop(DoubleSupplier power) {
        return run(() -> {
            double powerVal = MathUtil.applyDeadband(power.getAsDouble(), 0.1);
            m_targetAngle = m_targetAngle.plus(Degrees.of(powerVal * 1.2));
            m_targetAngle = Degrees
                .of(MathUtil.clamp(m_targetAngle.magnitude(), kMinAngle.magnitude(), kMaxAngle.magnitude()));

            m_motor.setControl(m_request.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public Command runMotor() {
        return runEnd(() -> m_motor.set(0.25), () -> m_motor.set(0));
    }

    public Command aim() {
        return setAimTarget().andThen(toTarget()).repeatedly();
    }

    public Command stop() {
        return run(() -> m_motor.set(0));
    }

    public Command goTo90() {
        var setTargetCmd = runOnce(() -> m_targetAngle = Degrees.of(90));
        return setTargetCmd.andThen(toTarget());
    }

    public Command goTo30() {
        var setTargetCmd = runOnce(() -> m_targetAngle = Degrees.of(30));
        return setTargetCmd.andThen(toTarget());
    }

    public Command toTarget() {
        return toAngle(m_targetAngle);
    }

    public Command beAt90() {
        return runOnce(() -> m_motor.setPosition(Units.degreesToRotations(90)));
    }

    /**
     * @return a command that changes the target angle of the shooter based on where it is relative to the speaker 
     */
    public Command setAimTarget() {
        var translation = AllianceFlipUtil.apply(m_robotPoseSupplier.get().getTranslation());
        var poseToSpeaker = speakerPose.plus(translation);
        return runOnce(() -> {
            m_targetAngle = Radians.of(Math.atan((poseToSpeaker.getZ()) / (poseToSpeaker.getX())));
            m_targetAngle = Degrees
                .of(MathUtil.clamp(m_targetAngle.in(Degrees), kMinAngle.magnitude(), kMaxAngle.magnitude()));
        });
    }

    /**
     * if the robot is to the right of the speaker center, aim at the left corner
     * of the speaker and vice versa
     * 
     * @return a command that changes the target speaker pose based on the robot's position
     */
    public Command changeSpeakerTarget() {
        // TODO check math
        var trans = m_robotPoseSupplier.get().getTranslation();
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            if (trans.getY() > (kRedCenterOpening.getY() + kAimOffset.baseUnitMagnitude())) {
                return runOnce(() -> speakerPose = AllianceFlipUtil.apply(kTopRight));
            } else if (trans.getY() < (kRedCenterOpening.getY() - kAimOffset.baseUnitMagnitude())) {
                return runOnce(() -> speakerPose = AllianceFlipUtil.apply(kTopLeft));
            } else {
                return runOnce(() -> speakerPose = kRedCenterOpening);
            }
        } else if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            if (trans.getY() < (kBlueCenterOpening.getY() + kAimOffset.baseUnitMagnitude())) {
                return runOnce(() -> speakerPose = kTopRight);
            } else if (trans.getY() > (kBlueCenterOpening.getY() - kAimOffset.baseUnitMagnitude())) {
                return runOnce(() -> speakerPose = kTopLeft);
            } else {
                return runOnce(() -> speakerPose = kBlueCenterOpening);
            }
        }

        return Commands.none();
    }

    public void clearTarget() {
        m_targetAngle = Rotations.of(m_motor.getPosition().getValueAsDouble());
    }

    public void aimAtAmp() {
        var translation = m_robotPoseSupplier.get().getTranslation();
        var poseToAmp = m_ampPose.minus(translation);
        m_targetAngle = Radians.of(Math.atan((poseToAmp.getZ()) / (poseToAmp.getX())));
        m_targetAngle = Degrees
            .of(MathUtil.clamp(m_targetAngle.in(Degrees), kMinAngle.magnitude(), kMaxAngle.magnitude()));
    }

    /**
     * @return a command that changes the angle of the shooter near the stage to avoid hitting it
     */
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("aimSpeed", m_motor.get());
        SmartDashboard.putNumber("aimPos",
            Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble()));

        boolean dashCoast = SmartDashboard.getBoolean("shooterCoast", false);
        if (dashCoast != m_isCoast) {
            m_isCoast = dashCoast;
            setCoast(m_isCoast);
        }
    }

    @Override
    public void simulationPeriodic() {
        m_motor.getSimState().setSupplyVoltage(12);
        var volts = m_motor.getSimState().getMotorVoltage();
        m_aimSim.setInputVoltage(volts);

        double angle = Units.radiansToRotations(m_aimSim.getAngleRads());
        double velocity = Units.radiansToRotations(m_aimSim.getVelocityRadPerSec());
        m_motor.getSimState().setRawRotorPosition(angle);
        m_motor.getSimState().setRotorVelocity(velocity);

        m_aim2d.setAngle(Units.rotationsToDegrees(
            m_motor.getPosition().getValueAsDouble())); // TODO: make this render correctly with the real robot too

        SmartDashboard.putNumber("simVoltage", volts);
        SmartDashboard.putNumber("simVelo", m_aimSim.getVelocityRadPerSec());
        SmartDashboard.putNumber("simAngle", angle);
        SmartDashboard.putNumber("targetAngle", m_targetAngle.in(Degrees));

        m_aimSim.update(kSimInterval);
    }
}
