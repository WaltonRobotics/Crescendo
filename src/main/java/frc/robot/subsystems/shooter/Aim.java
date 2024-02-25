package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import frc.robot.Constants.AimK.AimConfigs;
import frc.util.AllianceFlipUtil;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

import static frc.robot.Constants.FieldK.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.kCanbus;
import static frc.robot.Constants.AimK.*;
import static frc.robot.Constants.FieldK.SpeakerK.*;
import static frc.robot.Constants.RobotK.kSimInterval;
import static frc.robot.Robot.*;

import java.util.function.Supplier;

public class Aim extends SubsystemBase {
    private final Supplier<Pose3d> m_robotPoseSupplier;

    private final TalonFX m_motor = new TalonFX(kAimId, kCanbus);
    private final CANcoder m_cancoder = new CANcoder(15, kCanbus);

    private final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    private final CoastOut m_coastRequest = new CoastOut();

    private final DCMotor m_aimGearbox = DCMotor.getFalcon500(1);
    private final SingleJointedArmSim m_aimSim = new SingleJointedArmSim(
        m_aimGearbox, kGearRatio, 0.761, Units.inchesToMeters(19.75),
        kMinAngle.in(Radians), kMaxAngle.in(Radians), true, kInitAngle.in(Radians));

    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_aimPivot = m_mech2d.getRoot("aimPivot", 30, 30);
    private final MechanismLigament2d m_aim2d = m_aimPivot.append(
        new MechanismLigament2d(
            "Aim2d",
            30,
            Units.radiansToDegrees(m_aimSim.getAngleRads()),
            10,
            new Color8Bit(Color.kHotPink)));

    private Measure<Angle> m_targetAngle = Rotations.of(0);
    private Translation3d m_ampPose;

    private final DigitalInput m_home = new DigitalInput(kHomeSwitch);
    private final Trigger m_homeTrigger = new Trigger(m_home::get).negate();

    private boolean m_isCoast;

    private final DoubleLogger log_targetAngle = WaltLogger.logDouble(kDbTabName, "targetAngle");
    private final DoubleLogger log_motorSpeed = WaltLogger.logDouble(kDbTabName, "motorSpeed");
    private final DoubleLogger log_motorPos = WaltLogger.logDouble(kDbTabName, "motorPos");
    private final DoubleLogger log_cancoderPos = WaltLogger.logDouble(kDbTabName, "cancoderPos");

    private final DoubleLogger log_closedLoopError = WaltLogger.logDouble(kDbTabName, "closedLoopError");
    private final DoubleLogger log_reference = WaltLogger.logDouble(kDbTabName, "reference");
    private final DoubleLogger log_output = WaltLogger.logDouble(kDbTabName, "output");

    private final DoubleLogger log_simVoltage = WaltLogger.logDouble(kDbTabName + "/Sim", "motorVoltage");
    private final DoubleLogger log_simVelo = WaltLogger.logDouble(kDbTabName + "/Sim", "motorVelo");
    private final DoubleLogger log_simAngle = WaltLogger.logDouble(kDbTabName + "/Sim", "curAngle");
    private final DoubleLogger log_simTarget = WaltLogger.logDouble(kDbTabName + "/Sim", "targetAngle");

    private final GenericEntry nte_isCoast;

    // private LoggedTunableNumber m_tunableAngle = new
    // LoggedTunableNumber("targetAngle",
    // Units.rotationsToDegrees(m_target) + 22.5);

    // TODO check
    // private final Trigger m_atStart = new Trigger(
    // () -> m_motor.getPosition().getValueAsDouble() ==
    // Units.degreesToRotations(40));

    public Aim(Supplier<Pose3d> robotPoseSupplier) {
        m_robotPoseSupplier = robotPoseSupplier;

        m_motor.getConfigurator().apply(AimConfigs.motorConfig);
        m_cancoder.getConfigurator().apply(AimConfigs.cancoderConfig);

        SmartDashboard.putData("Mech2d", m_mech2d);

        if (Utils.isSimulation()) {
            // In simulation, make sure the CANcoder starts at the correct position
            m_cancoder.setPosition(Units.radiansToRotations(m_aimSim.getAngleRads()) * 1.69);
        }
        m_homeTrigger.onTrue(Commands.runOnce(() -> m_motor.setPosition(kInitAngle.in(Rotations)))
            .ignoringDisable(true));
        // m_atStart.onTrue(Commands.runOnce(() ->
        // m_motor.setNeutralMode(NeutralModeValue.Brake))
        // .ignoringDisable(true));

        nte_isCoast = Shuffleboard.getTab(kDbTabName)
            .add("isCoast", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
    }

    public double getDegrees() {
        return m_motor.getPosition().getValueAsDouble() * 360 + 22.5;
    }

    public Command to90ish() {
        return runOnce(() -> {
            m_targetAngle = Rotations.of(0.275879);
            m_motor.setControl(m_request.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public Command to0() {
        return runOnce(() -> {
            m_targetAngle = Rotations.of(0);
            m_motor.setControl(m_request.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public Command coastOut() {
        return runOnce(() -> m_motor.setControl(m_coastRequest));
    }

    public Command increaseAngle() {
        return runOnce(() -> {
            m_targetAngle = m_targetAngle.plus(Degrees.of(1));
            m_motor.setControl(m_request.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public Command decreaseAngle() {
        return runOnce(() -> {
            m_targetAngle = m_targetAngle.minus(Degrees.of(1));
            m_motor.setControl(m_request.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public Command speakerAngle() {
        return runOnce(() -> {
            m_targetAngle = Rotations.of(0.123535);
            m_motor.setControl(m_request.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public double getTargetAngle() {
        return m_targetAngle.in(Degrees);
    }

    private Command toAngle(Measure<Angle> angle) {
        return runOnce(
            () -> {
                m_motor.setControl(m_request.withPosition(angle.in(Rotations)));
            }); // idk
    }

    public Command intakeMode() {
        return toAngle(Degrees.of(10));
    }

    public void setCoast(boolean coast) {
        m_isCoast = coast;
        m_motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    public Command coastCmd(boolean coast) {
        return runOnce(
            () -> {
                setCoast(coast);
            });
    }

    // public Command teleop(DoubleSupplier power) {
    // return run(
    // () -> {
    // double powerVal = MathUtil.applyDeadband(power.getAsDouble(), 0.1);
    // m_targetAngle = m_targetAngle.plus(Degrees.of(powerVal * 1.2));
    // m_targetAngle = Degrees
    // .of(MathUtil.clamp(m_targetAngle.magnitude(), kMinAngle.magnitude(),
    // kMaxAngle.magnitude()));

    // m_motor.setControl(m_request.withPosition(m_targetAngle.in(Rotations)));
    // });
    // }

    public Command aim() {
        return setAimTarget().andThen(toTarget()).repeatedly();
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
        return runOnce(
            () -> {
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
        log_motorSpeed.accept(m_motor.get());
        log_motorPos.accept(m_motor.getPosition().getValueAsDouble());
        log_targetAngle.accept(getTargetAngle());
        log_cancoderPos.accept(m_cancoder.getPosition().getValueAsDouble());

        log_closedLoopError.accept(m_motor.getClosedLoopError().getValueAsDouble());
        log_reference.accept(m_motor.getClosedLoopReference().getValueAsDouble());
        log_output.accept(m_motor.getClosedLoopOutput().getValueAsDouble());

        // m_target = Units.degreesToRotations(m_tunableAngle.get() - 22.5);

        boolean dashCoast = nte_isCoast.getBoolean(false);
        if (dashCoast != m_isCoast) {
            m_isCoast = dashCoast;
            setCoast(m_isCoast);
            System.out.println("Changing coast from SD: " + m_isCoast);
        }
    }

    @Override
    public void simulationPeriodic() {
        m_motor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        m_cancoder.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        var volts = m_motor.getSimState().getMotorVoltage();
        m_aimSim.setInputVoltage(volts);
        m_aimSim.update(kSimInterval);

        double angle = Units.radiansToRotations(m_aimSim.getAngleRads());
        double velocity = Units.radiansToRotations(m_aimSim.getVelocityRadPerSec());
        // The values sent to the devices must be before gear ratios
        m_motor.getSimState().setRawRotorPosition(angle * kGearRatio);
        m_motor.getSimState().setRotorVelocity(velocity * kGearRatio);
        m_cancoder.getSimState().setRawPosition(angle * 1.69);
        m_cancoder.getSimState().setVelocity(velocity * 1.69);

        m_aim2d.setAngle(Units.rotationsToDegrees(
            m_motor.getPosition().getValueAsDouble())); // TODO: make this render correctly with the real robot too

        log_simVoltage.accept(volts);
        log_simVelo.accept(m_aimSim.getVelocityRadPerSec());
        log_simAngle.accept(angle);
        log_simTarget.accept(m_targetAngle.in(Degrees));
    }
}
