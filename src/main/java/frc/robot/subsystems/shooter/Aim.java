package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldK;
import frc.robot.Constants.RobotK;
import frc.robot.Constants.AimK.AimConfigs;
import frc.robot.Vision.VisionMeasurement3d;
import frc.util.AllianceFlipUtil;
import frc.util.WaltRangeChecker;
import frc.util.logging.LoggedTunableNumber;
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.*;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.kCanbus;
import static frc.robot.Constants.AimK.*;
import static frc.robot.Constants.AimK.AimConfigs.*;
import static frc.robot.Constants.RobotK.kSimInterval;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Aim extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(kAimId, kCanbus);
    private final CANcoder m_cancoder = new CANcoder(15, kCanbus);
    private final DigitalInput m_coastSwitch = new DigitalInput(kCoastSwitchId);

    private final Trigger trg_coastSwitch = new Trigger(m_coastSwitch::get);
    private final Trigger trg_autonAngleOk = new Trigger(() -> 
        MathUtil.isNear(kSubwooferAngle.plus(Degrees.of(5)).in(Rotations), m_motor.getPosition().getValueAsDouble(), Units.degreesToRotations(1))
        && RobotK.kStopCoast);

    public final DynamicMotionMagicVoltage m_dynamicRequest = new DynamicMotionMagicVoltage(0, 20, 40, 200);
    private final CoastOut m_coastRequest = new CoastOut();
    private final StaticBrake m_brakeRequest = new StaticBrake();

    private final LoggedTunableNumber m_tunableTest = new LoggedTunableNumber("tunableTest");
    private double m_tunableNumber = 0;
    private final DoubleLogger log_tunableTest = WaltLogger.logDouble("Test", "tunableTest");

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

    private final LinearFilter m_filter = LinearFilter.singlePoleIIR(0.06, 0.02);

    private double m_pitchToSpeaker = 0;

    private boolean m_isCoast;
    private boolean m_usingVision = false;

    private Translation3d m_centerPos;


    private final DoubleLogger log_targetAngle = WaltLogger.logDouble(kDbTabName, "targetAngle");
    private final DoubleLogger log_motorSpeed = WaltLogger.logDouble(kDbTabName, "motorSpeed");
    private final DoubleLogger log_motorPos = WaltLogger.logDouble(kDbTabName, "motorPos");
    private final DoubleLogger log_cancoderPos = WaltLogger.logDouble(kDbTabName, "cancoderPos");

    private final DoubleLogger log_error = WaltLogger.logDouble(kDbTabName, "error");
    private final DoubleLogger log_reference = WaltLogger.logDouble(kDbTabName, "reference");
    private final DoubleLogger log_output = WaltLogger.logDouble(kDbTabName, "output");
    private final DoubleLogger log_ff = WaltLogger.logDouble(kDbTabName, "feedforward");

    private final DoubleLogger log_statorCurrent = WaltLogger.logDouble(kDbTabName, "statorCurrent");
    private final DoubleLogger log_supplyCurrent = WaltLogger.logDouble(kDbTabName, "supplyCurrent");
    private final DoubleLogger log_tqCurrent = WaltLogger.logDouble(kDbTabName, "torqueCurrent");

    private final DoubleLogger log_simVoltage = WaltLogger.logDouble(kDbTabName + "/Sim", "motorVoltage");
    private final DoubleLogger log_simVelo = WaltLogger.logDouble(kDbTabName + "/Sim", "motorVelo");
    private final DoubleLogger log_simAngle = WaltLogger.logDouble(kDbTabName + "/Sim", "curAngle");
    private final DoubleLogger log_simTarget = WaltLogger.logDouble(kDbTabName + "/Sim", "targetAngle");

    private final Pose3dLogger log_pivotPos = WaltLogger.logPose3d(kDbTabName, "pivotPos");
    private final Pose3dLogger log_speakerPos = WaltLogger.logPose3d(kDbTabName, "speakerPos");
    private final DoubleLogger log_desiredPitch = WaltLogger.logDouble(kDbTabName, "desiredPitch");

    private final DoubleLogger log_zDist = WaltLogger.logDouble(kDbTabName, "zDist");
    private final DoubleLogger log_xDist = WaltLogger.logDouble(kDbTabName, "xDist");
    private final DoubleLogger log_measTimer = WaltLogger.logDouble(kDbTabName, "measurementTimer");

    private final BooleanLogger log_coastSwitch = WaltLogger.logBoolean(kDbTabName, "coastSwitch");
    private final BooleanLogger log_autonAngleOk = WaltLogger.logBoolean(kDbTabName, "autonAngleOk");

    private final GenericEntry nte_isCoast;

    private final Measure<Angle> kAngleAllowedError = Degrees.of(0.6);
    private final Measure<Angle> kAmpAngleAllowedError = Degrees.of(0.75);

    private final Timer m_targetTimer = new Timer();
    public final Timer m_measurementTimer = new Timer();

    private final VoltageOut m_voltage = new VoltageOut(0);
    private final SysIdRoutine m_sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Second),
            Volts.of(1.25),
            Seconds.of(15),
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
            m_motor.setControl(m_voltage.withOutput(volts.in(Volts)));
        }, null, this));

    public Aim() {
        m_motor.getConfigurator().apply(motorConfig);
        m_cancoder.getConfigurator().apply(cancoderConfig);

        SmartDashboard.putData("Mech2d", m_mech2d);

        if (Utils.isSimulation()) {
            // In simulation, make sure the CANcoder starts at the correct position
            m_cancoder.setPosition(Units.radiansToRotations(m_aimSim.getAngleRads()) * 1.69);
        }

        nte_isCoast = Shuffleboard.getTab(kDbTabName)
            .add("isCoast", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

        configureCoastTrigger();

        log_tunableTest.accept(0.0);

        WaltRangeChecker.addDoubleChecker("DesiredPitch", () -> m_pitchToSpeaker, 0, kSubwooferAngle.in(Radians), 1, false);

        m_measurementTimer.reset();
    }

    private void determineMotionMagicValues(boolean vision) {
        if (vision && MathUtil.isNear(m_targetAngle.in(Rotations), m_motor.getPosition().getValueAsDouble(), Units.degreesToRotations(2))) {
            m_dynamicRequest.Velocity = 0.1;
            m_dynamicRequest.Acceleration = 0.5;
            m_dynamicRequest.Jerk = 7.5;
            m_dynamicRequest.Slot = 0;
        } else if (m_targetAngle.lt(Rotations.of(m_motor.getPosition().getValueAsDouble())) && m_motor.getPosition().getValueAsDouble() <= 0.2) {
            m_dynamicRequest.Velocity = 0.3;
            m_dynamicRequest.Acceleration = 0.75;
            m_dynamicRequest.Jerk = 0;
            m_dynamicRequest.Slot = 0;
        } else {
            m_dynamicRequest.Velocity = 0.3 * 1.25;
            m_dynamicRequest.Acceleration = 1;
            m_dynamicRequest.Jerk = 0;
            m_dynamicRequest.Slot = 0;
        }
    }

    public Measure<Angle> getAngle() {
        return Rotations.of(m_motor.getPosition().getValueAsDouble());
    }

    private double getDegrees() {
        return Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble()) + 28;
    }

    public BooleanSupplier aimFinished() {
        return () -> {
            if ((m_targetAngle.in(Degrees) == 0 || m_targetAngle.in(Degrees) == 4) && !DriverStation.isAutonomous() && !m_usingVision) {
                return false;
            }
            var error = Rotations.of(Math.abs(m_targetAngle.in(Rotations) - m_motor.getPosition().getValueAsDouble()));
            log_error.accept(error.in(Degrees));

            if (m_targetAngle.baseUnitMagnitude() == kAmpAngle.baseUnitMagnitude()) {
                return error.lte(kAmpAngleAllowedError);
            }

            return error.lte(kAngleAllowedError);
        };
    }

    public BooleanSupplier aimFinished(double tolerance) {
        return () -> {
            var error = Rotations.of(Math.abs(m_targetAngle.in(Rotations) - m_motor.getPosition().getValueAsDouble()));
            log_error.accept(error.in(Degrees));

            return error.lte(m_targetAngle.times(tolerance));
        };
    }

    public Command coastOut() {
        return runOnce(() -> m_motor.setControl(m_coastRequest));
    }

    private void sendAngleRequestToMotor(boolean vision) {
        var target = m_targetAngle.in(Degrees);
        var safeAngle = MathUtil.clamp(target, 0, vision ? kSubwooferAngle.in(Degrees) : 120);
        m_targetAngle = Degrees.of(safeAngle);
        var ff = Math.cos(Units.degreesToRadians(getDegrees())) * kG;
        m_motor.setControl(m_dynamicRequest
            .withPosition(m_targetAngle.in(Rotations))
            .withFeedForward(ff));
    }

    public Command increaseAngle() {
        return Commands.runOnce(() -> {
            m_targetAngle = m_targetAngle.plus(Degrees.of(0.5));
            sendAngleRequestToMotor(false);
        });
    }

    public Command decreaseAngle() {
        return Commands.runOnce(() -> {
            m_targetAngle = m_targetAngle.minus(Degrees.of(0.5));
            sendAngleRequestToMotor(false);
        });
    }

    public Command amp() {
        return runOnce(() -> {
            m_targetAngle = kAmpAngle;
            sendAngleRequestToMotor(false);
        });
    }

    public double getTargetAngle() {
        return m_targetAngle.in(Degrees);
    }

    public Command toAngleUntilAt(Measure<Angle> angle, Measure<Angle> tolerance) {
        return toAngleUntilAt(() -> angle, tolerance);
    }

    public Command toAngleUntilAt(Measure<Angle> angle) {
        return toAngleUntilAt(() -> angle, Degrees.of(0.5)).withName("ToAngleUntilAt"); // TODO unmagify i'm lazy rn
    }

    public Command aim() {
        return runEnd(() -> {
            m_usingVision = true;
            m_targetAngle = Radians.of(m_pitchToSpeaker);
            sendAngleRequestToMotor(true);
        }, () -> {
            m_usingVision = false;
            m_motor.setControl(m_brakeRequest);
        }).withName("AimWithVision");
    }

    public Command toAngleUntilAt(Supplier<Measure<Angle>> angle, Measure<Angle> tolerance) {
       return toAngleUntilAt(angle, tolerance, false);
    }

    public Command toAngleUntilAt(Supplier<Measure<Angle>> angle, Measure<Angle> tolerance, boolean amp) {
        Runnable goThere = () -> {
            m_targetAngle = angle.get();
             if(amp) {
                m_dynamicRequest.Acceleration = 6;
             }
            sendAngleRequestToMotor(false);
        };
        BooleanSupplier isFinished = () -> {
            var error = Rotations.of(Math.abs(m_targetAngle.in(Rotations) - m_motor.getPosition().getValueAsDouble()));
            log_error.accept(error.in(Degrees));

            boolean imThere = error.lte(tolerance);
            if (imThere) {
                m_targetTimer.stop();
                System.out.println("[AIM] Reached target in " + m_targetTimer.get() + " seconds");
                m_targetTimer.reset();
            }
            return imThere;
        };

        return new FunctionalCommand(() -> m_targetTimer.start(), goThere, (intr) -> {}, isFinished, this)
            .withName("AimToAngleUntilAt_+-" + tolerance.in(Degrees));
    }

    public Command intakeAngleNearCmd() {
        return toAngleUntilAt(Degrees.of(4), Degrees.of(10)).withName("AimToIntakeAngleNear");
    }

    public Command hardStop() {
        return Commands.parallel(
            Commands.print("hard stop"), 
            toAngleUntilAt(Degrees.of(0))
        );
    }

    public void setCoast(boolean coast) {
        m_isCoast = coast;
        m_motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        System.out.println("[ARM] SetCoast: " + coast);
    }

    public Command rezero() {
        return Commands.runOnce(() -> {
            var offset = AimConfigs.cancoderConfig.MagnetSensor.MagnetOffset;
            var zero = m_cancoder.getAbsolutePosition().getValueAsDouble() - offset;
            m_cancoder.getConfigurator().apply(
                AimConfigs.cancoderConfig.withMagnetSensor(
                    AimConfigs.cancoderConfig.MagnetSensor.withMagnetOffset(-zero)
                )
            );
            System.out.println("new offset: " + -zero);
        });
    }

    public void configureCoastTrigger() {
        var coastMgmtCmd = startEnd(() -> {
            setCoast(true);
        }, () -> {
            setCoast(false);
        }).until(trg_autonAngleOk).ignoringDisable(true);

        trg_coastSwitch.and(RobotModeTriggers.disabled()).whileTrue(coastMgmtCmd);
    }

    public Command setTarget(Measure<Angle> target) {
        return Commands.runOnce(() -> {
            m_targetAngle = target;
        });
    }

    public void calculatePitchToSpeaker(VisionMeasurement3d meas) {
        m_measurementTimer.restart();

        var pose = meas.estimate().estimatedPose;

        var pivotPose = pose.transformBy(kOriginToPivot);
        var pivotTrans = pivotPose.getTranslation();

        Translation3d speakerPos;
        m_centerPos = AllianceFlipUtil.apply(FieldK.SpeakerK.kBlueCenterOpening.minus(new Translation3d(0, 0, Units.inchesToMeters(2))));
        
        if (MathUtil.isNear(m_centerPos.getY(), pose.getY(), 1)) {
            speakerPos = m_centerPos;
            if ((AllianceFlipUtil.apply(pose.getX()) > 3.5)) {
                speakerPos = speakerPos.minus(new Translation3d(0, 0, Units.inchesToMeters(5)));
            }
        } else if (pose.getY() < m_centerPos.getY()) {
            speakerPos = AllianceFlipUtil.apply(FieldK.SpeakerK.kTopRight);
        } else {
            speakerPos = AllianceFlipUtil.apply(FieldK.SpeakerK.kTopLeft);
        }

        var distance = speakerPos.minus(pivotTrans);
        log_speakerPos.accept(speakerPos);
        log_pivotPos.accept(pivotPose);

        log_zDist.accept(Units.metersToInches(distance.getZ()));
        log_xDist.accept(Units.metersToInches(distance.getX()));

        m_pitchToSpeaker = (m_filter.calculate(Math.atan2(distance.getZ(), Math.hypot(distance.getX(), distance.getY()))) - Units.degreesToRadians(28));
        log_desiredPitch.accept(Units.radiansToDegrees(m_pitchToSpeaker));
    }

    @Override
    public void periodic() {
        determineMotionMagicValues(m_usingVision);
        log_measTimer.accept(m_measurementTimer.get());

        log_autonAngleOk.accept(trg_autonAngleOk.getAsBoolean());
        log_motorSpeed.accept(m_motor.get());
        log_motorPos.accept(Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble()));
        log_targetAngle.accept(getTargetAngle());
        log_cancoderPos.accept(Units.rotationsToDegrees(m_cancoder.getPosition().getValueAsDouble()));

        log_reference.accept(Units.rotationsToDegrees(m_motor.getClosedLoopReference().getValueAsDouble()));
        log_output.accept(m_motor.getClosedLoopOutput().getValueAsDouble());
        log_ff.accept(m_motor.getClosedLoopFeedForward().getValueAsDouble());

        log_statorCurrent.accept(m_motor.getStatorCurrent().getValueAsDouble());
        log_supplyCurrent.accept(m_motor.getSupplyCurrent().getValueAsDouble());
        log_tqCurrent.accept(m_motor.getTorqueCurrent().getValueAsDouble());

        boolean dashCoast = nte_isCoast.getBoolean(false);
        if (dashCoast != m_isCoast && !trg_coastSwitch.getAsBoolean()) {
            m_isCoast = dashCoast;
            setCoast(m_isCoast);
        }

        if (m_tunableNumber != m_tunableTest.get()) {
            m_tunableNumber = m_tunableTest.get();
            log_tunableTest.accept(m_tunableNumber);
        }

        log_coastSwitch.accept(trg_coastSwitch);
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysId.dynamic(direction);
    }
}
