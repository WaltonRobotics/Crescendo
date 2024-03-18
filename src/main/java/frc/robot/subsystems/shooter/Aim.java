package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
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
import frc.util.logging.WaltLogger;
import frc.util.logging.WaltLogger.DoubleLogger;

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

    private final Trigger trg_coastSwitch = new Trigger(() -> m_coastSwitch.get());

    private final DynamicMotionMagicVoltage m_dynamicRequest = new DynamicMotionMagicVoltage(0, 20, 40, 200);
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

    private boolean m_isCoast;

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

    private final GenericEntry nte_isCoast;

    private final Measure<Angle> kAngleAllowedError = Degrees.of(0.5);
    private final Timer m_targetTimer = new Timer();

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

        // configureCoastTrigger();
    }

    private void determineMotionMagicValues() {
        if (m_targetAngle.lt(Rotations.of(m_motor.getPosition().getValueAsDouble())) && m_motor.getPosition().getValueAsDouble() <= 0.1) {
            m_dynamicRequest.Velocity = 0.2;
            m_dynamicRequest.Acceleration = 0.3;
            m_dynamicRequest.Jerk = 0;
            m_dynamicRequest.Slot = 0;
        } else {
            m_dynamicRequest.Velocity = 0.3;
            m_dynamicRequest.Acceleration = 1;
            m_dynamicRequest.Jerk = 0;
            m_dynamicRequest.Slot = 0;
        }
    }

    private double getDegrees() {
        return Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble()) + 28;
    }

    public boolean aimFinished() {
        var error = Rotations.of(Math.abs(m_targetAngle.in(Rotations) - m_motor.getPosition().getValueAsDouble()));
        return error.lte(kAngleAllowedError);
    }

    public Command coastOut() {
        return runOnce(() -> m_motor.setControl(m_coastRequest));
    }

    public Command increaseAngle() {
        return Commands.runOnce(() -> {
            m_targetAngle = m_targetAngle.plus(Degrees.of(0.5));
            determineMotionMagicValues();
            m_motor.setControl(m_dynamicRequest.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public Command decreaseAngle() {
        return Commands.runOnce(() -> {
            m_targetAngle = m_targetAngle.minus(Degrees.of(0.5));
            determineMotionMagicValues();
            m_motor.setControl(m_dynamicRequest.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public Command amp() {
        return runOnce(() -> {
            m_targetAngle = kAmpAngle;
            determineMotionMagicValues();
            m_motor.setControl(m_dynamicRequest.withPosition(m_targetAngle.in(Rotations)));
        });
    }

    public double getTargetAngle() {
        return m_targetAngle.in(Degrees);
    }

    public Command toAngleUntilAt(Measure<Angle> angle, Measure<Angle> tolerance) {
        return toAngleUntilAt(() -> angle, tolerance);
    }

    public Command toAngleUntilAt(Measure<Angle> angle) {
        return toAngleUntilAt(() -> angle, Degrees.of(0.25)).withName("ToAngleUntilAt"); // TODO unmagify i'm lazy rn
    }

    public Command toAngleUntilAt(Supplier<Measure<Angle>> angle, Measure<Angle> tolerance) {
        Runnable goThere = () -> {
            m_targetAngle = angle.get();
            determineMotionMagicValues();
            var ff = Math.cos(Units.degreesToRadians(getDegrees())) * kG;
            var request = m_dynamicRequest
                .withPosition(angle.get().in(Rotations))
                .withFeedForward(ff);
            m_motor.setControl(request);
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
        return toAngleUntilAt(Degrees.of(2), Degrees.of(10)).withName("AimToIntakeAngleNear");
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
        trg_coastSwitch.and(RobotModeTriggers.disabled())
            .onTrue(
                runOnce(() -> setCoast(true)).ignoringDisable(true)
            )
            .onFalse(
                runOnce(() -> setCoast(false)).ignoringDisable(true)
            );
    }

    @Override
    public void periodic() {
        log_motorSpeed.accept(m_motor.get());
        log_motorPos.accept(Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble()));
        log_targetAngle.accept(getTargetAngle());
        log_cancoderPos.accept(Units.rotationsToDegrees(m_cancoder.getPosition().getValueAsDouble()));

        log_error.accept(Units.rotationsToDegrees(m_motor.getClosedLoopError().getValueAsDouble()));
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
