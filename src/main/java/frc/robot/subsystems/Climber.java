package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ClimberK.*;
import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {
    private final TalonFX m_right = new TalonFX(kRightId);
    private final TalonFX m_left = new TalonFX(kLeftId);
    private final Follower m_follower = new Follower(m_right.getDeviceID(), true);

    private final DigitalInput m_lowerLimit = new DigitalInput(kLimitSwitchId);
    private final Trigger m_lowerLimitTrigger = new Trigger(m_lowerLimit::get).negate();

    private final PIDController m_controller = new PIDController(kP, 0, 0);

    private double m_targetHeight = 0;

    public Climber() {
        m_left.setControl(m_follower);
    }

    private boolean isFullyRetracted() {
        return !m_lowerLimitTrigger.getAsBoolean();
    }

    private double getHeight() {
        double pos = m_left.getRotorPosition().getValueAsDouble();
        return pos * kMetersPerRotation.baseUnitMagnitude();
    }

    public Command autoHome() {
        return Commands.sequence(
            startEnd(() -> {
                m_right.setVoltage(-2);
            }, () -> {
                m_right.setVoltage(0);
            }).until(m_lowerLimitTrigger));
    }

    public Command teleop(DoubleSupplier power) {
        return run(() -> {
            double dir = Math.signum(power.getAsDouble());
            double output = 0;

            if (isFullyRetracted() && dir == -1) {
                output = 0;
            } else {
                output = MathUtil.applyDeadband(power.getAsDouble(), kStickDeadband);
            }

            m_targetHeight += output * 0.02;
            double effort = m_controller.calculate(getHeight(), m_targetHeight);
            double holdEffort = 0;

            if (output != 0) {
                m_right.setVoltage(effort);
            } else {
                output = MathUtil.applyDeadband(power.getAsDouble(), kStickDeadband);
                m_right.setVoltage(holdEffort);
            }
        })
            .withName("teleop");
    }

    public Command toHeight(double height) {
        var setupCmd = runOnce(() -> m_controller.setSetpoint(height));

        var moveCmd = run(() -> {
            var effort = m_controller.calculate(getHeight());
            m_right.setVoltage(effort);
        });

        return Commands.sequence(setupCmd, moveCmd);
    }

    // figure out maxHeight value
    public void toMaxHeight() {
        toHeight(kMaxHeight.baseUnitMagnitude());
    }
}
