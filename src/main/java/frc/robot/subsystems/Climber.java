package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.Climber.*;

public class Climber extends SubsystemBase {
    private final TalonFX m_right = new TalonFX(kRightId);
    private final TalonFX m_left = new TalonFX(kLeftId);
    private final Follower m_follower = new Follower(m_right.getDeviceID(), true);

    // TODO: Change values of PIDController :-)
    private final PIDController m_controller = new PIDController(0, 0, 0);
    
    public Climber() {
        m_left.setControl(m_follower);
    }

    private double getHeight() {
        double pos = m_left.getRotorPosition().getValueAsDouble();
        return pos * kMetersPerRotation;
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
        toHeight(kMaxHeight.in(Meters));
    }

    // figure out minHeight value (I assume its 0?)
    public void toMinHeight() {
        toHeight(0);
    }
}
