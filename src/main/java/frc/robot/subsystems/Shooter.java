package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Shooter.*;

public class Shooter extends SubsystemBase {
    private final TalonFX m_right = new TalonFX(kRightId);
    private final TalonFX m_left = new TalonFX(kLeftId);
    private final Follower m_follower = new Follower(m_right.getDeviceID(), true);

    private final CANSparkMax m_conveyor = new CANSparkMax(kConveyorId, MotorType.kBrushless);

    private final CANSparkMax m_aim = new CANSparkMax(kConveyorId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_aim.getEncoder();
    private final PIDController m_aimController = new PIDController(kPAim, 0, 0);

    public Shooter() {
        m_left.setControl(m_follower);
        m_encoder.setPositionConversionFactor(kConversion);
    }

    public Command shoot() {
        var shooterCmd = run(() -> m_right.set(1));
        var conveyorCmd = run(() -> m_conveyor.set(0.5));

        return Commands.parallel(shooterCmd, conveyorCmd);
    }

    public Command toAngle(double degrees) {
        var setupCmd = runOnce(() -> m_aimController.setSetpoint(degrees));
        var moveCmd = run(() -> {
            double effort = m_aimController.calculate(m_encoder.getPosition());
            m_aim.setVoltage(effort);
        })
            .until(() -> m_aimController.atSetpoint())
            .finallyDo(() -> m_aim.set(0));

        return Commands.sequence(setupCmd, moveCmd);
    }

    public Command aimAtSpeaker() {
        /*
         * angle for shooter = arctan((z of target - z of top of shooter) / (x of target
         * - x of top of shooter))
         * find distance to apriltag and then add the translation of shooter target to
         * get distance to target
         * set angle to correct angle
         */
        return Commands.none();
    }
}
