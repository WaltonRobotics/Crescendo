package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trap extends SubsystemBase {
    private final Servo m_servo = new Servo(0);

    public Trap() {}

    public Command deploy() {
        return runOnce(() -> m_servo.setPosition(0));
    } // TODO make it stop going back to 0 forever
}
