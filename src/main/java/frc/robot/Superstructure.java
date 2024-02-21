package frc.robot;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.Conveyor;

public class Superstructure {
    public Aim m_aim;
    public Intake m_intake;
    public Conveyor m_conveyor;

    public Superstructure(Aim aim, Intake intake, Conveyor conveyor) {
        m_aim = aim;
        m_intake = intake;
        m_conveyor = conveyor;
    }
}
