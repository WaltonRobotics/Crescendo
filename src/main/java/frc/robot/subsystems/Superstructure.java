package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.Conveyor;
import frc.robot.subsystems.shooter.Shooter;

public class Superstructure {
    public Aim m_aim;
    public Intake m_intake;
    public Conveyor m_conveyor;
    public Shooter m_shooter;

    public Superstructure(Aim aim, Intake intake, Conveyor conveyor, Shooter shooter) {
        m_aim = aim;
        m_intake = intake;
        m_conveyor = conveyor;
        m_shooter = shooter;
    }

    private Command spinUpWait() {
        return Commands.waitUntil(() -> m_shooter.spinUpFinished());
    }

    public Command intake() {
        var aimCmd = m_aim.intakeMode();
        var intakeCmd = m_intake.run();
        var conveyorCmd = m_conveyor.run(false);

        return Commands.sequence(aimCmd, Commands.race(intakeCmd, conveyorCmd));
    }

    public Command intakeShotCycle() {
        var aimCmd = m_aim.toTarget();
        var intakeCmd = m_intake.outtake();
        var conveyorCmd = m_conveyor.run(false);
        var shooterCmd = m_shooter.shoot();

        return Commands.parallel(
            aimCmd,
            shooterCmd,
            Commands.race(
                intakeCmd,
                Commands.sequence(
                    // TODO make this work
                    Commands.waitUntil(() -> spinUpWait().isFinished() && aimCmd.isFinished()),
                    conveyorCmd)));
    }

    public Command subwoofer() {
        var aimCmd = m_aim.subwoofer();
        var shootCmd = m_shooter.shoot();
        var conveyorCmd = m_conveyor.run(true);

        return Commands.parallel(
            shootCmd,
            aimCmd,
            Commands.sequence(
                // TODO make this work correctly
                Commands.parallel(
                    spinUpWait(),
                    Commands.waitUntil(() -> aimCmd.isFinished())),
                conveyorCmd));
    }

    public Command shoot() { // podium angle: 0.007568 rot
        var shootCmd = m_shooter.shoot();
        var conveyorCmd = m_conveyor.run(true);

        return Commands.parallel(
            Commands.print("shooting"),
            shootCmd,
            Commands.sequence(
                Commands.print("waiting for spinup"),
                spinUpWait(),
                Commands.print("conveying"),
                conveyorCmd,
                Commands.print("done")));
    }

    public Command shootFast() {
        var shootCmd = m_shooter.shootFast();
        var conveyorCmd = m_conveyor.run(true);

        return Commands.parallel(
            shootCmd,
            Commands.sequence(
                // TODO make this work correctly
                spinUpWait(),
                conveyorCmd));
    }

    public Command trapShot() {
        var shootCmd = m_shooter.trapShot();
        var conveyorCmd = m_conveyor.run(true);

        return Commands.parallel(
            shootCmd,
            Commands.sequence(
                // TODO make this work correctly
                spinUpWait(),
                conveyorCmd));
    }

    public Command ampShot() {
        var shootCmd = m_shooter.ampShot();
        var conveyorCmd = m_conveyor.run(true);

        return Commands.parallel(
            shootCmd,
            Commands.sequence(
                // TODO make this work correctly
                spinUpWait(),
                conveyorCmd));
    }

    public Command aimAndShoot() {
        var aimCmd = m_aim.toTarget();
        var conveyorCmd = m_conveyor.run(false);
        var shooterCmd = m_shooter.shoot();

        return Commands.parallel(
            aimCmd,
            shooterCmd,
            Commands.sequence(
                Commands.waitUntil(() -> m_shooter.spinUpFinished() && aimCmd.isFinished()),
                conveyorCmd));
    }

    public Command backwardsRun() {
        var shooterCmd = m_shooter.runBackwards();
        var conveyorCmd = m_conveyor.runBackwards();

        return Commands.parallel(shooterCmd, conveyorCmd);
    }
}
