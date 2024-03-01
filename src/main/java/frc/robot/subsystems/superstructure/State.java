package frc.robot.subsystems.superstructure;

public enum State {
    IDLE(0),
    INTAKE(1),
    INTAKE_TOP_RISING(2),
    INTAKE_TOP_FALLING(3),
    // B1,
    INTAKE_BOT_RISING(4),
    INTAKE_BOT_FALLING(5),
    // C0,
    ROLLER_BEAM_RETRACT(6), // Note hit top beam
    // ROLLER_BEAM_RETRACTED(7), // Note back from top beam
    SHOOT_OK(8),
    SHOT_SPINUP(9),
    SHOOTING(10),
    D1(11),
    D2(12);

    public final int idx;

    private State(int idx) {
        this.idx = idx;
    }
}
