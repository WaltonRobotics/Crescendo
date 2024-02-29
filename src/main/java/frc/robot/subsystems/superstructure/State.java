package frc.robot.subsystems.superstructure;

public enum State {
    READY(0),
    INTAKE(1),
    A1(2),
    // A2,
    CONVEY(3),
    // B1,
    B2(4),
    // C0,
    C1(5),
    C2(6),
    SHOOT_OK(7),
    SHOOT(8),
    SHOOTING(9),
    D1(11),
    D2(11);

    private int m_id;

    private State(int id) {
        m_id = id;
    }

    public int getId() {
        return m_id;
    }
}
