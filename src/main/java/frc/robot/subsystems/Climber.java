// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Climber extends SubsystemBase {
//     private final PWMSparkMax m_right = new PWMSparkMax(3); 
//     private final PWMSparkMax m_left = new PWMSparkMax(8);

//     public Climber() {}
    
//     public Command climb() {
//         return runEnd(() -> {
//             m_right.set(1);
//             m_left.set(1);
//         }, 
//         () -> {
//             m_right.set(0);
//             m_left.set(0);
//         });
//     }

//     public Command moveLeft() {
//         return runEnd(() -> {
//             m_left.set(1);
//         }, 
//         () -> {
//             m_left.set(0);
//         });
//     }

//     public Command moveRight() {
//         return runEnd(() -> {
//             m_right.set(1);
//         }, 
//         () -> {
//             m_right.set(0);
//         });
//     }

//     public Command release() {
//         return runEnd(() -> {
//             m_right.set(-1);
//             m_left.set(-1);
//         }, 
//         () -> {
//             m_right.set(0);
//             m_left.set(0);
//         });
//     }
// }  
