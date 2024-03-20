// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.intakeshooter;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;

// public class IntakeShoot extends Command {

//   private Intake m_intake;
//   private Shooter m_shooter;
//   private Joystick m_joystick;
//   private Timer m_timer = new Timer();
//   private int m_timerCounter;

//   /** Creates a new IntakeShoot. */
//   public IntakeShoot(Intake intake, Shooter shooter, Joystick joystick) {
//     m_intake = intake;
//     m_shooter = shooter;
//     m_joystick = joystick;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_intake, m_shooter);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_intake.setSpeed(0);
//     m_shooter.setSpeed(0);
//     m_timerCounter = 0;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (!m_intake.ifSensorTriggered()){
//       m_intake.setSpeed(m_joystick.getRawAxis(2));
//     }
//     else if (m_intake.ifSensorTriggered()){
//       m_intake.setSpeed(0);
//     }
//     else if (m_intake.ifSensorTriggered() && Math.abs(m_joystick.getRawAxis(3))>0){
//       if (!m_timer.hasElapsed(0.1+0.02*m_timerCounter)){
//         m_intake.setSpeed(-0.5);
//       }
//     }
//     else if (Math.abs(m_joystick.getRawAxis(3))>0){
//       m_shooter.setSpeed(m_joystick.getRawAxis(3));
//       m_intake.setSpeed(m_joystick.getRawAxis(2));
//     }
//    m_timerCounter++;
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_intake.setSpeed(0);
//     m_shooter.setSpeed(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
