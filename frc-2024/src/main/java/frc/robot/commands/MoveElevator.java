// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Elevator;
// import frc.robot.Constants;

// public class MoveElevator extends Command {

//   private double m_endPosition;
//   private Elevator m_elevator;
//   private PIDController m_PID = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);
//   /** Creates a new MoveElevator. */
//   public MoveElevator(double degrees, Elevator elevator) {
//     m_elevator = elevator;
//     m_endPosition = degrees*Constants.ElevatorConstants.RotationsPerInch*Constants.kNeoTicksPerRevolution;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_elevator.setSpeed(m_PID.calculate(m_elevator.getEncoderPosition(), m_endPosition));
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_elevator.setSpeed(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (Math.abs(m_elevator.getEncoderPosition() - m_endPosition) < Constants.ElevatorConstants.kDeadBand);
//   }
// }
