// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorFF extends Command {
  private Elevator m_elevator;
  /** Creates a new ElevatorFF. */
  public ElevatorFF(Elevator elevator) {
    m_elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("FFTest/speed", 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double s = SmartDashboard.getNumber("FFTest/speed", 0.0);
    m_elevator.setSpeed(s);

    SmartDashboard.putNumber("FFTest/CalculatedKV", s / m_elevator.getVelocity());
    SmartDashboard.putNumber("FFTest/RPM", m_elevator.getVelocity());
    SmartDashboard.putNumber("FFTest/Rots", m_elevator.getEncoderPosition());
    SmartDashboard.putNumber("FFTest/MotorRots", m_elevator.getMotorSensorPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
