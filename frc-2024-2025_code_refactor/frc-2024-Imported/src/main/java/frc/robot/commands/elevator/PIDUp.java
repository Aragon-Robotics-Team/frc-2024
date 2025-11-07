// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class PIDUp extends Command {

  private static final class Config{
    public static final double kSetpoint = -1.40;
    public static final double kDeadband = 0.05;
    public static final double kP = 0.15;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  private Elevator m_elevator;
  private PIDController m_pid = new PIDController(Config.kP, Config.kI, Config.kD);
  private double m_speed;

  /** Creates a new PIDElevatorTest. */
  public PIDUp(Elevator elevator) {
    m_elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_speed = m_pid.calculate(m_elevator.getEncoderPosition(), Config.kSetpoint);
    System.out.println("I'm running");

    if (!(Math.abs(m_elevator.getEncoderPosition() - Config.kSetpoint)<= Config.kDeadband)){
      System.out.println("I'm running in if-else loop");
      System.out.println(m_speed);
      SmartDashboard.putNumber("PID value", m_speed);
      m_elevator.setSpeed(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_elevator.getEncoderPosition() - Config.kSetpoint)<= Config.kDeadband);
  }
}
