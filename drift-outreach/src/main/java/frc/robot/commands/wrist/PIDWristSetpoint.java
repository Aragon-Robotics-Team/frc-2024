// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class PIDWristSetpoint extends Command {

    private static final class Config{
    public static final double kDeadband = 0.000;
    public static final double kP = 2.5;
    public static final double kI = 0.1;
    public static final double kD = 0.1;
  }

  private Wrist m_wrist;
  private PIDController m_pid = new PIDController(Config.kP, Config.kI, Config.kD);
  private double m_speed;
  private double m_setpoint;

  /** Creates a new PIDForward. */
  public PIDWristSetpoint(Wrist wrist, double setpoint) {
    m_wrist = wrist;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependenxcies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_speed = m_pid.calculate(m_wrist.getEncoderPosition(), m_setpoint);
    //  if (!(Math.abs(m_wrist.getEncoderPosition() - m_setpoint)<= Config.kDeadband)){
      // // System.out.println("I'm running in if-else loop");
      // System.out.println(m_speed);
      SmartDashboard.putNumber("PID value", m_speed);
      m_wrist.setSpeed(m_speed);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(m_wrist.getEncoderPosition() - m_setpoint)<= Config.kDeadband;
    return false;
  }
}
