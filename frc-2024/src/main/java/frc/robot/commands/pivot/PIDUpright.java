// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PIDUpright extends Command {
  private static final class Config{
    public static final double kSetpoint = -0.25;
    public static final double kDeadband = 0.005;
    public static final double kP = 2.5;
    public static final double kI = 0.20;
    public static final double kD = 0.15;
  }

  private Pivot m_pivot;
  private PIDController m_pid = new PIDController(Config.kP, Config.kI, Config.kD);
  private double m_speed;

  /** Creates a new PIDForward. */
  public PIDUpright(Pivot pivot) {
    m_pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivot.setSpeed(0);
  }

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
   m_speed = m_pid.calculate(m_pivot.getEncoderPosition(), Config.kSetpoint);

   if (!(Math.abs(m_pivot.getEncoderPosition() - Config.kSetpoint)<= Config.kDeadband)){
     SmartDashboard.putNumber("PID value", m_speed);
     SmartDashboard.putNumber("PID Error", m_pivot.getEncoderPosition() - Config.kSetpoint);
     m_pivot.setSpeed(m_speed);
   }
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.setSpeed(0);
    // System.out.println("Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(m_pivot.getEncoderPosition() - Config.kSetpoint)<= Config.kDeadband;
    return false;
  }
}
