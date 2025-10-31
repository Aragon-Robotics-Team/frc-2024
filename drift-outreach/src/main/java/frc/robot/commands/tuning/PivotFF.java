// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotFF extends Command {
  private Pivot m_pivot;
  /** Creates a new PivotFF. */
  public PivotFF(Pivot pivot) {
    m_pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
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
    if (s == 0.0) {
      SmartDashboard.putNumber("FFTest/speed", 0.0);
    }
    m_pivot.setSpeed(s);

    SmartDashboard.putNumber("FFTest/CalculatedKV", s / m_pivot.getVelocity());
    SmartDashboard.putNumber("FFTest/RPS", m_pivot.getVelocity());
    SmartDashboard.putNumber("FFTest/Rots", m_pivot.getEncoderPosition());
    SmartDashboard.putNumber("FFTest/MotorRots", m_pivot.getMotorSensorPosition());
    SmartDashboard.putBoolean("FFTest/ForwardSwitch", m_pivot.ifForwardOpen());
    SmartDashboard.putBoolean("FFTest/BackwardSwitch", m_pivot.ifBackwardOpen());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
