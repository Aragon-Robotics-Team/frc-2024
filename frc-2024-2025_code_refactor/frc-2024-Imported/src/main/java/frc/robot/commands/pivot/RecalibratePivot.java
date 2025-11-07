// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class RecalibratePivot extends Command {

  private Pivot m_pivot;

  /** Creates a new RecalibratePivot. */
  public RecalibratePivot(Pivot pivot) {
    m_pivot = pivot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_pivot.getEncoderPosition()<=0){
      m_pivot.setSpeed(0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.resetEncoderPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_pivot.ifBackwardTriggered();
  }
}
