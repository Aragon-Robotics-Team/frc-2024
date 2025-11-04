// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeshooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class OutakeToSwitch extends Command {
  private Intake m_intake;
  private int m_state; // 1 --> over-intaked, 2 --> inside intake, 3 --> perfect position
  private double m_initTime;
  /** Creates a new OutakeToSwitch. */
  public OutakeToSwitch(Intake intake) {
    m_intake = intake;

    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intake.getSwitchTriggered()) {
      m_state = 2;
    } else {
      m_state = 1;
    }
    m_initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setSpeed(-0.6);
    if (m_state == 1) {
      if (m_intake.getSwitchTriggered())
        m_state++;
    } else {
      if (!m_intake.getSwitchTriggered())
        m_state++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_state == 3 || Timer.getFPGATimestamp() - m_initTime >= 0.3;
  }
}
