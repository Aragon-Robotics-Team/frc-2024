// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ArcadeElevator extends Command {
 private static final class Config{
    public static final int kAxis = 5;
    public static final double kMultiplier = 0.5;
  }

  private Elevator m_elevator;
  private Joystick m_joystick;
  private double m_joystickInput;

  /** Creates a new TestElevator. */
  public ArcadeElevator(Elevator elevator, Joystick joystick) {
    m_elevator = elevator;
    m_joystick = joystick;
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
    m_joystickInput = m_joystick.getRawAxis(Config.kAxis)*Config.kMultiplier;
    SmartDashboard.putNumber("Elevator input", m_joystickInput);
    SmartDashboard.putNumber("Elevator left output", m_elevator.getLeftSpeed());
    SmartDashboard.putNumber("Elevator right output", m_elevator.getRightSpeed());
    m_elevator.setSpeed(m_joystickInput);
    
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
