// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class ArcadePivot extends Command {

    private static final class Config{
    public static final int kAxis = 1;
    public static final double kMultiplier = 0.5;
  }

  private Pivot m_pivot;
  private Joystick m_joystick;
  private double m_joystickInput;

  /** Creates a new ArcadePivot. */
  public ArcadePivot(Pivot pivot, Joystick joystick) {
    m_pivot = pivot;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("I'm running");
    m_pivot.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_joystickInput = m_joystick.getRawAxis(Config.kAxis)*Config.kMultiplier;
    // if (!m_pivot.ifForwardTriggered() && m_joystickInput > 0){
    //   m_pivot.setSpeed(0);
    //   // System.out.println("Forward pressed, moving forward, setting speed to 0");
    // } else if (!m_pivot.ifBackwardTriggered() && m_joystickInput < 0){
    //   m_pivot.setSpeed(0);
    //   // System.out.println("Backwards pressed, moving backward, setting speed to 0");
    // } else {
      SmartDashboard.putNumber("Pivot speed", m_joystickInput);
      m_pivot.setSpeed(m_joystickInput);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //   if (m_switch1.ifTriggered() || m_switch2.ifTriggered()){
  //   return true;
  // } else {
  //   return false;
  // }
    return false;
  }
}