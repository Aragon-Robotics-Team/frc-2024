// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Flywheel;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  private static final class Config {
    private static final double kP = 0.5;
    private static final double kI = 0;
    private static final double kD = 0;
    
  }
  private Flywheel m_flywheel;
  private PIDController m_pid = new PIDController(Config.kP, Config.kI, Config.kD);
  private double m_initialRPM;
  private double m_currentRPM;
  private double m_error;

  public Shoot(Flywheel flywheel, double wheelRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_flywheel = flywheel;
    m_pid.setSetpoint(wheelRPM);
    //m_pid.setTolerance();
    addRequirements(m_flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialRPM = m_flywheel.getRPM();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentRPM = m_flywheel.getRPM();
    m_error = m_currentRPM - m_initialRPM;
    m_flywheel.shoot(m_pid.calculate(m_error, m_pid.getSetpoint()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.stopShooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pid.atSetpoint();
  }
}