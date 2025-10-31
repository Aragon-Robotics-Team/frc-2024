// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;

/*
 * Moves the subsystem to a selected position
 * with movement constrained to a trapezoidal 
 * motion profile. Uses feedforward and PID
 * to achieve the motion.
 */
public class TrapezoidalWrist extends Command {
  private Wrist m_wrist;
  private PIDController m_pid = new PIDController(
    WristConstants.kPTrap, WristConstants.kITrap, WristConstants.kDTrap
  );
  private TrapezoidProfile m_profile = new TrapezoidProfile(WristConstants.kConstraints);
  private SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(WristConstants.kS, WristConstants.kV);
  private State m_sample; // The last sampled state of the profile
  private State m_goal; // the goal state
  private double m_prevTime; // used to calculate dt; we could use 0.02s, but this should be more accurate.

  /** Creates a new TrapezoidalWrist. */
  public TrapezoidalWrist(Wrist wrist, double goal) {
    m_wrist = wrist;
    m_goal = new State(goal, 0.0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
    m_pid.setTolerance(0.0); // we want the PID always correcting; isFinished will use position to close the command.
    m_pid.setIntegratorRange(-0.015, 0.015);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // velocity should be zero at this point. position is anything.
    m_sample = new State(m_wrist.getEncoderPosition(), m_wrist.getVelocity());
    m_prevTime = Timer.getFPGATimestamp();
    
    SmartDashboard.putBoolean("TrapezoidalMotion/Wrist/Running", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = Timer.getFPGATimestamp(); // used to calculate dt
    m_sample = m_profile.calculate((currTime - m_prevTime), m_sample, m_goal); // sample the profile
    m_prevTime = currTime;

    // apply controllers
    double ff = m_ff.calculate(m_sample.velocity);
    double pid = m_pid.calculate(m_wrist.getEncoderPosition(), m_sample.position);
    double speed = ff + pid;
    m_wrist.setSpeed(speed);

    // logging
    SmartDashboard.putNumber("TrapezoidalMotion/Wrist/Commanded/Speed", speed);
    SmartDashboard.putNumber("TrapezoidalMotion/Wrist/Profile/Velocity", m_sample.velocity);
    SmartDashboard.putNumber("TrapezoidalMotion/Wrist/Profile/Position", m_sample.position);
    SmartDashboard.putNumber("TrapezoidalMotion/Wrist/Measured/Position", m_wrist.getEncoderPosition());
    SmartDashboard.putNumber("TrapezoidalMotion/Wrist/Measured/Velocity", m_wrist.getVelocity());
    SmartDashboard.putNumber("PIDTuning/pid_output", pid);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setSpeed(0.0);

    SmartDashboard.putBoolean("TrapezoidalMotion/Wrist/Running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(m_wrist.getEncoderPosition() - m_goal.position) <= WristConstants.kPositionDeadband;
    return false;
  }
}
