// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

/*
 * Moves the subsystem to a selected position
 * with movement constrained to a trapezoidal 
 * motion profile. Uses feedforward and PID
 * to achieve the motion.
 */
public class TrapezoidalPivot extends Command {
  private Pivot m_pivot;
  private PIDController m_pid = new PIDController(
    PivotConstants.kPTrap, PivotConstants.kITrap, PivotConstants.kDTrap
  );
  private TrapezoidProfile m_profile = new TrapezoidProfile(PivotConstants.kConstraints);
  private SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(PivotConstants.kS, PivotConstants.kV);
  private State m_sample; // The last sampled state of the profile
  private State m_goal; // the goal state
  private double m_prevTime; // used to calculate dt; we could use 0.02s, but this should be more accurate.

  /** Creates a new TrapezoidalPivot. */
  public TrapezoidalPivot(Pivot pivot, double goal) {
    m_pivot = pivot;
    m_goal = new State(goal, 0.0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
    m_pid.setTolerance(0.0); // we want the PID always correcting; isFinished will use position to close the command.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // velocity should be zero at this point. position is anything.
    m_sample = new State(m_pivot.getEncoderPosition(), m_pivot.getVelocity());
    m_prevTime = Timer.getFPGATimestamp();
    
    SmartDashboard.putBoolean("TrapezoidalMotion/Pivot/Running", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = Timer.getFPGATimestamp(); // used to calculate dt
    m_sample = m_profile.calculate((currTime - m_prevTime), m_sample, m_goal); // sample the profile
    m_prevTime = currTime;

    // apply controllers
    double ff = m_ff.calculate(m_sample.velocity);
    double pid = m_pid.calculate(m_pivot.getEncoderPosition(), m_sample.position);
    double speed = ff + pid;
    m_pivot.setSpeed(speed);

    // logging
    SmartDashboard.putNumber("TrapezoidalMotion/Pivot/Commanded/Speed", speed);
    SmartDashboard.putNumber("TrapezoidalMotion/Pivot/Profile/Velocity", m_sample.velocity);
    SmartDashboard.putNumber("TrapezoidalMotion/Pivot/Profile/Position", m_sample.position);
    SmartDashboard.putNumber("TrapezoidalMotion/Pivot/Measured/Position", m_pivot.getEncoderPosition());
    SmartDashboard.putNumber("TrapezoidalMotion/Pivot/Measured/Velocity", m_pivot.getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.setSpeed(0.0);

    SmartDashboard.putBoolean("TrapezoidalMotion/Pivot/Running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(m_pivot.getEncoderPosition() - m_goal.position) <= PivotConstants.kPositionDeadband;
    return false;
  }
}
