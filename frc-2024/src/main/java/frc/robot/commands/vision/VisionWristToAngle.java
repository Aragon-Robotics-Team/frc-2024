// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.DistanceToEncoderRegression;

public class VisionWristToAngle extends Command {
  
  private static final class Config{
    public static final double kDeadband = 0.005;
    public static final double kP = 2.0;
    public static final double kI = 0.1;
    public static final double kD = 0.1;
  }

  private Wrist m_wrist;
  private PIDController m_pid = new PIDController(Config.kP, Config.kI, Config.kD);

  private boolean m_detected;
  private double m_distance;
  private double m_speed;
  private double m_encoderValue;

  /** Creates a new VisionWristToAngle. */
  public VisionWristToAngle(Wrist wrist) {
    m_wrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
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
    m_distance = SmartDashboard.getNumber("Distance", 0);
    System.out.println("Distance: " + m_distance);
    m_detected = SmartDashboard.getBoolean("TagDetected", false);
    System.out.println("Detected: " + m_detected);
    m_encoderValue = DistanceToEncoderRegression.getWristPosition(m_distance);
    // Construct chassis speed objects.
    m_speed = m_pid.calculate(m_wrist.getEncoderPosition(), m_encoderValue);
    // Set module states.
    m_wrist.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stop");
    m_wrist.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
