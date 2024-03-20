// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.DistanceToEncoderInterpolator;
import frc.robot.utils.DistanceToEncoderRegression;

public class VisionTest extends Command {
  private static final class Config{
    public static final double kDeadband = 0.005;
    public static final double kP = 2.0;
    public static final double kI = 0.05;
    public static final double kD = 0.1;
  }

  private Wrist m_wrist;
  private PIDController m_pid = new PIDController(Config.kP, Config.kI, Config.kD);
  private double m_speed;
  private DistanceToEncoderInterpolator m_interpolator = new DistanceToEncoderInterpolator();

  /** Creates a new VisionTest. */
  public VisionTest(Wrist wrist) {
    m_wrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_distance = SmartDashboard.getNumber("Vision/Distance", 0);
    SmartDashboard.putNumber("Vision/Distance", m_distance);
    double m_encoder = m_interpolator.getWristPosition(m_distance);
    SmartDashboard.putNumber("Vision/Wrist encoder", m_encoder);
    m_speed = m_pid.calculate(m_wrist.getEncoderPosition(), m_encoder);
    // Set module states.
    m_wrist.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
