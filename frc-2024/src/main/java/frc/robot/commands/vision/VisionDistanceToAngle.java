
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class VisionDistanceToAngle extends Command {

  private static final class Config{
    public static final double kP = 0.007;
    public static final double kI = 0;
    public static final double kD = 0;    
    public static final double kMinI = -0.25;
    public static final double kMaxI = 0.25;
    public static final double kDeadband = 0.05;
  }

  private SwerveDrive m_swerve;
  private PIDController m_pid = new PIDController(Config.kP, Config.kI, Config.kD);
  // private double m_targetAngle;
  private double m_currentAngle;
  private double m_initAngle;
  private double m_turningSpeed;
  private double m_distance;
  private boolean m_detected;
  /** Creates a new VisionDistanceToAngle. */
  public VisionDistanceToAngle(SwerveDrive swerve) {
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    m_pid.setIntegratorRange(Config.kMinI, Config.kMaxI);
    System.out.println("Running vision snap");
    m_initAngle = SmartDashboard.getNumber("angle", 0);
    System.out.println("first angle: " + m_initAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentAngle = SmartDashboard.getNumber("angle", 0);
    // m_turningSpeed = m_pid.calculate(m_currentAngle, m_currentAngle + m_targetAngle);
    m_turningSpeed = m_pid.calculate(m_currentAngle, 0);

    SmartDashboard.putNumber("Turning speed", m_turningSpeed);
    // Construct chassis speed objects.
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, m_turningSpeed, m_swerve.getAngle());
    // Calculate module states.
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Set module states.
    m_swerve.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stop");
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_currentAngle) <= Config.kDeadband;
  }
}
