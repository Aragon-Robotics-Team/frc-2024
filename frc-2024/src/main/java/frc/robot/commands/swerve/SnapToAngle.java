// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class SnapToAngle extends Command {

  private static final class Config{
    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  private SwerveDrive m_swerve;
  private PIDController m_pid = new PIDController(Config.kP, Config.kI, Config.kD);
  private double m_turningSpeed;
  //Angles in degrees
  private double m_targetAngle;
  private double m_initAngle;
  private double m_currentAngle;

  /** Creates a new SnapToAngle. */
  public SnapToAngle(SwerveDrive swerve) {
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("I'm running!");
    m_initAngle = m_swerve.getAngle().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_targetAngle = SmartDashboard.getNumber("Goal angle degrees", 90);
    SmartDashboard.putNumber("Goal angle degrees", m_targetAngle);
    m_currentAngle = m_swerve.getAngle().getDegrees();

    m_turningSpeed = m_pid.calculate(m_currentAngle, m_initAngle + m_targetAngle);

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
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}