// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class Drive extends Command {
  private static final class Config {
    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  /** Creates a new Drive. */
  //Drivetrain with Snap and Joystick integrated together

  private final SwerveDrive m_swerve;
  
  private double m_xSpeed, m_ySpeed, m_turningSpeed;
  private final Joystick m_driverJoystick;
  private final SlewRateLimiter m_xLimiter, m_yLimiter, m_turningLimiter;

  private final JoystickButton m_visionAimButton, m_snapToZeroButton;

  private PIDController m_pid = new PIDController(Config.kP, Config.kI, Config.kD);
  private double m_targetAngle, m_initAngle, m_currentAngle;

  private DataLog m_log = DataLogManager.getLog();
  private DoubleLogEntry m_driveLog;

  public Drive(SwerveDrive swerve, Joystick driverJoystick, JoystickButton visionAimButton, JoystickButton snapToZeroButton) {
    m_swerve = swerve;

    m_driverJoystick = driverJoystick;

    m_xLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAccelMetersPerSecondSquared);
    m_yLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAccelMetersPerSecondSquared);
    m_turningLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAngularAccelBotRotsPerSecondSquared);

    m_visionAimButton = visionAimButton;
    m_snapToZeroButton = snapToZeroButton;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initAngle = m_swerve.getAngle().getDegrees();
    m_driveLog = new DoubleLogEntry(m_log, "/drive");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    m_xSpeed = -m_driverJoystick.getRawAxis(DriveConstants.kJoystickXAxis);
    m_ySpeed = -m_driverJoystick.getRawAxis(DriveConstants.kJoystickYxis);
    
    m_driveLog.append(m_xSpeed);
    m_driveLog.append(m_ySpeed);
    
    if (m_visionAimButton.getAsBoolean()){
      m_currentAngle = SmartDashboard.getNumber("angle", 0);
    // m_turningSpeed = m_pid.calculate(m_currentAngle, m_currentAngle + m_targetAngle);
      m_turningSpeed = m_pid.calculate(m_currentAngle, 0);

    } else if (m_snapToZeroButton.getAsBoolean()){
      m_targetAngle = 0;
      SmartDashboard.putNumber("Goal angle degrees", m_targetAngle);
      m_currentAngle = m_swerve.getAngle().getDegrees();

      m_turningSpeed = m_pid.calculate(m_currentAngle, m_initAngle + m_targetAngle);
    } else {
      m_turningSpeed = -m_driverJoystick.getRawAxis(DriveConstants.kJoystickRotAxis);
    }

    // Apply deadband.
    m_xSpeed = Math.abs(m_xSpeed) > DriveConstants.kDeadband ? m_xSpeed : 0.0;
    m_ySpeed = Math.abs(m_ySpeed) > DriveConstants.kDeadband ? m_ySpeed : 0.0;
    m_turningSpeed = Math.abs(m_turningSpeed) > DriveConstants.kDeadband ? m_turningSpeed : 0.0;
    // m_turningSpeed = 0.0;

    // Limit velocity and acceleration.
    m_xSpeed = m_xLimiter.calculate(m_xSpeed) * DriveConstants.kTeleopMaxSpeedMetersPerSecond;
    m_ySpeed = m_yLimiter.calculate(m_ySpeed) * DriveConstants.kTeleopMaxSpeedMetersPerSecond;
    m_turningSpeed = m_turningLimiter.calculate(m_turningSpeed) * DriveConstants.kTeleopMaxTurningRadiansPerSecond;
    SmartDashboard.putNumber("Swerve/turningSpeedCommanded", m_turningSpeed);

    // Construct chassis speed objects.
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_turningSpeed, m_swerve.getAngle());
    // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, ySpeed, turningSpeed);


    // Calculate module states.
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Set module states.
    m_swerve.setModuleStates(moduleStates);

    SmartDashboard.putNumber("Swerve/heading", m_swerve.getAngle().getDegrees());
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
