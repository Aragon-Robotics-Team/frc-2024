// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.swerve;

// import edu.wpi.first.wpilibj2.command.Command;

// public class Drive extends Command {
//   private static final class Config {
//     public static final double kP = 0.01;
//     public static final double kI = 0;
//     public static final double kD = 0;
//   }

//   /** Creates a new Drive. */
//   //Drivetrain with Snap and Joystick integrated together

//   private final SwerveDrive m_swerve;
  
//   private final Supplier<Double> m_xSpeed, m_ySpeed, m_turningSpeed;
//   private final SlewRateLimiter m_xLimiter, m_yLimiter, m_turningLimiter;

//   private final JoystickButton m_visionAimButton, m_snapToZeroButton;

//   private PIDController m_pid = new PIDController(Config.kP, Config.kI, Config.kD);
//   private double m_targetAngle, m_initAngle, m_currentAngle;

//   public Drive(SwerveDrive swerve, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> turningSpeed, JoystickButton visionAimButton, JoystickButton snapToZeroButton) {
//     m_swerve = swerve;

//     m_xSpeed = xSpeed;
//     m_ySpeed = ySpeed;
//     m_turningSpeed = turningSpeed;

//     m_xLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAccelMetersPerSecondSquared);
//     m_yLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAccelMetersPerSecondSquared);
//     m_turningLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAngularAccelBotRotsPerSecondSquared);

//     m_visionAimButton = visionAimButton;
//     m_snapToZeroButton = snapToZeroButton;

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_swerve);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() { 
//     double xSpeed = m_xSpeed.get();
//     double ySpeed = m_ySpeed.get();
    
//     // if (m_visionAimButton){
      
//     // }
//     double turningSpeed = m_turningSpeed.get();

//     // Apply deadband.
//     xSpeed = Math.abs(xSpeed) > DriveConstants.kDeadband ? xSpeed : 0.0;
//     ySpeed = Math.abs(ySpeed) > DriveConstants.kDeadband ? ySpeed : 0.0;
//     turningSpeed = Math.abs(turningSpeed) > DriveConstants.kDeadband ? turningSpeed : 0.0;
//     // turningSpeed = 0.0;

//     // Limit velocity and acceleration.
//     xSpeed = m_xLimiter.calculate(xSpeed) * DriveConstants.kTeleopMaxSpeedMetersPerSecond;
//     ySpeed = m_yLimiter.calculate(ySpeed) * DriveConstants.kTeleopMaxSpeedMetersPerSecond;
//     turningSpeed = m_turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleopMaxTurningRadiansPerSecond;
//     SmartDashboard.putNumber("Swerve/turningSpeedCommanded", turningSpeed);

//     // Construct chassis speed objects.
//     ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, m_swerve.getAngle());
//     // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
//     // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, ySpeed, turningSpeed);


//     // Calculate module states.
//     SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

//     // Set module states.
//     m_swerve.setModuleStates(moduleStates);

//     SmartDashboard.putNumber("Swerve/heading", m_swerve.getAngle().getDegrees());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_swerve.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
