// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.util.Named;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveId, 
    DriveConstants.kFrontLeftTurnId,
    DriveConstants.kFrontLeftAbsoluteEncoderPort, 
    DriveConstants.kFrontLeftAbsoluteEncoderOffset, 
    DriveConstants.kFrontLeftDriveReversed,
    DriveConstants.kFrontLeftTurningReversed, 
    0
  );

  private final SwerveModule m_frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveId, 
    DriveConstants.kFrontRightTurnId,
    DriveConstants.kFrontRightAbsoluteEncoderPort, 
    DriveConstants.kFrontRightAbsoluteEncoderOffset, 
    DriveConstants.kFrontRightDriveReversed, 
    DriveConstants.kFrontRightTurningReversed, 
    1
  );

  private final SwerveModule m_backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveId, 
    DriveConstants.kBackLeftTurnId,
    DriveConstants.kBackLeftAbsoluteEncoderPort, 
    DriveConstants.kBackLeftAbsoluteEncoderOffset, 
    DriveConstants.kBackLeftDriveReversed, 
    DriveConstants.kBackLeftTurningReversed, 
    2
  );

  private final SwerveModule m_backRight = new SwerveModule(
    DriveConstants.kBackRightDriveId, 
    DriveConstants.kBackRightTurnId,
    DriveConstants.kBackRightAbsoluteEncoderPort, 
    DriveConstants.kBackRightAbsoluteEncoderOffset, 
    DriveConstants.kBackRightDriveReversed, 
    DriveConstants.kBackRightTurningReversed, 
    3
  );

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeft.getTranslation(), m_frontRight.getTranslation(), m_backLeft.getTranslation(), m_backRight.getTranslation());

  private final AHRS m_imu = new AHRS();

  private double m_totalCurrent;

  private final PIDController m_xPID = new PIDController(DriveConstants.kP_X, DriveConstants.kI_X, DriveConstants.kD_X);
  private final PIDController m_yPID = new PIDController(DriveConstants.kP_Y, DriveConstants.kI_Y, DriveConstants.kD_Y);
  private final PIDController m_thetaPID = new PIDController(DriveConstants.kP_Theta, DriveConstants.kI_Theta, DriveConstants.kD_Theta);

  private final Field2d m_field = new Field2d();

  private final SwerveDriveOdometry m_odo = new SwerveDriveOdometry(m_kinematics, getAngle(), new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_backLeft.getPosition(),
    m_backRight.getPosition()
  });

  private double m_xStartPose;
  private double m_yStartPose;

  public void resetHeading() {
    // m_imu.reset();
    // DEBUG: CHANGE THIS
    // System.out.println("DEBUG: called resetHeading.");
    m_imu.reset();
    // m_imu.setAngleAdjustment(90);
  }

  public InstantCommand resetHeadingCommand(){
    return new InstantCommand(this::resetHeading, this);
  }

  public void adjustAngle(double angle){
    m_imu.setAngleAdjustment(angle);
  }

  public void resetOdo(Pose2d pose) {
    m_odo.resetPosition(getAngle(), new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    }, pose);
  }

  public Pose2d getPoseMeters(){
    return m_odo.getPoseMeters();
  }

  public SwerveDriveKinematics getSwerveKinematics(){
    return m_kinematics;
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxTranslationalMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setModuleStates(states);  
  }

  public void resetAllDistances() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backLeft.resetEncoders();
    m_backRight.resetEncoders();
  }

  public void printVelocitiesandPositions(){
    System.out.println("Front left Velocity: " + m_frontLeft.getDriveVelocity());
    System.out.println("Front left Position: " + m_frontLeft.getDrivePosition());
    System.out.println("Front right velocity: " + m_frontRight.getDriveVelocity());
    System.out.println("Front right Position: " + m_frontRight.getDrivePosition());
    System.out.println("Back left velocity: " + m_backLeft.getDriveVelocity());
    System.out.println("Back left Position: " + m_backLeft.getDrivePosition());
    System.out.println("Back right velocity: " + m_backRight.getDriveVelocity());
    System.out.println("Back right Position: " + m_backRight.getDrivePosition());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(m_imu.getAngle(), 360));
  }

  public PIDController getXController(){
    return m_xPID;
  }

  public PIDController getYController(){
    return m_yPID;
  }

  public PIDController getThetaController(){
    return m_thetaPID;
  }
  
  public ChassisSpeeds getChassisSpeeds() { 
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(), 
      m_frontRight.getState(), 
      m_backLeft.getState(), 
      m_backRight.getState()
    );
  }

  public void driveRobotRelative(ChassisSpeeds speeds) { 
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  public void resetOdoToPose(){
    m_odo.resetPosition(getAngle(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(),
        m_backLeft.getPosition(), m_backRight.getPosition()
      }, new Pose2d(m_xStartPose, m_yStartPose, getAngle()));
  }

  public SwerveDrive(Command stowCommand, Command autoShoot, Command autoIntake, Command groundIntake, Command subwoofer, Command outtake) {
    NamedCommands.registerCommand("Print", new PrintCommand("Print command is running!!!"));
    NamedCommands.registerCommand("Stow", stowCommand);
    NamedCommands.registerCommand("AutoShoot", autoShoot);
    NamedCommands.registerCommand("AutoIntake", autoIntake);
    NamedCommands.registerCommand("GroundIntake", groundIntake);
    NamedCommands.registerCommand("Outtake", outtake);
    NamedCommands.registerCommand("Subwoofer", subwoofer);
    AutoBuilder.configureHolonomic(
                this::getPoseMeters, // Robot pose supplier
                this::resetOdo, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(this.getXController().getP(), this.getXController().getI(), this.getXController().getD()), // Translation PID constants
                        new PIDConstants(this.getThetaController().getP(), this.getThetaController().getI(), this.getThetaController().getD()), // Translation PID constants
                         // Rotation PID constants
                        DriveConstants.kMaxTranslationalMetersPerSecond, // Max module speed, in m/s
                        Units.inchesToMeters(14.0), // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
     );

     SmartDashboard.putData("Swerve/Distance/reset", new InstantCommand(this::resetAllDistances));
     double m_angle = SmartDashboard.getNumber("Driving/Adjust angle", 0);
     SmartDashboard.putNumber("Driving/Adjust angle", m_angle);
     adjustAngle(m_angle);

     new Thread(() -> {
      try {
        Thread.sleep(1000);
        resetHeading();
      } catch (Exception e) {
        // System.out.println("ERROR in sleep thread: " + e);
      }
     }).start();
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  @Override
  public void periodic() {
    m_odo.update(getAngle(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(),
        m_backLeft.getPosition(), m_backRight.getPosition()
      });

    SmartDashboard.putNumber("Angle", getAngle().getDegrees());
    
    m_totalCurrent = m_frontLeft.getDriveCurrent() + m_frontLeft.getTurnCurrent() + m_frontRight.getDriveCurrent() + m_frontRight.getTurnCurrent() + m_backLeft.getDriveCurrent() + m_backLeft.getTurnCurrent() + m_backRight.getDriveCurrent() + m_backRight.getTurnCurrent();
    SmartDashboard.putNumber("Total Current", m_totalCurrent);

    m_field.setRobotPose(m_odo.getPoseMeters());
    SmartDashboard.putNumber("Swerve/Odo/Pose X", m_odo.getPoseMeters().getX());
    SmartDashboard.putNumber("Swerve/Odo/Pose Y", m_odo.getPoseMeters().getY());
    SmartDashboard.putData("Swerve/Odo/Field", m_field);
    
    m_xStartPose = SmartDashboard.getNumber("Swerve/Odo/X", 2);
    m_yStartPose = SmartDashboard.getNumber("Swerve/Odo/Y", 2);
    SmartDashboard.putNumber("Swerve/Odo/X", m_xStartPose);
    SmartDashboard.putNumber("Swerve/Odo/Y", m_yStartPose);

    
    // // System.out.println("Chassis speeds:" + this.getChassisSpeeds());
    // // System.out.println("X error: " + (3 - m_odo.getPoseMeters().getX()));
  }
}