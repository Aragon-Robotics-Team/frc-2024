// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;
  private final SparkMaxConfig config;

  private final RelativeEncoder m_driveEncoder;
  private final DutyCycleEncoder m_absoluteEncoder;

  private final double m_absoluteEncoderOffset;
  private final PIDController m_drivingPIDController;
  private final PIDController m_turningPIDController;

  private final int m_moduleId;

  private Translation2d m_translation = new Translation2d();

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveId, int turnId, int absoluteEncoderPort, double absoluteEncoderOffset,
      boolean driveReversed, boolean turningReversed, int moduleId) {
    // Initialize motors and encoders.
    m_driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    m_turnMotor = new SparkMax(turnId, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_absoluteEncoder = new DutyCycleEncoder(new DigitalInput(absoluteEncoderPort));

    // conversion coefficiencts now set directly in each method

    // Initialize Everything else.
    m_absoluteEncoderOffset = absoluteEncoderOffset;
    m_turningPIDController = new PIDController(DriveConstants.kPTurning, DriveConstants.kITurning,
        DriveConstants.kDTurning);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_drivingPIDController = new PIDController(DriveConstants.kPDriving, DriveConstants.kIDriving, 
        DriveConstants.kDDriving);
    m_moduleId = moduleId;

    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(driveReversed);
    
    m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(turningReversed);
    m_turnMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    resetEncoders();

    SmartDashboard.putData("Swerve/Distance/reset_" + m_moduleId,  new InstantCommand(() -> resetEncoders()));

  }

  
  public double getTurningPosition() {
    return m_absoluteEncoder.get()*DriveConstants.kTurnEncoderPositionToRadians - m_absoluteEncoderOffset;
    //return m_absoluteEncoder.get();
  }

  public Rotation2d getRotation() {
    return new Rotation2d(getTurningPosition());
  }

  public double getDrivePosition() {
    return m_driveEncoder.getPosition()*DriveConstants.kDriveEncoderPositionToMeters;
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity()*DriveConstants.kDriveEncoderVelocityToMetersPerSec;
  }

  public void resetEncoders() {
    m_driveEncoder.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getRotation());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), getRotation());
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < DriveConstants.kTranslationalDeadbandMetersPerSecond) {
      stop();
      return;
    }

    // deprecated later, idc to change it for now
    state = SwerveModuleState.optimize(state, getState().angle);
    
    SmartDashboard.putNumber("Swerve/Speed/Commanded/Module_" + m_moduleId, state.speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Commanded/Angle_" + m_moduleId, state.angle.getRadians());
    SmartDashboard.putNumber("Swerve/Angle/Commanded/Module_" + m_moduleId, state.angle.getRadians());

    double ff = state.speedMetersPerSecond / DriveConstants.kMaxTranslationalMetersPerSecond;
    double pid = m_drivingPIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);
    m_driveMotor.set(ff + pid);
    SmartDashboard.putNumber("steering set" + m_moduleId, (m_turningPIDController.calculate(getRotation().getRadians(), state.angle.getRadians())));
    m_turnMotor.set(m_turningPIDController.calculate(getRotation().getRadians(), state.angle.getRadians()));
    System.out.println(""+m_moduleId + ": "+ m_turningPIDController.calculate(getRotation().getRadians(), state.angle.getRadians()));

    SmartDashboard.putString("Swerve_" + m_moduleId + "_state", state.toString());
  }

  public Translation2d getTranslation(){
    return new Translation2d(m_translation.getX(), m_translation.getY());
  }

  public void stop() {
    m_driveMotor.set(0.0);
    m_turnMotor.set(0.0);
  }

  public double getDriveCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

  public double getTurnCurrent() {
    return m_turnMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Swerve/Angle/Measured/Module_" + m_moduleId, getTurningPosition());
    // SmartDashboard.putNumber("Swerve/Speed/Measured/Module_" + m_moduleId, getDriveVelocity());
    // SmartDashboard.putNumber("Swerve/Distance/Module_" + m_moduleId, getDrivePosition());
    
  }
}
