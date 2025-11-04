// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private SparkMax m_neo1 = new SparkMax(ShooterConstants.kShooterMotorID1, MotorType.kBrushless);
  private SparkMax m_neo2 = new SparkMax(ShooterConstants.kShooterMotorID2, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Shooter() {
    SparkMaxConfig config1 = new SparkMaxConfig();
    config1.inverted(true);
    m_neo1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //m_neo2.burnFlash();
    //setInverted depending on which way the motor spins?    
  }

  public void setSpeed(double speed){
    m_neo1.set(speed);
    m_neo2.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Mech/Shooter/Motor Temperature", m_neo1.getMotorTemperature());
    SmartDashboard.putNumber("Mech/Shooter/Motor Current", m_neo1.getOutputCurrent());
    // This method will be called once per scheduler run
  }
}
