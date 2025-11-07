// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;


public class Wrist extends SubsystemBase {

  private TalonFX m_falcon = new TalonFX(WristConstants.kMotorID);
  private TalonFXConfiguration config;
  private LimitSwitch m_forward = new LimitSwitch(WristConstants.kForwardSwitchID);
  private LimitSwitch m_backward = new LimitSwitch(WristConstants.kBackwardSwitchID);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(WristConstants.kEncoderID);
  private StatusSignal<AngularVelocity> m_velocity;
  
  public Wrist() {
    config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = WristConstants.kGearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_falcon.setPosition(0);
    m_falcon.getConfigurator().apply(config);
    m_velocity = m_falcon.getVelocity();
  }

  public double getVelocity(){
    return m_velocity.refresh().getValueAsDouble();
  }

  public double getEncoderPosition(){
    // a simple .get() should work
    return m_encoder.get() - WristConstants.kEncoderOffset;
  }

  public boolean ifForwardTriggered(){
    return m_forward.ifTriggered();
  }

  public boolean ifBackwardTriggered(){
    return m_backward.ifTriggered();
  }

  public void setSpeed(double speed){
    if (!ifForwardTriggered() && speed<0) {
      speed = 0;
    } else if (!ifBackwardTriggered() && speed>0){
      speed = 0;
    }
    SmartDashboard.putNumber("Wrist speed", speed);
    m_falcon.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Wrist forward", ifForwardTriggered());
    SmartDashboard.putBoolean("Wrist backward", ifBackwardTriggered());
    SmartDashboard.putNumber("Wrist encoder", getEncoderPosition());
  }
}

