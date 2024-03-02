// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.Logger;


public class Wrist extends SubsystemBase {

  private TalonFX m_falcon = new TalonFX(WristConstants.kMotorID);
  private LimitSwitch m_forward = new LimitSwitch(5);
  private LimitSwitch m_backward = new LimitSwitch(4);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(6);
  public Wrist() {
    m_falcon.setPosition(0);
    m_falcon.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getEncoderPosition(){
    return m_encoder.getAbsolutePosition() - WristConstants.kEncoderOffset;
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
    Logger.recordOutput("Wrist forward", ifForwardTriggered());
    Logger.recordOutput("Wrist backward", ifBackwardTriggered());
    Logger.recordOutput("Wrist encoder", getEncoderPosition());
  }
}

