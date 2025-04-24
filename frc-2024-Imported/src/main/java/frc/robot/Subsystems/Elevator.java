// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private CANSparkMax m_neo1 = new CANSparkMax(ElevatorConstants.kMotorID1, MotorType.kBrushless);
  private CANSparkMax m_neo2 = new CANSparkMax(ElevatorConstants.kMotorID2, MotorType.kBrushless);

  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);

  private LimitSwitch m_top = new LimitSwitch(1);
  private LimitSwitch m_bottom = new LimitSwitch(8);

  /** Creates a new Elevator. */
  public Elevator() {
    m_neo1.getEncoder().setPosition(0);
    m_neo1.setIdleMode(IdleMode.kBrake);
    m_neo2.setIdleMode(IdleMode.kBrake);
  }

  public double getEncoderPosition(){
    // return m_neo1.getEncoder().getPosition();
    return m_encoder.get();
  }

  public Boolean ifTopTriggered(){
    return m_top.ifTriggered();
  }

  public Boolean ifBottomTriggered(){
    return m_bottom.ifTriggered();
  }

  public void setSpeed(double speed){
    if (!ifTopTriggered() && speed<0) {
      speed = 0;
    } else if (!ifBottomTriggered() && speed>0){
      speed = 0;
    }
    System.out.println(-speed);
    m_neo1.set(-speed);
    m_neo2.set(-speed);

  }
  
  public double getLeftSpeed(){
    return m_neo1.getEncoder().getVelocity();
  }

  public double getRightSpeed(){
    return m_neo2.getEncoder().getVelocity();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator encoder", getEncoderPosition());

    SmartDashboard.putBoolean("Top switch", m_top.ifTriggered());
    SmartDashboard.putBoolean("Bottom switch", m_bottom.ifTriggered());
  }
}
