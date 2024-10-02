// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private CANSparkMax m_neo1 = new CANSparkMax(ElevatorConstants.kMotorID1, MotorType.kBrushless);
  private CANSparkMax m_neo2 = new CANSparkMax(ElevatorConstants.kMotorID2, MotorType.kBrushless);

  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(ElevatorConstants.kEncoderID);

  private LimitSwitch m_top = new LimitSwitch(ElevatorConstants.kTopSwitchID);
  private LimitSwitch m_bottom = new LimitSwitch(ElevatorConstants.kBottomSwitchID);


  /** Creates a new Elevator. */
  public Elevator() {
    m_neo1.getEncoder().setPosition(0);
    
    m_neo1.setIdleMode(IdleMode.kBrake);
    m_neo2.setIdleMode(IdleMode.kBrake);

    m_neo1.getEncoder().setPositionConversionFactor(1.0 / ElevatorConstants.kGearRatio);
    m_neo1.getEncoder().setVelocityConversionFactor(1.0 / ElevatorConstants.kGearRatio);
    m_neo2.getEncoder().setPositionConversionFactor(1.0 / ElevatorConstants.kGearRatio);
    m_neo2.getEncoder().setVelocityConversionFactor(1.0 / ElevatorConstants.kGearRatio);
    resetEncoderPosition(); // BOT STARTS AT BOTTOM POS.
  }

  public double getEncoderPosition(){
    // return m_neo1.getEncoder().getPosition();
    return m_encoder.get();
  }

  // This should be equal to the value returned by getEncoderPosition,
  // within an uncertainty.
  public double getMotorSensorPosition() {
    return m_neo1.getEncoder().getPosition();
  }

  public Boolean ifTopOpen(){
    return m_top.ifTriggered();
  }

  public Boolean ifBottomOpen(){
    return m_bottom.ifTriggered();
  }

  public void setSpeed(double speed){
    /*
     * NOTE: when converting to trapezoidal; these polarities NEED TO BE CHANGED.
     */
    if (!ifTopOpen() && speed < 0) {
      speed = 0;
    } else if (!ifBottomOpen() && speed > 0){
      speed = 0;
    }
    m_neo1.set(-speed);
    m_neo2.set(-speed);
  }
  
  public double getLeftSpeed(){
    return m_neo1.getEncoder().getVelocity();
  }

  public double getRightSpeed(){
    return m_neo2.getEncoder().getVelocity();
  }

  // Using left motor as the reference.
  public double getVelocity() {
    return getLeftSpeed();
  }  
  
  public void resetEncoderPosition(){
    m_encoder.reset();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Mech/Elevator/Encoder", getEncoderPosition());

    SmartDashboard.putBoolean("Mech/Elevator/Top switch", m_top.ifTriggered());
    // SmartDashboard.putBoolean("Mech/Elevator/Bottom switch", m_bottom.ifTriggered());

    if (!ifBottomOpen()){
      resetEncoderPosition();;
    }
  }
}
