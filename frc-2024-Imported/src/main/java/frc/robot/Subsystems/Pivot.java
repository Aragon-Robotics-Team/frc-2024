// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private TalonFX m_falcon1 = new TalonFX(PivotConstants.kMotorID1);
  private TalonFX m_falcon2 = new TalonFX(PivotConstants.kMotorID2);
  private TalonFX m_falcon3 = new TalonFX(PivotConstants.kMotorID3);
  private TalonFX m_falcon4 = new TalonFX(PivotConstants.kMotorID4);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(2);

  private LimitSwitch m_forward = new LimitSwitch(9);
  private LimitSwitch m_backward = new LimitSwitch(7);


  /** Creates a new ElevatorPivot. */
  public Pivot() {
  //   m_falcon2.follow(m_falcon1);
  //   m_falcon4.follow(m_falcon3);
    m_falcon1.setInverted(true);
    m_falcon2.setInverted(true);
    m_falcon1.setNeutralMode(NeutralModeValue.Brake);
    m_falcon2.setNeutralMode(NeutralModeValue.Brake);
    m_falcon3.setNeutralMode(NeutralModeValue.Brake);
    m_falcon4.setNeutralMode(NeutralModeValue.Brake);

    // m_falcon1.getEncoder().setPosition(0);
  }

  public Boolean ifForwardTriggered(){
    return m_forward.ifTriggered();
  }

  public Boolean ifBackwardTriggered(){
    return m_backward.ifTriggered();
  }

  public double getEncoderPosition(){
    return m_encoder.get();
  }

  public void resetEncoderPosition(){
    m_encoder.reset();
  }

  public void setSpeed(double speed){
    if (!ifForwardTriggered() && speed<0) {
      speed = 0;
    } else if (!ifBackwardTriggered() && speed>0){
      speed = 0;
    }
    m_falcon1.set(-speed);
    m_falcon2.set(-speed);
    m_falcon3.set(-speed);
    m_falcon4.set(-speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot", getEncoderPosition());
    
    SmartDashboard.putBoolean("Forward switch: ", ifForwardTriggered());
    SmartDashboard.putBoolean("Backward switch: ", ifBackwardTriggered());

    if (!ifBackwardTriggered()){
      resetEncoderPosition();;
    }
  }
}
