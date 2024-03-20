// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private TalonFX m_falcon1 = new TalonFX(PivotConstants.kMotorID1);
  private TalonFX m_falcon2 = new TalonFX(PivotConstants.kMotorID2);
  private TalonFX m_falcon3 = new TalonFX(PivotConstants.kMotorID3);
  private TalonFX m_falcon4 = new TalonFX(PivotConstants.kMotorID4);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(PivotConstants.kEncoderID);

  private LimitSwitch m_forward = new LimitSwitch(PivotConstants.kForwardSwitchID);
  private LimitSwitch m_backward = new LimitSwitch(PivotConstants.kBackwardSwitchID);

  private StatusSignal<Double> m_falconPosition;

  /** Creates a new ElevatorPivot. */
  public Pivot() {
  //   m_falcon2.follow(m_falcon1);
  //   m_falcon4.follow(m_falcon3);
    // m_falcon1.setInverted(true);
    // m_falcon2.setInverted(true);
    // m_falcon1.setNeutralMode(NeutralModeValue.Brake);
    // m_falcon2.setNeutralMode(NeutralModeValue.Brake);
    // m_falcon3.setNeutralMode(NeutralModeValue.Brake);
    // m_falcon4.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration configRight = new TalonFXConfiguration();
    TalonFXConfiguration configLeft = new TalonFXConfiguration();

    configRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configRight.Feedback.SensorToMechanismRatio = PivotConstants.kGearRatio;

    configLeft.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configLeft.Feedback.SensorToMechanismRatio = PivotConstants.kGearRatio;


    m_falcon1.getConfigurator().apply(configRight);
    m_falcon2.getConfigurator().apply(configRight);
    m_falcon3.getConfigurator().apply(configLeft);
    m_falcon4.getConfigurator().apply(configLeft);
    // m_falcon1.getEncoder().setPosition(0);

    m_falconPosition = m_falcon1.getPosition();
  }

  public Boolean ifForwardTriggered(){
    return m_forward.ifTriggered();
  }

  public Boolean ifBackwardTriggered(){
    return m_backward.ifTriggered();
  }

  public double getEncoderPosition(){
    return m_encoder.getAbsolutePosition() - PivotConstants.kAbsEncoderOffset;
  }

  public void resetEncoderPosition(){
    m_encoder.reset();
  }

  public void resetFalconEncoderPosition() {
    m_falcon1.setPosition(0.0);
  }

  public double getFalconEncoderPosition() {
    return m_falconPosition.refresh().getValueAsDouble();
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
    SmartDashboard.putNumber("Mech/Pivot/Encoder", getEncoderPosition());

    SmartDashboard.putBoolean("Mech/Pivot/Forward switch: ", ifForwardTriggered());
    SmartDashboard.putBoolean("Mech/Pivot/Backward switch: ", ifBackwardTriggered());
    SmartDashboard.putNumber("Encoder Testing/Relative", m_encoder.get());
    SmartDashboard.putNumber("Encoder Testing/Absolute", getEncoderPosition());
    SmartDashboard.putNumber("Encoder Testing/Falcon", getFalconEncoderPosition());

    

    // if (!ifBackwardTriggered()){
    //   resetEncoderPosition();
    // }
    if (!ifBackwardTriggered()) {
      resetFalconEncoderPosition();
    }
  }
}
