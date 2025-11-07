// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.FeedbackConfigs;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private TalonFX m_falcon1 = new TalonFX(PivotConstants.kMotorID1);
  private TalonFX m_falcon2 = new TalonFX(PivotConstants.kMotorID2);
  private TalonFX m_falcon3 = new TalonFX(PivotConstants.kMotorID3);
  private TalonFX m_falcon4 = new TalonFX(PivotConstants.kMotorID4);


  private TalonFXConfiguration cwMotorConfig;
  private TalonFXConfiguration ccwMotorConfig;
  // private FeedbackConfigs clockwiseMotorFeedbackConfig;
  // private FeedbackConfigs ccwMotorFeedbackConfig;

  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(PivotConstants.kEncoderID);

  private LimitSwitch m_forward = new LimitSwitch(PivotConstants.kForwardSwitchID);
  private LimitSwitch m_backward = new LimitSwitch(PivotConstants.kBackwardSwitchID);

  private double encoderOffset = 0.292; // used to replace the deprecated encoder.reset() method
  private StatusSignal<AngularVelocity> m_velocity;

  /** Creates a new ElevatorPivot. */
  public Pivot() {
    // clockwiseMotorConfig = new MotorOutputConfigs();
    // clockwiseMotorConfig.withInverted(InvertedValue.Clockwise_Positive); 
    cwMotorConfig = new TalonFXConfiguration();
    cwMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cwMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cwMotorConfig.Feedback.SensorToMechanismRatio = PivotConstants.kGearRatio;
    
    ccwMotorConfig = new TalonFXConfiguration();
    ccwMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ccwMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ccwMotorConfig.Feedback.SensorToMechanismRatio = PivotConstants.kGearRatio;

    // if this is backwards just switch it (falcon 1/2 should then get ccw motor config)
    m_falcon1.getConfigurator().apply(cwMotorConfig); // previous command subject to deprecation in 2026
    m_falcon2.getConfigurator().apply(cwMotorConfig);
    m_falcon3.getConfigurator().apply(ccwMotorConfig);
    m_falcon4.getConfigurator().apply(ccwMotorConfig);
    
    m_falcon1.setNeutralMode(NeutralModeValue.Brake);
    m_falcon2.setNeutralMode(NeutralModeValue.Brake);
    m_falcon3.setNeutralMode(NeutralModeValue.Coast);
    m_falcon4.setNeutralMode(NeutralModeValue.Coast);

    m_velocity = m_falcon3.getVelocity();

    // m_falcon1.getEncoder().setPosition(0);
  }

  public double getVelocity(){
    return m_velocity.refresh().getValueAsDouble();
  }
  public Boolean ifForwardTriggered(){
    return m_forward.ifTriggered();
  }

  public Boolean ifBackwardTriggered(){
    return m_backward.ifTriggered();
  }

  public double getEncoderPosition(){
    return m_encoder.get() - PivotConstants.kAbsEncoderOffset;
  }

  
  //something that was deprecated here has been removed
  public void resetEncoderPosition(){
    encoderOffset = m_encoder.get();
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
      resetEncoderPosition();
    }
  }
}
