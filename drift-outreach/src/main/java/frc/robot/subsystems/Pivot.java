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

  private StatusSignal<Double> m_velocity; // using motor 3 as the reference.
  private StatusSignal<Double> m_position; // using motor 3 as the reference.

  /** Creates a new ElevatorPivot. */
  public Pivot() {
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

    m_velocity = m_falcon3.getVelocity();
    m_position = m_falcon3.getPosition();

  }

  public Boolean ifForwardOpen(){
    return m_forward.ifTriggered();
  }

  public Boolean ifBackwardOpen(){
    return m_backward.ifTriggered();
  }

  public double getEncoderPosition(){
    // negative one is added in order to match the velocity direction of the motors.
    return -1.0 * (m_encoder.getAbsolutePosition() - PivotConstants.kAbsEncoderOffset);
  }

  // This should be equal to the value returned by getEncoderPosition,
  // within an uncertainty.
  public double getMotorSensorPosition() {
    return m_position.refresh().getValueAsDouble();
  }

  public void resetEncoderPosition(){
    m_encoder.reset();
  }

  public void setSpeed(double speed){
    if (speed > 0 && !ifForwardOpen()) {
      speed = 0;
    } else if (speed < 0 && !ifBackwardOpen()){
      speed = 0;
    }
    m_falcon1.set(speed);
    m_falcon2.set(speed);
    m_falcon3.set(speed);
    m_falcon4.set(speed);
  }

  public double getVelocity() {
    return m_velocity.refresh().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Mech/Pivot/Encoder", getEncoderPosition());
    SmartDashboard.putNumber("Mech/Pivot/Motor1Current", m_falcon1.getSupplyCurrent().refresh().getValueAsDouble());
    SmartDashboard.putNumber("Mech/Pivot/Motor2Current", m_falcon2.getSupplyCurrent().refresh().getValueAsDouble());
    SmartDashboard.putNumber("Mech/Pivot/Motor3Current", m_falcon3.getSupplyCurrent().refresh().getValueAsDouble());
    SmartDashboard.putNumber("Mech/Pivot/Motor4Current", m_falcon4.getSupplyCurrent().refresh().getValueAsDouble());


    SmartDashboard.putBoolean("Forward switch: ", ifForwardOpen());
    SmartDashboard.putBoolean("Backward switch: ", ifBackwardOpen());

    if (!ifBackwardOpen()) {
      resetEncoderPosition();
    }
  }
}
