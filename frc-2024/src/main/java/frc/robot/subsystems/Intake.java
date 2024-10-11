// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  
  private CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  private DigitalInput m_sensor = new DigitalInput(5);


  /** Creates a new Intake. */
  public Intake() {
    //setInverted depending on which way the motor spins?
    m_intakeMotor.burnFlash(); //does something to preven burnouts?
  }

  public boolean getSwitchTriggered() {
    return !m_sensor.get();
  }

  public void setSpeed(double speed){
    m_intakeMotor.set(speed);
  }
// 
  public Boolean ifSensorTriggered(){
    return m_sensor.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Mech/Intake/Motor Temperature", m_intakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Mech/Intake/Motor Current", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Mech/Intake/switch", getSwitchTriggered());
    SmartDashboard.putBoolean("Driving/IntakeSwitch", getSwitchTriggered());
    // This method will be called once per scheduler run
  }
}
