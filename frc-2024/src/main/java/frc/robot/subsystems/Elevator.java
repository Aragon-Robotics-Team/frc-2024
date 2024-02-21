// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private CANSparkMax m_neo1 = new CANSparkMax(ElevatorConstants.kMotorID1, MotorType.kBrushless);
  private CANSparkMax m_neo2 = new CANSparkMax(ElevatorConstants.kMotorID2, MotorType.kBrushless);
<<<<<<< HEAD
=======
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private CANSparkMax m_neo1 = new CANSparkMax(Constants.Elevator.kMotorID1, MotorType.kBrushless);
  private CANSparkMax m_neo2 = new CANSparkMax(Constants.Elevator.kMotorID2, MotorType.kBrushless);
>>>>>>> 31fa534 (Created Elevator subsytem and command)
=======
>>>>>>> 3d2e4b4 (Got pivot working with arcade)

  /** Creates a new Elevator. */
  public Elevator() {
    m_neo2.setInverted(true);
    m_neo1.getEncoder().setPosition(0);
  }

  public double getEncoderPosition(){
    return m_neo1.getEncoder().getPosition();
  }

  public void setSpeed(double speed){
    m_neo1.set(speed);
    // m_neo2.set(speed);
  }
<<<<<<< HEAD
=======
>>>>>>> 31fa534 (Created Elevator subsytem and command)
=======
>>>>>>> 3d2e4b4 (Got pivot working with arcade)

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
