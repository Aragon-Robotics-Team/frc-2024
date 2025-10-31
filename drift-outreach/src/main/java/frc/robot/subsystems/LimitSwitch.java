// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {

  private DigitalInput m_limitSwitch;

  public LimitSwitch(int slotDIO) {
    m_limitSwitch = new DigitalInput(slotDIO);
  }

  public boolean ifTriggered() {
    return m_limitSwitch.get();
  }
}
