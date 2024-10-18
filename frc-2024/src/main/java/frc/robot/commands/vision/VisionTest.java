// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class VisionTest extends Command {
  /** Creates a new VisionTest. */
  private PhotonCamera m_cam;
  private Vision m_vision;

  public VisionTest(Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = vision;
    m_cam = m_vision.getCam();

    addRequirements(m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    //SmartDashboard.putNumber("vision_yaw", m_vision.getYaw());
    //System.out.println(m_vision.getYaw());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.print("Yaw: ");
    System.out.println(m_vision.getYaw());
    System.out.print("Pitch: ");
    System.out.println(m_vision.getPitch());

    //System.out.println(m_vision.getArea());
    //System.out.print("bello world");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
