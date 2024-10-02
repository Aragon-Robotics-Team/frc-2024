// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class JoystickTurn extends Command {

  // private final SwerveDrive m_swerve;
  // private final Supplier<Double> m_turningSpeed;
  // private final SlewRateLimiter m_turningLimiter;

  // /** Creates a new JoystickTurn. */
  // public JoystickTurn(SwerveDrive swerve, Supplier<Double> turningSpeed) {
  //   m_swerve = swerve;
  //   m_turningSpeed = turningSpeed;
  //   m_turningLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAngularAccelRadiansPerSecondSquared);

  //   addRequirements(swerve);
  // }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {}

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
