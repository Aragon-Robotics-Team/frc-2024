// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.PIDDown;
import frc.robot.commands.elevator.PIDElevatorZero;
import frc.robot.commands.pivot.PIDFront;
import frc.robot.commands.pivot.PIDPivotIntake;
import frc.robot.commands.wrist.PIDRaise;
import frc.robot.commands.wrist.PIDWristIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundIntake extends ParallelCommandGroup {
  /** Creates a new GroundIntake. */
  public GroundIntake(Pivot pivot, Wrist wrist, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PIDWristIntake(wrist),
      new SequentialCommandGroup(new WaitCommand(3), new PIDPivotIntake(pivot)),
      new SequentialCommandGroup(new WaitCommand(6), new PIDElevatorZero(elevator))
    );
  }
}