// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.PIDElevatorAmp;
import frc.robot.commands.pivot.PIDPivotAmp;
import frc.robot.commands.pivot.PIDPivotSubwoofer;
import frc.robot.commands.wrist.PIDWristAmp;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScore extends ParallelCommandGroup {
  /** Creates a new AmpScore. */
  public AmpScore(Pivot pivot, Wrist wrist, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // // addCommands(new FooCommand(), new BarCommand());
    // addCommands(
    //   new PIDWristAmp(wrist),
    //   new SequentialCommandGroup(new WaitCommand(3), new PIDPivotAmp(pivot)), //0.638
    //   new SequentialCommandGroup(new WaitCommand(6), new PIDElevatorAmp(elevator))
    // );
    addCommands(
      new PIDPivotAmp(pivot),
      new SequentialCommandGroup(new WaitCommand(0.25), new PIDWristAmp(wrist)),
      new SequentialCommandGroup(new WaitCommand(0.5), new PIDElevatorAmp(elevator))
    );
  }
}
