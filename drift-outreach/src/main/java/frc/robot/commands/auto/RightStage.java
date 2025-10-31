// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.elevator.PIDElevatorZero;
import frc.robot.commands.pivot.PIDPivotSetpoint;
import frc.robot.commands.pivot.PIDPivotStow;
import frc.robot.commands.pivot.PIDPivotSubwoofer;
import frc.robot.commands.pivot.TrapezoidalPivot;
import frc.robot.commands.wrist.PIDWristSetpoint;
import frc.robot.commands.wrist.PIDWristStow;
import frc.robot.commands.wrist.PIDWristSubwoofer;
import frc.robot.commands.wrist.TrapezoidalWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightStage extends ParallelCommandGroup {
  /** Creates a new SpeakerScore. */
  public RightStage(Pivot pivot, Wrist wrist, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PIDElevatorZero(elevator),
      new TrapezoidalPivot(pivot, PositionConstants.kSubwooferPivot),
      new TrapezoidalWrist(wrist, PositionConstants.kStageWrist)
    );
  }
}
