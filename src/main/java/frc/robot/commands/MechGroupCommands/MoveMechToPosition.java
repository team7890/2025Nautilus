// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechGroupCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.MechCommands.RunElevator;
import frc.robot.commands.MechCommands.RunTilter;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tilter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveMechToPosition extends SequentialCommandGroup {
  private DoubleSupplier dsJoystickFake;

  /** Creates a new MoveMechToPosition. */
  public MoveMechToPosition(Elevator objElevator, Tilter objTilter, double dElevatorPos, double dTilterPos) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new RunElevator(objElevator, dsJoystickFake, false, dElevatorPos),
        new RunTilter(objTilter, dsJoystickFake, false, dTilterPos)
      )
    );
  }
}
