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
public class MoveMechToHome extends SequentialCommandGroup {
  private DoubleSupplier dsJoystickFake;

  /** Creates a new MoveMechToPosition. */
  public MoveMechToHome(Elevator objElevator, Tilter objTilter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunTilter(objTilter, dsJoystickFake, false, 4.0),
      new RunElevator(objElevator, dsJoystickFake, false, 5.0)//,
     // new RunTilter(objTilter, dsJoystickFake, false, 0.0)
    );
  }
}
