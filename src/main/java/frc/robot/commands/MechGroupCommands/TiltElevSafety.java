// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechGroupCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Utilities;

import frc.robot.commands.MechCommands.RunTilter;
import frc.robot.commands.MechCommands.RunElevator;
import frc.robot.commands.MechCommands.RunTilterToSafe;

import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TiltElevSafety extends SequentialCommandGroup {

  private double dMax = 18.0;
  private double dMin = 9.0;

  /** Creates a new TiltElevSafety. */
  public TiltElevSafety(Tilter objTilter, Elevator objElevator, double dTilterTarget, double dElevTarget) {

    addCommands(
      new RunTilterToSafe(objTilter, dTilterTarget),
      new ParallelDeadlineGroup(
        new RunElevator(objElevator, null, false, dElevTarget),
        new RunTilter(objTilter, null, false, Utilities.limitVariable(dMin, dTilterTarget, dMax))
      ),
      new RunTilter(objTilter, null, false, dTilterTarget)
    );

  }
}
