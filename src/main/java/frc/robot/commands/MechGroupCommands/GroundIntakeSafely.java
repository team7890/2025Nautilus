// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechGroupCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.MechCommands.RunCombinedTiltElev;
import frc.robot.commands.MechCommands.RunGroundIntake;
import frc.robot.commands.MechCommands.RunGroundPivot;
import frc.robot.Constants;
import frc.robot.commands.MechCommands.CoralIntakeRun;

import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.GroundPivot;
import frc.robot.subsystems.CoralIntake;

import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundIntakeSafely extends SequentialCommandGroup {
  /** Creates a new GroundIntakeSafely. */
  public GroundIntakeSafely(Tilter objTilter, Elevator objElevator, GroundIntake objGroundIntake, GroundPivot objGroundPivot, CoralIntake objCoralIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new CoralIntakeRun(objCoralIntake, Constants.MechSpeeds.dCoralIntake),
        new SequentialCommandGroup(
          new RunCombinedTiltElev(objTilter, objElevator, Constants.MechPos.dTiltHome, Constants.MechPos.dElevHome),
          new RunGroundPivot(objGroundPivot, null, false, Constants.MechPos.dGroundPosDown),
          new RunGroundIntake(objGroundIntake, Constants.MechSpeeds.dGroundIntake)
        )
      )
    );
  }
}
