// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaeIntake;


import frc.robot.commands.MechCommands.RunCombinedTiltElev;
import frc.robot.Constants;
import frc.robot.commands.MechCommands.AlgaeIntakeRun;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAlgTilterWithIntakeL3 extends SequentialCommandGroup {
  /** Creates a new RunAlgTilterWithIntake. */
  public RunAlgTilterWithIntakeL3(Tilter objTilter, Elevator objElevator, AlgaeIntake objAlgaeIntake)

  {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new RunCombinedTiltElev(objTilter, objElevator, Constants.MechPos.dTiltAlgL3, Constants.MechPos.dElevALgL3),
        new AlgaeIntakeRun(objAlgaeIntake, Constants.MechSpeeds.dAlgaeIntake )
      )
    );
  }
}
