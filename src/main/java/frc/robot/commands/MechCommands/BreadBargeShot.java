// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechCommands;

import java.util.concurrent.DelayQueue;
import java.util.concurrent.Delayed;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.BargeShoot;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tilter;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BreadBargeShot extends SequentialCommandGroup {
  /** Creates a new BreadBargeShot. */
  public BreadBargeShot(Tilter objTilter, Elevator objElevator, AlgaeIntake objAlgaeIntake ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new RunCombinedTiltElev(objTilter, objElevator, Constants.MechPos.dTiltAlgBarge, Constants.MechPos.dElevHome).withTimeout(0.5),
      new ParallelCommandGroup(
        new RunCombinedTiltElev(objTilter, objElevator, Constants.MechPos.dTiltAlgBarge, Constants.MechPos.dElevAlgBarge),
        new SequentialCommandGroup(
          Commands.waitSeconds(0.5),
          new AlgaeIntakeRun(objAlgaeIntake, Constants.MechSpeeds.dAlgaeShoot)
        )
      )
    );
  }
}
