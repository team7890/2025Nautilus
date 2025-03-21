// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.Elevator;
import frc.robot.Utilities;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunCombinedTiltElev extends Command {
  private final Tilter objTilter;
  private final Elevator objElevator;
  private final double dTargetTilt, dTargetElev;

  private boolean bDoneTilt, bDoneElev, bDone;
  private double dTiltPos, dElevPos;
  private double dTiltSafeMin, dTiltSafeMax;
  private double dElevLastKnownPos;

  /** Creates a new RunCombinedTiltElev. */
  public RunCombinedTiltElev(Tilter objTilter_in, Elevator objElevator_in, double dTargetTilt_in, double dTargetElev_in) {
    objTilter = objTilter_in;
    objElevator = objElevator_in;
    dTargetTilt = dTargetTilt_in;
    dTargetElev = dTargetElev_in;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objTilter, objElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dTiltPos = objTilter.getTilterAPos();
    dElevPos = objElevator.getWinchAPos();
    objElevator.setLastKnownPos();
    dElevLastKnownPos = objElevator.getLastKownPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dTiltPos = objTilter.getTilterAPos();
    dElevPos = objElevator.getWinchAPos();

    // set min tilter position for three zones within elevator's motion
    // zone 1 means elevator can tilt farther back (to load coral) the lower it is and there is a line for the safe zone
    if (dElevPos < 6.5) {
      dTiltSafeMin = dElevPos * 2.0 - 9.0;
    }
    // zone 2 is where tilt is 4 and it is safe to go up to 15 before risk of hitting the funnel
    else if (dElevPos >= 6.5 && dElevPos < 15.0) {
      dTiltSafeMin = 4.0;
    }
    // zone 3 is where the funnel is at risk of hitting the crossbar
    else {
      dTiltSafeMin = 6.0;
    }
    // max tilt is always 30, no matter the elevator's position (can go farther at certian points but we don't need it to)
    dTiltSafeMax = 30.0;

    // move tilter to the target within its safe zone
    objTilter.moveToPositionMM(Utilities.limitVariable(dTiltSafeMin, dTargetTilt, dTiltSafeMax));

    // move elevator but only when tilter is in the safe zone
    if ((dTiltPos > (dTiltSafeMin - 0.1)) && (dTiltPos < (dTiltSafeMax + 0.1))) {
      objElevator.moveToPositionMM(dTargetElev);
      objElevator.setLastKnownPos();
      dElevLastKnownPos = objElevator.getLastKownPos();
    }
    else {
      objElevator.moveToPositionMM(dElevLastKnownPos);
    }

    // calculate when targets for tilt and elev have been reached
    bDoneTilt = Math.abs(objTilter.getTilterAPos() - dTargetTilt) < 0.1;
    bDoneElev = Math.abs(objElevator.getWinchAPos() - dTargetElev) < 0.1;
    bDone = bDoneTilt && bDoneElev;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objTilter.setLastKnownPos();
    objElevator.setLastKnownPos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bDone;
  }
}
