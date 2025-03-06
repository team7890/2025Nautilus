// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Tilter;

import frc.robot.Utilities;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunTilterToSafe extends Command {

  private Tilter objTilter;
  private double dTarget;

  private final double dMax = 18.0;
  private final double dMin = 9.0;

  private boolean bDone;

  /** Creates a new RunTilterToSafe. */
  public RunTilterToSafe(Tilter objTilter_in, double dTarget_in) {
    objTilter = objTilter_in;
    dTarget = dTarget_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objTilter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bDone = false;
    System.out.println("Run Tilter To Safe Init:  " + dTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objTilter.moveToPositionMM(Utilities.limitVariable(dMin, dTarget, dMax));
    bDone = (objTilter.getTilterAPos() > dMin - 0.1) && (objTilter.getTilterAPos() < dMax + 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objTilter.setLastKnownPos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bDone;
  }
  
}
