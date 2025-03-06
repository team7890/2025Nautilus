// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities;
import frc.robot.subsystems.Tilter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunTilter extends Command {
  private Tilter objTilter;
  private DoubleSupplier dsJoystick;
  private boolean bIsDefaultCmd;
  private double dTargetPos;
  private double dTargetActive;
  private boolean bDone;

  /** Creates a new RunTilter. */

  public RunTilter(Tilter objTilter_in, DoubleSupplier dsJoystick_in, boolean bIsDefaultCmd_in, double dTargetPos_in) {
    objTilter = objTilter_in; 
    dsJoystick = dsJoystick_in;
    bIsDefaultCmd = bIsDefaultCmd_in;
    dTargetPos = dTargetPos_in;

    addRequirements(objTilter); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    if (bIsDefaultCmd) {
      dTargetActive = objTilter.getLastKownPos();
    }
    bDone = false;
    System.out.println("Run Tilter Init:  " + dTargetPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override  
  public void execute() {
    if (bIsDefaultCmd) {
      if (Math.abs(dsJoystick.getAsDouble()) > 0.02) {
        dTargetActive = dTargetActive + dsJoystick.getAsDouble() / 20.0;
      }
      dTargetActive = Utilities.limitVariable(-10.5, dTargetActive, 50.0);
      objTilter.moveToPositionMM(dTargetActive);
    }
    else {
      dTargetActive = dTargetPos;
      objTilter.moveToPositionMM(dTargetPos);
    }
    SmartDashboard.putNumber("Tilter Target", dTargetActive);
    bDone = (!bIsDefaultCmd && (Math.abs(objTilter.getTilterAPos() - dTargetPos) < 0.1));
  }

  // Called once the command ends or is interrupted.
  @Override  //do i need to add both of the Tilteres?
  public void end(boolean interrupted) {
    objTilter.setLastKnownPos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bDone;
  }
}
