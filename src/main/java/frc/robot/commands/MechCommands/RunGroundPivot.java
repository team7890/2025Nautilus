// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Utilities;
import frc.robot.subsystems.GroundPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunGroundPivot extends Command {
  private GroundPivot objGroundPivot;  //do i need to add both of the winches?
  private DoubleSupplier dsJoystick;
  private boolean bIsDefaultCmd;
  private double dTargetPos;
  private double dTargetActive;
  private boolean bDone;
  /** Creates a new RunGroundPivot. */
  public RunGroundPivot(GroundPivot objGroundPivot_in, DoubleSupplier dsJoystick_in, boolean bIsDefaultCmd_in, double dTargetPos_in) {
    objGroundPivot = objGroundPivot_in;
    dsJoystick = dsJoystick_in;
    bIsDefaultCmd = bIsDefaultCmd_in;
    dTargetPos = dTargetPos_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objGroundPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (bIsDefaultCmd) {
      dTargetActive = objGroundPivot.getLastKownPos();
    }
    bDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        if (bIsDefaultCmd) {
      if (Math.abs(dsJoystick.getAsDouble()) > 0.02) {
        dTargetActive = dTargetActive + dsJoystick.getAsDouble() / 10.0;
      }
     
      dTargetActive = Utilities.limitVariable(0.0, dTargetActive, 30.0);
     // objElevator.moveToPositionCM(dTarget);
      objGroundPivot.moveToPositionMM(dTargetActive);
     // System.out.println("run elevator execute");
    }
    else {
      objGroundPivot.moveToPositionMM(dTargetPos);
     
    }
    SmartDashboard.putNumber("Ground Pivot Target", dTargetActive);
    bDone = (!bIsDefaultCmd && (Math.abs(objGroundPivot.getGroundPivotPos() - dTargetPos) < 0.2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objGroundPivot.setLastKnownPos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bDone;
  }
}
