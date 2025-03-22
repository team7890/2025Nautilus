// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Utilities;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevator extends Command {
  private Elevator objElevator;  //do i need to add both of the winches?
  private DoubleSupplier dsJoystick;
  private boolean bIsDefaultCmd;
  private double dTargetPos;
  private double dTargetActive;
  private boolean bDone;

  /** Creates a new RunElevator. */

  public RunElevator(Elevator objElevator_in, DoubleSupplier dsJoystick_in, boolean bIsDefaultCmd_in, double dTargetPos_in) {
    objElevator = objElevator_in; //do i need to add both of the winches?
    dsJoystick = dsJoystick_in;
    bIsDefaultCmd = bIsDefaultCmd_in;
    dTargetPos = dTargetPos_in;

    addRequirements(objElevator); //do i need to add both of the winches?
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override //do i need to add both of the winches?
  public void initialize() {
    if (bIsDefaultCmd) {
      dTargetActive = objElevator.getLastKownPos();
    }
    bDone = false;
    // System.out.println("Run Elevator Init:  " + dTargetPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override  //do i need to add both of the winches?
  public void execute() {
    if (bIsDefaultCmd) {
      if (Math.abs(dsJoystick.getAsDouble()) > 0.02) {
        dTargetActive = dTargetActive + dsJoystick.getAsDouble() / 20.0;
      }
     
      dTargetActive = Utilities.limitVariable(0.0, dTargetActive, 33.0);
     // objElevator.moveToPositionCM(dTarget);
      objElevator.moveToPositionMM(dTargetActive);
     // System.out.println("run elevator execute");
    }
    else {
      objElevator.moveToPositionMM(dTargetPos);
     
    }
    SmartDashboard.putNumber("Elevator Target", dTargetActive);
    bDone = (!bIsDefaultCmd && (Math.abs(objElevator.getWinchAPos() - dTargetPos) < 0.2));
  }


  // Called once the command ends or is interrupted.
  @Override  //do i need to add both of the winches?
  public void end(boolean interrupted) {
    objElevator.setLastKnownPos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bDone;
  }
}
