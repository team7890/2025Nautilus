// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Tilter;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;

// import frc.robot.commands.MechCommands.AlgaeIntakeRun;
// import frc.robot.commands.MechCommands.RunTilter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BargeShoot extends Command {
  private final Tilter objTilter;
  private final AlgaeIntake objAlgaeIntake;
  // private double dTilterPos;
  private int iCount;
  /** Creates a new BargeShoot. */
  public BargeShoot(Tilter objTilter_in, AlgaeIntake objAlgaeIntake_in) {
    objTilter = objTilter_in;
    objAlgaeIntake = objAlgaeIntake_in;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objTilter, objAlgaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // dTilterPos = objTilter.getTilterAPos();
    iCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    
    objTilter.moveToPositionMM(30.0);
    iCount = iCount + 1;
    if (iCount > 6) {
      objAlgaeIntake.runAlgaeIntake(Constants.MechSpeeds.dAlgaeShoot);
    }
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objAlgaeIntake.stopAlgaeIntake();
    objTilter.setLastKnownPos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
