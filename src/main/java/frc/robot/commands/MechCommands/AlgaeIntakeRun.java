// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeRun extends Command {

  private final AlgaeIntake objAlgaeIntake;
  private final double dSpeed;
  /** Creates a new AlgaeIntakeRun. */
  public AlgaeIntakeRun(AlgaeIntake objAlgaeIntake_in, double dSpeed_in) {
    objAlgaeIntake = objAlgaeIntake_in;
    dSpeed = dSpeed_in;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objAlgaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   objAlgaeIntake.resetCurrentTrip();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objAlgaeIntake.runAlgaeIntake(dSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objAlgaeIntake.stopAlgaeIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
