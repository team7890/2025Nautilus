// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.GroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunGroundIntake extends Command {
  private GroundIntake objGroundIntake;
  private double dSpeed;
  /** Creates a new GroundCoralIn. */
  public RunGroundIntake(GroundIntake objGroundIntake_in, double dSpeed_in) {
    objGroundIntake = objGroundIntake_in;
    dSpeed = dSpeed_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objGroundIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objGroundIntake.runGroundIntake(dSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objGroundIntake.stopGroundIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
