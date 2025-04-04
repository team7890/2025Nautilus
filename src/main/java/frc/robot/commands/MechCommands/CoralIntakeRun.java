// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntakeRun extends Command {
  private final CoralIntake objCoralIntake;
  private final double dSpeed;
  /** Creates a new CoralIntakeRun. */
  public CoralIntakeRun(CoralIntake objCoralIntake_in, double dSpeed_in) {
    objCoralIntake = objCoralIntake_in;
    dSpeed = dSpeed_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objCoralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    objCoralIntake.resetCurrentTrip();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objCoralIntake.runCoralIntake(dSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objCoralIntake.stopCoralIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
