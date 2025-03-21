// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Base_DriveRobotAuto extends Command {
  private final CommandSwerveDrivetrain objSwerve;
  private final double dMaxSpeed;
  private final double dMaxAngularRate;
  private final double dTranslateX, dTranslateY, dRotate;

  private double dCmdLeftY, dCmdLeftX, dCmdRightX;

   private SwerveRequest.FieldCentric drive  = new SwerveRequest.FieldCentric()
  .withDeadband(0.02).withRotationalDeadband(0.02) // Add a 2% deadband 10% is CTRE Value
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;

  /** Creates a new TeleOpDrive. */
  public Base_DriveRobotAuto(CommandSwerveDrivetrain objSwerve_in, double dMaxSpeed_in, double dMaxAngularRate_in, double dTranslateY_in, double dTranslateX_in, double dRotate_in) {
    objSwerve = objSwerve_in;
    dMaxSpeed = dMaxSpeed_in;
    dMaxAngularRate = dMaxAngularRate_in;
    dTranslateX = dTranslateX_in;
    dTranslateY = dTranslateY_in;
    dRotate = dRotate_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    dCmdLeftY = dTranslateY * dMaxSpeed;
    dCmdLeftX = dTranslateX * dMaxSpeed;
    dCmdRightX = dRotate * dMaxAngularRate;

    // do the get as double here and put value into another variable which you can then put through the util function to do the deadband
    
    objSwerve.setControl(
          drive.withVelocityX(dCmdLeftY).withVelocityY(dCmdLeftX).withRotationalRate(dCmdRightX)
    );

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objSwerve.setControl(
          drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
