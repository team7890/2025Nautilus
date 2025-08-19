// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LookAtLime extends Command {
  private final CommandSwerveDrivetrain objSwerve;
  private final double dMaxSpeed;
  private final double dMaxAngularRate;
 
  private final DoubleSupplier dsDriverLeftX;
  private final DoubleSupplier dsDriverLeftY;
 
  private double dCmdLeftX, dCmdLeftY;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .01;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= dMaxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTA("limelight") * kP;
    targetingForwardSpeed *= dMaxSpeed;
    targetingForwardSpeed *= 1.0;
    return targetingForwardSpeed;
  }




   private SwerveRequest.FieldCentric drive  = new SwerveRequest.FieldCentric()
  .withDeadband(0.02).withRotationalDeadband(0.02) // Add a 2% deadband 10% is CTRE Value
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;

  /** Creates a new TeleOpDrive. */
  public LookAtLime(CommandSwerveDrivetrain objSwerve_in, double dMaxSpeed_in, double dMaxAngularRate_in, DoubleSupplier dsDriverLeftX_in, DoubleSupplier dsDriverLeftY_in) {
    objSwerve = objSwerve_in;
    dMaxSpeed = dMaxSpeed_in;
    dMaxAngularRate = dMaxAngularRate_in;
    dsDriverLeftX = dsDriverLeftX_in;
    dsDriverLeftY = dsDriverLeftY_in;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    dCmdLeftX = dsDriverLeftX.getAsDouble() * dMaxSpeed;
    dCmdLeftY = dsDriverLeftY.getAsDouble() * dMaxSpeed;


    // do the get as double here and put value into another variable which you can then put through the util function to do the deadband
    
    objSwerve.setControl(
          drive.withVelocityX(dCmdLeftY).withVelocityY(dCmdLeftX).withRotationalRate(limelight_aim_proportional())
    );

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
