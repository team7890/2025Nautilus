// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimelightDrive extends Command {
  private final CommandSwerveDrivetrain objSwerve;
  private final double dMaxSpeed;
  private final double dMaxAngularRate;

 

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
    double dragonRotVel = LimelightHelpers.getTX("limelight-dragon") * kP;

    // convert to radians per second for our drive method
    dragonRotVel *= dMaxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    dragonRotVel *= -1.0;

    double trainRotVel = LimelightHelpers.getTX("limelight-train") * kP;

    // convert to radians per second for our drive method
    trainRotVel *= dMaxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    trainRotVel *= -1.0;

    return ((dragonRotVel + trainRotVel) / 2);
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {   
    if (LimelightHelpers.getTV("limelight-dragon")) {
      double kP = 0.08;
    double dragonDistanceVel = (12 - LimelightHelpers.getTA("limelight-dragon")) * kP;
    double trainDistanceVel = (12 - LimelightHelpers.getTA("limelight-train")) * kP;
    // targetingForwardSpeed *= dMaxSpeed;
    SmartDashboard.putNumber("LimeDistanceSpeed", dragonDistanceVel);
    return ((dragonDistanceVel + trainDistanceVel) / 2);
    } 
    else {
      return 0.0;
    }
  }




   private SwerveRequest.RobotCentric drive  = new SwerveRequest.RobotCentric()
  .withDeadband(0.02).withRotationalDeadband(0.02) // Add a 2% deadband 10% is CTRE Value
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;

  /** Creates a new TeleOpDrive. */
  public LimelightDrive(CommandSwerveDrivetrain objSwerve_in, double dMaxSpeed_in, double dMaxAngularRate_in) {
    objSwerve = objSwerve_in;
    dMaxSpeed = dMaxSpeed_in;
    dMaxAngularRate = dMaxAngularRate_in;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // do the get as double here and put value into another variable which you can then put through the util function to do the deadband
    
    objSwerve.setControl(
          drive.withVelocityX(limelight_range_proportional()).withVelocityY(0.0).withRotationalRate(limelight_aim_proportional())
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
