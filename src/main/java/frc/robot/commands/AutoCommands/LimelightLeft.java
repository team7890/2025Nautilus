// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimelightHelpers;
import frc.robot.commands.LimelightDrive;
import frc.robot.commands.LookAtLime;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightLeft extends SequentialCommandGroup {
  /** Creates a new LimelightLeft. */
  public LimelightLeft(CommandSwerveDrivetrain objDrivetrain, double dMaxSpeed, double dMaxAngularRate) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LimelightDrive(objDrivetrain, dMaxSpeed, dMaxAngularRate).withTimeout(2.25),
      new Base_DriveRobotCentricAuto(objDrivetrain, dMaxSpeed, dMaxAngularRate,0.0 , 0.25 , 0.0).withTimeout(0.15)

    );
  }
}
