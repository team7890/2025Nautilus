// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.AutoCommands.Base_DriveRobotAuto;
// import frc.robot.commands.MechGroupCommands.TiltElevSafety;
import frc.robot.commands.MechCommands.AlgaeIntakeRun;
import frc.robot.commands.MechCommands.CoralIntakeRun;
import frc.robot.commands.MechCommands.RunCombinedTiltElev;
import frc.robot.Constants;

import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;



public class Auto_MoveForwardGetAlgInfrontofDSRed extends SequentialCommandGroup {
  /** Creates a new Auto_MoveForwardGetAlgInfrontofDS. */
  public Auto_MoveForwardGetAlgInfrontofDSRed(CommandSwerveDrivetrain objDriveTrain, Tilter objTilter, Elevator objElevator, AlgaeIntake objAlgaeIntake, CoralIntake objCoralIntake, double dMaxSpeed, double dMaxAngularRate) {
   //
    addCommands(
      new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, -0.25, 0.0, 0.0).withTimeout(3.3),
      new ParallelCommandGroup(
      //Drives forwords to DS
        new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, -0.3, 0.0, 0.0).withTimeout(1.0),
      //Intakes alg
        new AlgaeIntakeRun(objAlgaeIntake, Constants.MechSpeeds.dAlgaeIntake).withTimeout(1.0)
      ),
    //Drives to Barge
      new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, 0.2, 0.0, 0.0).withTimeout(6.4),
    //Moves Mech up
      new RunCombinedTiltElev(objTilter, objElevator, Constants.MechPos.dTiltAlgBarge, Constants.MechPos.dElevAlgBarge),
      //Drives the Robot slowly into Barge
      new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, 0.07, -0.05, 0.0).withTimeout(0.25),
      //Shoots into Barge
      new AlgaeIntakeRun(objAlgaeIntake, Constants.MechSpeeds.dAlgaeShoot).withTimeout(0.5),
      //Moves Mech to Home pos
      new RunCombinedTiltElev(objTilter, objElevator, Constants.MechPos.dTiltHome, Constants.MechPos.dElevHome),
      // === Leave Points === \\
      new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, -0.3, 0.0, 0.0).withTimeout(1.0)
    
    
    
    );
  }
}
