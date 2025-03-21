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
import frc.robot.commands.MechCommands.RunGroundPivot;
import frc.robot.commands.MechGroupCommands.GroundIntakeSafely;
import frc.robot.commands.MechCommands.RunGroundIntake;
import frc.robot.Constants;

import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundPivot;
import frc.robot.subsystems.GroundIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_MoveForwardAndGetAlgwithouthomeBlue extends SequentialCommandGroup {


  /** Creates a new Auto_MoveForward. */
  public Auto_MoveForwardAndGetAlgwithouthomeBlue(CommandSwerveDrivetrain objDriveTrain, Tilter objTilter, Elevator objElevator, AlgaeIntake objAlgaeIntake, CoralIntake objCoralIntake, GroundPivot objGroundPivot, GroundIntake objGroundIntake, double dMaxSpeed, double dMaxAngularRate) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Drives the Robot close to the Reef
      new ParallelCommandGroup(
        new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, 0.2, 0.0, 0.0),
        // add put intake out and transfer coral to scoring mech
        new GroundIntakeSafely(objTilter, objElevator, objGroundIntake, objGroundPivot, objCoralIntake)
      ).withTimeout(0.9),
      new ParallelCommandGroup(
        //Runs to L2 Alg position
        new RunCombinedTiltElev(objTilter, objElevator, Constants.MechPos.dTiltAlgL2, Constants.MechPos.dElevAlgL2),
        //Intakes the Alg
        new AlgaeIntakeRun(objAlgaeIntake, Constants.MechSpeeds.dAlgaeIntake),
        //Drives the Robot slowly into the Reef
        new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, 0.07, 0.0, 0.0)
      ).withTimeout(1.45),
      //Intakes the Alg for longer time 
      new AlgaeIntakeRun(objAlgaeIntake, Constants.MechSpeeds.dAlgaeIntake).withTimeout(0.5),
      //Drives the Robot away and alines to L4 
      new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, -0.15, -0.1, 0.0).withTimeout(0.37),
      //Moves Mech to L4 pos
      new RunCombinedTiltElev(objTilter, objElevator, Constants.MechPos.dTiltCorL4, Constants.MechPos.dElevCorL4),
      //Drives Robot in to score L4
      new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, 0.075, 0.0, 0.0).withTimeout(0.6),
      //Shoots coral
      new CoralIntakeRun(objCoralIntake, Constants.MechSpeeds.dCoralShoot).withTimeout(0.45),
      //Drives Robot away from Reef 
      // new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, -0.15, -0.1, 0.0).withTimeout(0.5),
      //Moves Mech to home
      // new RunCombinedTiltElev(objTilter, objElevator, Constants.MechPos.dTiltHome, Constants.MechPos.dElevHome),
      //Drives towards the barge 
      new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, -0.2, -0.2, 0.0).withTimeout(1.57),
      //Moves Mech to Barge pos
      new RunCombinedTiltElev(objTilter, objElevator, Constants.MechPos.dTiltAlgBarge, Constants.MechPos.dElevAlgBarge),
      new ParallelCommandGroup(
        new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, -0.07, 0.0, 0.0),
        //Shoots into Barge
        new AlgaeIntakeRun(objAlgaeIntake, Constants.MechSpeeds.dAlgaeShoot)
      ).withTimeout(0.5),
      //Moves Mech to Home pos
      new RunCombinedTiltElev(objTilter, objElevator, Constants.MechPos.dTiltHome, Constants.MechPos.dElevHome),
      // === Leave points === \\ 
      new Base_DriveRobotAuto(objDriveTrain, dMaxSpeed, dMaxAngularRate, 0.3, 0.0, 0.0).withTimeout(1.0)




      );
  }
}
