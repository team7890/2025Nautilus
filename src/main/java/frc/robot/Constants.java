// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CANIds{
    public static final int iElevator = 21;
    // public static final int iElevatorFollower = 5;
    public static final int iAlgaeIntake = 25;
    public static final int iCoralIntake = 26;
    public static final int iTilterA = 23;
    public static final int iTilterB = 24;
    // public static final int iClimber = 6;
    // public static final int iClimberFollower = ;
    // public static final int iClimberWinchA = 27;
    // public static final int iClimberWinchB = 28;
    public static final int iClimberDeploy = 29;
    public static final int iGroundIntake = 31;
    public static final int iGroundPivot = 28;
  }

  public static class MechPos{
    // === home ===
    public static final double dTiltHome = 0.0;
    public static final double dElevHome = 0.0;
    public static final double dTiltHandoff = 0.5;

    // === algae brage ===
    public static final double dTiltAlgBarge = 23.0;
    public static final double dElevAlgBarge = 28.0;
    
    // === Algae proc ===
    public static final double dTiltAlgProc = -2.0;
    public static final double dElevAlgProc = 0.0;

    // === Algae load L3 ===
    public static final double dTiltAlgL3 = 8.0;
    public static final double dElevALgL3 = 19.4;

    // === Algae load L2 ===
    public static final double dTiltAlgL2 = 7.0;  //changed from 6 -> 7 after qual2
    public static final double dElevAlgL2 = 6.4;

    // === Coral L4 ===
    public static final double dTiltCorL4 = 25.0;
    public static final double dElevCorL4 = 31.5;

    // === Coral L3 ===
    public static final double dTiltCorL3 = 25.2;
    public static final double dElevCorL3 = 3.0;

    // === Coral L2 ===
    public static final double dTiltCorL2 = 20.0;
    public static final double dElevCorL2 = 0.0;

    // === Coral L1 ===
    public static final double dTiltCorL1 = -3.0;    //SET NUMBERS
    public static final double dElevCorL1 = 0.0;

    // === Coral load ===
    public static final double dTiltCorLoad = -8.0;
    public static final double dElevCorLoad = 0.0;

    // === Algae Driver Station === \\
    public static final double dTiltDSAlg = 2.0;
    public static final double dElevDSAlg = 0.0;

    public static final double dGroundPosDown = 23.0;
    public static final double dGroundPosHome = 0.15; //TODO: FIX THESE NOW PLEASE THANKN YOU
    public static final double dGroundPosL1 = 5.0;

    public static final double dTiltGroundSafety = 5.0;

  }

  public static class MechSpeeds {
    
    
    public static final double dCoralShoot = -0.25;   // coral shoot OUT
    public static final double dCoralHold = 0.055;     // coral idle rollers to hold coral in mechanism
    public static final double dCoralIntake = 0.25;   // coral intake
    public static final double dGroundIntake = -0.5;   // ground coral IN
    public static final double dAlgaeShoot = -1.0;    // algae shoot OUT
    public static final double dAlgaeIntake = 0.5;    // algae intake
    public static final double dGroundEject = 0.3;   // eject ground coral or score L1
  
  }

  // public static class Climb{
  //   public static final double dDeploySpeed = 0.05;
  //   public static final double dWinchSpeed = 0.4;
  // }
}
