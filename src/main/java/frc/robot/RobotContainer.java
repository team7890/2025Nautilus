// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.MechPos;
import frc.robot.Constants.MechSpeeds;
// === COMMANDS === \\
import frc.robot.commands.MechCommands.RunElevator;
import frc.robot.commands.MechCommands.RunGroundPivot;
import frc.robot.commands.TeleOpDrive;
import frc.robot.commands.MechCommands.AlgaeIntakeRun;
import frc.robot.commands.MechCommands.CoralIntakeRun;
import frc.robot.commands.MechCommands.RunGroundIntake;
import frc.robot.commands.MechCommands.RunTilter;
import frc.robot.commands.MechCommands.ShootAlgae;
import frc.robot.commands.MechCommands.ShootCoral;
// import frc.robot.commands.MechCommands.RunClimber;
import frc.robot.commands.MechCommands.RunCombinedTiltElev;
import frc.robot.commands.MechCommands.ResetTilter;
import frc.robot.commands.MechCommands.RunGroundPivot;
import frc.robot.commands.MechGroupCommands.GroundIntakeSafely;
import frc.robot.commands.MechGroupCommands.PositionForCoralL1;
// === Group Commands ===
// import frc.robot.commands.MechGroupCommands.MoveMechToHome;
// import frc.robot.commands.MechGroupCommands.MoveMechToPosition;
import frc.robot.commands.MechGroupCommands.TiltElevSafety;

// === Auto Commands ===
import frc.robot.commands.AutoCommands.Auto_MoveForwardBlue;
import frc.robot.commands.AutoCommands.Auto_MoveForwardAndGetAlg;
import frc.robot.commands.AutoCommands.Auto_MoveForwardAndGetAlgwithouthomeBlue;
import frc.robot.commands.AutoCommands.Auto_MoveForwardAndGetAlgwithouthomeRed;
import frc.robot.commands.AutoCommands.Auto_MoveForwardGetAlgInfrontofDSBlue;
import frc.robot.commands.AutoCommands.Auto_MoveForwardGetAlgInfrontofDSRed;
import frc.robot.commands.AutoCommands.Auto_MoveForwardRed;
import frc.robot.commands.AutoCommands.BargeShoot;
// === SUBSYSTEMS === \\
import frc.robot.subsystems.AlgaeIntake;
// import frc.robot.subsystems.ClimbDeployer;
import frc.robot.subsystems.CoralIntake;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.GroundPivot;
// import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Tilter;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private Rotation2d objRot2d;

    /* Setting up bindings for necessary control of the swerve drive platform */
    // ==== 7890 make deadband much smaller than 0.1 ====
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // === Controllers === \\
    private final CommandXboxController objDriverXbox = new CommandXboxController(0);
    private final CommandXboxController objCoPilotXbox = new CommandXboxController(1);
    private final CommandGenericHID objButtonBoxA = new CommandGenericHID(2);
    private final CommandGenericHID objButtonBoxB = new CommandGenericHID(3);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // === SUBSYSTEM OBJECTS === \\
    SendableChooser<Command> objChooser = new SendableChooser<>();

    private final AlgaeIntake objAlgaeIntake = new AlgaeIntake();
    private final CoralIntake objCoralIntake = new CoralIntake();
    private final Elevator objElevator = new Elevator();
    private final Tilter objTilter = new Tilter();
    // private final Climber objClimber = new Climber();
    // private final ClimbDeployer objClimbDeployer = new ClimbDeployer();
    private final GroundIntake objGroundIntake = new GroundIntake();
    private final GroundPivot objGroundPivot = new GroundPivot();
    

    // === AUTONOMOUS OBJECTS ===\\
    private final SequentialCommandGroup objLeaveBlue = new Auto_MoveForwardBlue(drivetrain, MaxSpeed, MaxAngularRate);
    private final SequentialCommandGroup objLeaveRed = new Auto_MoveForwardRed(drivetrain, MaxSpeed, MaxAngularRate);
    private final SequentialCommandGroup objL4AndBarge = new Auto_MoveForwardAndGetAlg(drivetrain, objTilter, objElevator, objAlgaeIntake, objCoralIntake, MaxSpeed, MaxAngularRate);
    private final SequentialCommandGroup objL4AndBargeQuickBlue = new Auto_MoveForwardAndGetAlgwithouthomeBlue(drivetrain, objTilter, objElevator, objAlgaeIntake, objCoralIntake, objGroundPivot, objGroundIntake, MaxSpeed, MaxAngularRate);
    private final SequentialCommandGroup objL4AndBargeQuickRed = new Auto_MoveForwardAndGetAlgwithouthomeRed(drivetrain, objTilter, objElevator, objAlgaeIntake, objCoralIntake, objGroundPivot, objGroundIntake, MaxSpeed, MaxAngularRate);
    private final SequentialCommandGroup objDSAlgaeRed = new Auto_MoveForwardGetAlgInfrontofDSRed(drivetrain, objTilter, objElevator, objAlgaeIntake, objCoralIntake, MaxSpeed, MaxAngularRate);
    
    private final SequentialCommandGroup objDSAlgaeBlue = new Auto_MoveForwardGetAlgInfrontofDSBlue(drivetrain, objTilter, objElevator, objAlgaeIntake, objCoralIntake, MaxSpeed, MaxAngularRate);
        /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);

        // NamedCommands.registerCommand("coral auto test", Commands.print("Test registerCommand"));
        // new EventTrigger("test event algae").whileTrue(Commands.print("Test EventTrigger"));

        objChooser.setDefaultOption("Leave Blue", objLeaveBlue);
        objChooser.addOption("Leave Red", objLeaveRed);
        
        objChooser.addOption("L4 and Barge Blue", objL4AndBargeQuickBlue);
        objChooser.addOption("L4 And Barge Red", objL4AndBargeQuickRed);

        objChooser.addOption("Red Driver Station Algae", objDSAlgaeRed);
        objChooser.addOption("Blue Driver Station Algae", objDSAlgaeBlue);

        Shuffleboard.getTab("7890 Autos").add(objChooser);

        configureBindings();
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
                // === CTRE Drive === \\
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(Math.pow(-objDriverXbox.getLeftY(), 3.0) * MaxSpeed) // Drive forward with negative Y (forward)
            //         .withVelocityY(Math.pow(-objDriverXbox.getLeftX(), 3.0) * MaxSpeed) // Drive left with negative X (left)
            //         .withRotationalRate(-objDriverXbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            // )
            // === Manual Drive Control === \\
            new TeleOpDrive(drivetrain, MaxSpeed, MaxAngularRate,
                    ()-> -objDriverXbox.getLeftY(),
                    ()-> -objDriverXbox.getLeftX() ,
                    ()-> -objDriverXbox.getRightX(),
                false)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        // objDriverXbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // objDriverXbox.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-objDriverXbox.getLeftY(), -objDriverXbox.getLeftX()))
        // ));

        // reset the field-centric heading on start
        objDriverXbox.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // =========================================
        // === 7890 default commands and buttons ===
        // =========================================

        // ===============================================================================
        // === default commands for tilter and elevator allow joystick to fine tune it ===
        // === default command for coral intake runs slowly in to hold coral in place ===
        // === default command for ground pivot allows joystick to fine tune it ===
        // ===============================================================================
        // === button box joystick left/right runs tilter
        objTilter.setDefaultCommand(new RunTilter(objTilter, ()-> objButtonBoxA.getRawAxis(1), true, 0.0));
        // === button box joystick up/down runs elevator
        objElevator.setDefaultCommand(new RunElevator(objElevator, () -> objButtonBoxA.getRawAxis(0), true, 0.0));
        // === no button to hold coral
        objCoralIntake.setDefaultCommand(new RunCommand(()-> objCoralIntake.ShootCoral(Constants.MechSpeeds.dCoralHold), objCoralIntake));
        // === copilot xbox right joystick up/down runs ground pivot
        objGroundPivot.setDefaultCommand(new RunGroundPivot(objGroundPivot, () -> objCoPilotXbox.getRightY(), true, 0.0));

        // === driver Y is shoot Algae ===
        objDriverXbox.y().whileTrue(new ShootAlgae(objAlgaeIntake, Constants.MechSpeeds.dAlgaeShoot)); //OUT
        // === driver A is to do a barge shot (tilt up and shoot to give algae more velocity up)
        objDriverXbox.a().whileTrue(new BargeShoot(objTilter, objAlgaeIntake));
        // === driver X is shoot Coral ===
        objDriverXbox.x().whileTrue(new ShootCoral(objCoralIntake, Constants.MechSpeeds.dCoralShoot)); //OUT
        // === driver LEFT BUMPER is slow mode ===
        objDriverXbox.leftBumper().toggleOnFalse(new TeleOpDrive(drivetrain, MaxSpeed, MaxAngularRate,  
                    ()-> -objDriverXbox.getLeftY(),
                    ()-> -objDriverXbox.getLeftX(), 
                    ()-> -objDriverXbox.getRightX(), 
                true));

        // === COPILOT BUTTON BOX commands ===
        objButtonBoxA.button(1).whileTrue(new RunCombinedTiltElev(objTilter, objElevator, 
            MechPos.dTiltAlgBarge, MechPos.dElevAlgBarge));
        objButtonBoxA.button(2).whileTrue(new RunCombinedTiltElev(objTilter, objElevator, 
            MechPos.dTiltAlgProc, MechPos.dElevAlgProc));
        objButtonBoxA.button(3).whileTrue(new RunCombinedTiltElev(objTilter, objElevator, 
            MechPos.dTiltAlgL3, MechPos.dElevALgL3));
        objButtonBoxA.button(4).whileTrue(new RunCombinedTiltElev(objTilter, objElevator, 
            MechPos.dTiltAlgL2, MechPos.dElevAlgL2));
        objButtonBoxA.button(5).whileTrue(new RunCombinedTiltElev(objTilter, objElevator, 
            MechPos.dTiltCorL4, MechPos.dElevCorL4));
        objButtonBoxA.button(6).whileTrue(new RunCombinedTiltElev(objTilter, objElevator, 
            MechPos.dTiltCorL3, MechPos.dElevCorL3));
        objButtonBoxA.button(7).whileTrue(new RunCombinedTiltElev(objTilter, objElevator, 
            MechPos.dTiltCorL2, MechPos.dElevCorL2));
        // objButtonBoxA.button(8).whileTrue(new RunCombinedTiltElev(objTilter, objElevator, MechPos.dTiltCorL1, MechPos.dElevCorL1));
        objButtonBoxA.button(8).whileTrue(new PositionForCoralL1(objTilter, objElevator, objGroundIntake, objGroundPivot, objCoralIntake));
        objButtonBoxA.button(9).whileTrue(new RunCombinedTiltElev(objTilter, objElevator, 
            MechPos.dTiltCorLoad, MechPos.dElevCorLoad));
        
        objButtonBoxA.button(11).whileTrue(new RunGroundPivot(objGroundPivot, null, false, 
            Constants.MechPos.dGroundPosHome));
        // objButtonBoxA.button(11).whileTrue(new RunCommand(()-> objGroundPivot.runGroundPivot(0.1), objGroundIntake)).onFalse(new RunCommand(()-> objGroundPivot.runGroundPivot(0.0), objGroundPivot));
        // objButtonBoxA.button(12).whileTrue(new RunCommand(()-> objGroundPivot.runGroundPivot(-0.1), objGroundIntake)).onFalse(new RunCommand(()-> objGroundPivot.runGroundPivot(0.0), objGroundPivot));        

        objButtonBoxB.button(1).whileTrue(new RunGroundPivot(objGroundPivot, null, false, 
            Constants.MechPos.dGroundPosDown));
        objButtonBoxB.button(2).whileTrue(new AlgaeIntakeRun(objAlgaeIntake, 
            Constants.MechSpeeds.dAlgaeIntake));
        objButtonBoxB.button(3).whileTrue(new GroundIntakeSafely(objTilter, objElevator, objGroundIntake, objGroundPivot, objCoralIntake));
        objButtonBoxB.button(4).whileTrue(new RunGroundIntake(objGroundIntake, MechSpeeds.dGroundEject));
        objButtonBoxB.button(5).whileTrue(new RunCombinedTiltElev(objTilter, objElevator, 
            MechPos.dTiltHome, MechPos.dElevHome));

        // objButtonBoxB.button(4).toggleOnTrue(
        // // objButtonBoxB.button(1).whileTrue(
        //         // new SequentialCommandGroup(
        //                  new InstantCommand(() -> objGroundPivot.moveToPositionMM(Constants.MechPos.dGroundPosDown)), 
        //                 new RunCombinedTiltElev(objTilter, objElevator, MechPos.dTiltHome, MechPos.dElevHome),
        //                 new WaitUntilCommand(() -> objGroundPivot.isAtPos() && objTilter.isAtPos()) 
        //         //         new GroundCoralIn(objGroundIntake, objCoralIntake, objTilter, objElevator, objGroundPivot)
        //         // )
        // );
        //             new ParallelCommandGroup(
        //                   new RunCommand(()-> objGroundIntake.runGroundIntake(-0.5), objGroundIntake),
        //                   new CoralIntakeRun(objCoralIntake, 0.5))
        //           .toggleOnFalse(new SequentialCommandGroup(
        //               new RunCombinedTiltElev(objTilter, objElevator, MechPos.dTiltGroundSafety, MechPos.dElevHome),
        //               new InstantCommand(()-> objGroundPivot.moveToPositionMM(Constants.MechPos.dGroundPosHome))));

        // objButtonBoxB.button(4).whileTrue
        //     (new CoralIntakeRun(objCoralIntake, 0.5)
        //         .alongWith(new RunCommand( ()-> objGroundIntake.runGroundIntake(0.5), objGroundIntake)));

        // === testing autos ===
        // objCoPilotXbox.x().whileTrue(new Auto_MoveForward(drivetrain, MaxSpeed, MaxAngularRate));
        // objCoPilotXbox.x().whileTrue(new Auto_MoveForwardGetAlgInfrontofDSRed(drivetrain, objTilter, objElevator, objAlgaeIntake, objCoralIntake, MaxSpeed, MaxAngularRate));
    }
    public void ResetGyroAtEndOfAuton(){
        objRot2d = drivetrain.getRotation3d().toRotation2d();
        drivetrain.resetRotation(objRot2d.plus(Rotation2d.kPi));
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        /* Run the path selected from the auto chooser */
        // return autoChooser.getSelected();
        return objChooser.getSelected();
    }
}
