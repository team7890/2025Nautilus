// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.MechPos;

// === COMMANDS === \\
import frc.robot.commands.MechCommands.RunElevator;
import frc.robot.commands.TeleOpDrive;
import frc.robot.commands.MechCommands.AlgaeIntakeRun;
import frc.robot.commands.MechCommands.CoralIntakeRun;
import frc.robot.commands.MechCommands.RunTilter;
import frc.robot.commands.MechCommands.RunClimber;

// === Group Commands ===
import frc.robot.commands.MechGroupCommands.MoveMechToHome;
import frc.robot.commands.MechGroupCommands.MoveMechToPosition;
import frc.robot.commands.MechGroupCommands.TiltElevSafety;

// === SUBSYSTEMS === \\
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.ClimbDeployer;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Tilter;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
    private final AlgaeIntake objAlgaeIntake = new AlgaeIntake();
    private final CoralIntake objCoralIntake = new CoralIntake();
    private final Elevator objElevator = new Elevator();
    private final Tilter objTilter = new Tilter();
    private final Climber objClimber = new Climber();
    private final ClimbDeployer objClimbDeployer = new ClimbDeployer();
        /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);

        // NamedCommands.registerCommand("coral auto test", Commands.print("Test registerCommand"));
        // new EventTrigger("test event algae").whileTrue(Commands.print("Test EventTrigger"));

        configureBindings();
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Math.pow(-objDriverXbox.getLeftY(), 3.0) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(Math.pow(-objDriverXbox.getLeftX(), 3.0) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-objDriverXbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
            // === Manual Drive Control === \\
            // new TeleOpDrive(drivetrain, MaxSpeed, MaxAngularRate,
            //         ()-> -objDriverXbox.getLeftY(),
            //         ()-> -objDriverXbox.getLeftX() ,
            //         ()-> -objDriverXbox.getRightX(),
            //     false)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        objDriverXbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        objDriverXbox.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-objDriverXbox.getLeftY(), -objDriverXbox.getLeftX()))
        ));

        // reset the field-centric heading on start
        objDriverXbox.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // ====================
        // === 7890 buttons ===
        // ====================

        // === driver Y is shoot Algae ===
        objDriverXbox.y().whileTrue(new AlgaeIntakeRun(objAlgaeIntake, 1.0)); //OUT
        // === driver X is shoot Coral ===
        objDriverXbox.x().whileTrue(new CoralIntakeRun(objCoralIntake, 0.25)); //OUT

        // === default commands for tilter and elevator allow joystick to fine tune it ===
        objTilter.setDefaultCommand(new RunTilter(objTilter, ()-> objButtonBoxA.getRawAxis(1), true, 0.0));
        objElevator.setDefaultCommand(new RunElevator(objElevator, () -> objButtonBoxA.getRawAxis(0), true, 0.0));

        objButtonBoxA.button(1).whileTrue(new TiltElevSafety(objTilter, objElevator, MechPos.dTiltAlgBarge, MechPos.dElevAlgBarge)); //k1
        objButtonBoxA.button(2).whileTrue(new TiltElevSafety(objTilter, objElevator, MechPos.dTiltAlgProc, MechPos.dElevAlgProc)); //k1
        objButtonBoxA.button(3).whileTrue(new TiltElevSafety(objTilter, objElevator, MechPos.dTiltAlgL3, MechPos.dElevALgL3)); //k1
        objButtonBoxA.button(4).whileTrue(new TiltElevSafety(objTilter, objElevator, MechPos.dTiltAlgL2, MechPos.dElevAlgL2)); //k1
        objButtonBoxA.button(5).whileTrue(new TiltElevSafety(objTilter, objElevator, MechPos.dTiltCorL4, MechPos.dElevCorL4)); //k1
        objButtonBoxA.button(6).whileTrue(new TiltElevSafety(objTilter, objElevator, MechPos.dTiltCorL3, MechPos.dElevCorL3)); //k1
        objButtonBoxA.button(7).whileTrue(new TiltElevSafety(objTilter, objElevator, MechPos.dTiltCorL2, MechPos.dElevCorL2)); //k1
        objButtonBoxA.button(8).whileTrue(new TiltElevSafety(objTilter, objElevator, MechPos.dTiltCorL1, MechPos.dElevCorL1)); //k1
        objButtonBoxA.button(9).whileTrue(new TiltElevSafety(objTilter, objElevator, MechPos.dTiltCorLoad, MechPos.dElevCorLoad)); //k1
        objButtonBoxA.button(12).whileTrue(new RunClimber(objClimber, objClimbDeployer, true));
       
        objButtonBoxB.button(1).whileTrue(new RunClimber(objClimber, objClimbDeployer, false));
        objButtonBoxB.button(2).whileTrue(new AlgaeIntakeRun(objAlgaeIntake, -0.5)); //k1
        objButtonBoxB.button(3).whileTrue(new CoralIntakeRun(objCoralIntake, -0.35));
        objButtonBoxB.button(5).whileTrue(new TiltElevSafety(objTilter, objElevator, MechPos.dTiltHome, MechPos.dElevHome));
   
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
        /* Run the path selected from the auto chooser */
        // return autoChooser.getSelected();
    }
}
