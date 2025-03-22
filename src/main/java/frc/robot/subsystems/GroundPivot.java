// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;

public class GroundPivot extends SubsystemBase {

  // source of info on coding this subsystem with two motors not directly coupled through a common gearbox
  //   https://www.chiefdelphi.com/t/the-follower-control-logic-of-ctre-is-not-working/491787/11
  // control options are:
  //  (1) leader/follower with the follower in coast mode (don't want to do this - want brake mode on both to hold GroundPivot)
  //  (2) independent motion magic
  //  (3) differential mechanism

  StatusSignal objStatSigA, objStatSigB;
  private double dTargetPos;
 
  private TalonFX objGroundPivot = new TalonFX(Constants.CANIds.iGroundPivot, "canivore");

  private TalonFXConfiguration objConfigEachMotor;
  private MotionMagicConfigs objMMConfig;
  private MotionMagicVoltage objMMV = new MotionMagicVoltage(0);
  // private Mechanism objMechanism = new Mechanisms();
  private StatusCode objStatus;


  private double dLastKnownPos;
  // private final double dGearRatio = (58.0 / 10.0 ) * (58.0 / 18) * (80.0 / 22.0);



  /** Creates a new GroundPivot. */
  public GroundPivot() {
    // === configs for each motor individually ===
    objConfigEachMotor = new TalonFXConfiguration();
    // current limit
    objConfigEachMotor.CurrentLimits.SupplyCurrentLimit = 60.0;
    objConfigEachMotor.CurrentLimits.SupplyCurrentLimitEnable = true;
    objConfigEachMotor.CurrentLimits.StatorCurrentLimit = 120.0;
    objConfigEachMotor.CurrentLimits.StatorCurrentLimitEnable = true;
    objConfigEachMotor.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    objConfigEachMotor.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    // brake mode
    objConfigEachMotor.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // open loop ramp so it doesn't jerk
    objConfigEachMotor.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    // config software position limits so sysid doesn't break the GroundPivot
    objConfigEachMotor.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    objConfigEachMotor.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 23.0;
    objConfigEachMotor.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    objConfigEachMotor.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    // not sure yet why using this but was in example code
    objConfigEachMotor.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    FeedbackConfigs objFeedbackConfigs = objConfigEachMotor.Feedback;
    //objFeedbackConfigs.SensorToMechanismRatio = dGearRatio;
    objFeedbackConfigs.SensorToMechanismRatio = 1.0;

    // congigure motion magic
    objMMConfig = objConfigEachMotor.MotionMagic;
    objMMConfig
      .withMotionMagicCruiseVelocity(RotationsPerSecond.of(200))  //50 (mechanim) rotations per second cruise
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(400)) // take approximatley 0.5 secs to reach max vel
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(300)); // take approximately 0.1 secs to reach max accel
    
    Slot0Configs objSlot0 = objConfigEachMotor.Slot0;
    objSlot0.kG = 0.15;
    objSlot0.kS = 0.15; // Add 0.25 V output to overcome static friction
    objSlot0.kV = 0.09; // v1.0- kG+kS =0.3 , result 15 revoluations 4.5 sec , volt to move/rotations/sec .3/(15/4.5) = .09
    objSlot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output, use defult value of 0.01
    objSlot0.kP = 40.0; // A position error of 0.2 rotations results in 12 V output
    objSlot0.kI = 0.1; // No output for integrated error
    objSlot0.kD = 0.0; // A velocity error of 1 rps results in 0.5 V output
    //objSlot0.GravityType = GravityTypeValue.Arm_Cosine;
    objSlot0.GravityType = GravityTypeValue.Elevator_Static;
    objSlot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    //voltage
    objConfigEachMotor.Voltage.PeakForwardVoltage = 16.0;
    objConfigEachMotor.Voltage.PeakReverseVoltage = -16.0;
    objConfigEachMotor.Voltage.SupplyVoltageTimeConstant = 0.0;

    //Open Loop Ramps
    objConfigEachMotor.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
    objConfigEachMotor.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    objConfigEachMotor.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0.0;

    //Closed Loop Ramps
    objConfigEachMotor.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
    objConfigEachMotor.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    objConfigEachMotor.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.0;

    //Hardware limit switch off
    objConfigEachMotor.HardwareLimitSwitch.ForwardLimitEnable =false;
    objConfigEachMotor.HardwareLimitSwitch.ReverseLimitEnable = false;

    //sofware limit switch
    objConfigEachMotor.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 33.0;
    objConfigEachMotor.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;


    

    objStatus = StatusCode.StatusCodeNotInitialized;
    // apply configs for each motor individually
    for (int i = 1; i < 5; i++) {
      objStatus = objGroundPivot.getConfigurator().apply(objConfigEachMotor);
      if (objStatus.isOK()) break;
    }
   // objConfigEachMotor.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // apply configs for each motor individually
    }
    

  @Override
  public void periodic() {
    SmartDashboard.putNumber("GroundPivot Position", getGroundPivotPos());
    SmartDashboard.putNumber("GroundPivot Target Position", dTargetPos);
  }

  public void moveToPositionMM(double dTarget){
    objGroundPivot.setControl(objMMV.withPosition(dTarget).withSlot(0));
    dTargetPos = dTarget;
  }


  public double getGroundPivotPos() {
    objStatSigA = objGroundPivot.getPosition();
    return objStatSigA.getValueAsDouble();
  }

  public double getGroundPivotVel(){
    objStatSigA = objGroundPivot.getVelocity();
    return objStatSigA.getValueAsDouble();
  }

  public Command runGroundPivotMethodCmd(double dSpeed) {
    return new RunCommand(
      () -> this.runGroundPivot(dSpeed), this
    );
  }

  public Command stopGroundPivotMethodCmd() {
    return new RunCommand(
      () -> this.stopGroundPivot(), this
    );
  }

  public void runGroundPivot(double dspeed) {
    objGroundPivot.set(dspeed);
  }

  public void stopGroundPivot() {
    objGroundPivot.stopMotor();
  }


  public void setLastKnownPos(){
    dLastKnownPos = getGroundPivotPos();
  }

  public double getLastKownPos(){
    return dLastKnownPos;
  }

  public void resetPos(){
    objGroundPivot.setPosition(0.0);
  }

  public boolean isAtPos(){
    if (Math.abs(getGroundPivotPos() - dTargetPos) < 0.05) {
      return true;
    }
    else{
      return false;
    }
  }


}
