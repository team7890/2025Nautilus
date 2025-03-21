// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;

public class CoralIntake extends SubsystemBase {

  private TalonFX objCoralIntake = new TalonFX(CANIds.iCoralIntake, "rio");
  private StatusCode objTalonFXStatusCode;
  private StatusSignal objStatSig;
  private double dCurrentNow;
  // private double dCurrentMax = 0.0;
  private boolean bCurrentTripped = false, bInrushDone = false;
  private int iCount = 0, iTripCount = 0;
  // private double dCurrMaxInrush = 0.0;
  private double dCurrentAvg = 0.0;



  /** Creates a new CoralIntake. */
  public CoralIntake() {

    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 100.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.04;
    objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;
    
    for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objCoralIntake.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
    }
  }

  @Override
  public void periodic() {
  }

  public void stopCoralIntake(){
    objCoralIntake.stopMotor();
  }

  public void runCoralIntake (double dSpeed){

    objStatSig = objCoralIntake.getTorqueCurrent();
    dCurrentNow = Math.abs(objStatSig.getValueAsDouble());
    dCurrentAvg = dCurrentAvg * 0.9 + dCurrentNow * 0.1;
    // following if is for debug
    // if (dCurrentNow > dCurrentMax) {
    //   dCurrentMax = dCurrentNow;
    // }

    if (iCount < 7) {
      iCount = iCount + 1;
      // dCurrMaxInrush = dCurrentNow;
    }
    else {
      bInrushDone = true;
    }

    if (bInrushDone) {
      // set trip bit if current is high and it hasn't yet tripped
      if (dCurrentAvg > 35.0 && !bCurrentTripped) {
        bCurrentTripped = true;
      }
      // if current has tripped, keep going for trip count and then go to low/hold speed
      // otherwise, keep going at full intake speed
      if (bCurrentTripped) {
        iTripCount = iTripCount + 1;
        if (iTripCount > 7) {
          objCoralIntake.set(Math.signum(dSpeed) * Constants.MechSpeeds.dCoralHold);
        }
        else {
          objCoralIntake.set(dSpeed);
        }
      }
      else {
        objCoralIntake.set(dSpeed);
      }
    }
    // objCoralIntake.set(dSpeed);

    // SmartDashboard.putNumber("Coral Current", dCurrentNow);
    // SmartDashboard.putNumber("Coral Max Current", dCurrentMax);
    // SmartDashboard.putNumber("Coral Current Inrush", dCurrMaxInrush);
    // SmartDashboard.putBoolean("Coral Inrush Done", bInrushDone);
    // SmartDashboard.putBoolean("Coral Tripped", bCurrentTripped);
    // SmartDashboard.putNumber("Coral Avg Current", dCurrentAvg);
  }

  public void resetCurrentTrip(){
    bCurrentTripped = false;
    iCount = 0;
    iTripCount = 0;
    bInrushDone = false;
    // dCurrentMax = 0.0;
    // dCurrMaxInrush = 0.0;
  }
  
  public void ShootCoral(double dSpeed){
    objCoralIntake.set(dSpeed);
  }
  
}
