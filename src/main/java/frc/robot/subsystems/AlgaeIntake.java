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
import frc.robot.Constants.CANIds;

public class AlgaeIntake extends SubsystemBase {

  private TalonFX objAlgaeIntake = new TalonFX(CANIds.iAlgaeIntake, "rio");
  private StatusCode objTalonFXStatusCode;
  private StatusSignal objStatSig;
  private double dCurrentNow, dCurrentMax = 0.0;
  private boolean bCurrentTripped = false, bInrushDone = false;
  private int iCount = 0;
  private double dCurrMaxInrush = 0.0, dCurrentAvg = 0.0;



  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {

    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 100.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.04;
    objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;
    
    for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objAlgaeIntake.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
    }
  }

  @Override
  public void periodic() {
  }

  public void stopAlgaeIntake(){
    objAlgaeIntake.stopMotor();
  }

  public void runAlgaeIntake (double dSpeed){

    // objAlgaeIntake.set(dSpeed);

    objStatSig = objAlgaeIntake.getTorqueCurrent();
    dCurrentNow = Math.abs(objStatSig.getValueAsDouble());
    dCurrentAvg = dCurrentAvg * 0.9 + dCurrentNow * 0.1;
    if (dCurrentNow > dCurrentMax) {
      dCurrentMax = dCurrentNow;
    }
//====================
    if (iCount < 7) {
      iCount = iCount + 1;
      dCurrMaxInrush = dCurrentNow;
    }
    else {
      bInrushDone = true;
      
    }

    if (bInrushDone) {
      if (dCurrentAvg > 35.0 && !bCurrentTripped) {
        bCurrentTripped = true;
      }

      if (bCurrentTripped) {
        objAlgaeIntake.set(Math.signum(dSpeed) * 0.05);  
      }
      else {
        objAlgaeIntake.set(dSpeed);
      }
    }
    // objAlgaeIntake.set(dSpeed);

    SmartDashboard.putNumber("Algae Current", dCurrentNow);
    SmartDashboard.putNumber("Algae Max Current", dCurrentMax);
    SmartDashboard.putNumber("Algae Current Inrush", dCurrMaxInrush);
    SmartDashboard.putBoolean("Algae Inrush Done", bInrushDone);
    SmartDashboard.putBoolean("Algae Tripped", bCurrentTripped);
    SmartDashboard.putNumber("Algae Avg Current", dCurrentAvg);
  }

  public void resetCurrentTrip(){
    bCurrentTripped = false;
    iCount = 0;
    bInrushDone = false;
    dCurrentMax = 0.0;
    dCurrMaxInrush = 0.0;
  }
  public void ShootAlgae(double dSpeed){
    objAlgaeIntake.set(dSpeed);
  }
  
}
