// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class ClimbDeployer extends SubsystemBase {

  private TalonFX objClimbDeployer = new TalonFX(CANIds.iClimberDeploy, "canivore");
  private StatusCode objTalonFXStatusCode;



  /** Creates a new ClimbDeployer. */
  public ClimbDeployer() {

    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objClimbDeployer.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
    }
  }

  @Override
  public void periodic() {
  }

  public void stopClimbDeployer(){
    objClimbDeployer.stopMotor();
  }

  public void runClimbDeployer (double dSpeed){
    objClimbDeployer.set(dSpeed);
  }
}
