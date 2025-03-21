// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CANIds;

// public class Climber extends SubsystemBase {

//   private TalonFX objClimberA = new TalonFX(CANIds.iClimberWinchA, "canivore");
//   private TalonFX objClimberB = new TalonFX(CANIds.iClimberWinchB, "canivore");
//   private StatusCode objTalonFXStatusCode;



//   /** Creates a new Climber. */
//   public Climber() {

//     TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
//     objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
//     objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//     objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//     objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
//     objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;
//     for (int i = 1; i < 5; i++) {
//       objTalonFXStatusCode = objClimberA.getConfigurator().apply(objTalonFXConfig);
//       if (objTalonFXStatusCode.isOK()) break;
//     }
//     for (int i = 1; i < 5; i++) {
//       objTalonFXStatusCode = objClimberB.getConfigurator().apply(objTalonFXConfig);
//       if (objTalonFXStatusCode.isOK()) break;
//     }
//   }

//   @Override
//   public void periodic() {
//   }

//   public void stopClimber(){
//     objClimberA.stopMotor();
//     objClimberB.stopMotor();
//   }

//   public void runClimber (double dSpeed){
//     objClimberA.set(dSpeed);
//     objClimberB.set(-dSpeed);
//   }
// }
