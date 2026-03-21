// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.config.ClosedLoopConfig.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFeederSubsystem extends SubsystemBase {
  private final SparkMax feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder feederEncoder = feederRoller.getEncoder();
  SparkClosedLoopController feederPID = feederRoller.getClosedLoopController();
  SparkMaxConfig feederConfig = new SparkMaxConfig();

  @SuppressWarnings("removal")
  public CANFeederSubsystem() {
    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
//    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
//    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
//    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Feeder Target RPM", LAUNCHER_RPMS);
    SmartDashboard.putNumber("Feeder Current RPM", 0);
    SmartDashboard.putBoolean("Feeder RPM Achieved", false);
    SmartDashboard.putBoolean("Feeder Button", false);
    SmartDashboard.putNumber("Feeder Motor P", 0.0001);
    SmartDashboard.putNumber("Feeder Motor I", 0.0);
    SmartDashboard.putNumber("Feeder Motor D", 0.0);

    feederConfig.inverted(true);
    feederConfig.idleMode(IdleMode.kBrake);
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0001, 0.0, 0.0).velocityFF(0.00215);
    feederConfig.encoder.velocityConversionFactor(1.0);

    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void feederRun(double feederRpm){
    feederPID.setSetpoint(feederRpm,ControlType.kVelocity);
    SmartDashboard.putBoolean("Intake Button", true);
  }

  public Command feederRunCommand(double targetrpm) {
    return this.run(() -> feederRun(targetrpm));
  }

  public void feederStop() {
    feederRoller.set(0);
  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
    double feederRPM = feederEncoder.getVelocity();
    if (feederRPM > (FEEDER_RPMS - RPM_TOLERANCE) && feederRPM < (FEEDER_RPMS + RPM_TOLERANCE)) {
      SmartDashboard.putBoolean("Feeder RPM Achieved", true);
    } else {
      SmartDashboard.putBoolean("Feeder RPM Achieved", false);
    }
    feederRPM = Math.round(feederRPM *1.0)/1.0;
    SmartDashboard.putNumber("Feeder Current RPM", feederRPM);
    SmartDashboard.putNumber("Feeder Motor P", feederRoller.configAccessor.closedLoop.getP());
    SmartDashboard.putNumber("Feeder Motor I", feederRoller.configAccessor.closedLoop.getI());
    SmartDashboard.putNumber("Feeder Motor D", feederRoller.configAccessor.closedLoop.getD());
  }
}
