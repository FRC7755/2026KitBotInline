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

public class CANIntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeRoller = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeRoller.getEncoder();
  SparkClosedLoopController intakePID = intakeRoller.getClosedLoopController();
  SparkMaxConfig intakeConfig = new SparkMaxConfig();

  @SuppressWarnings("removal")
  public CANIntakeSubsystem() {
    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
//    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
//    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
//    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intake Target RPM", INTAKE_RPMS);
    SmartDashboard.putNumber("Intake Current RPM", 0);
    SmartDashboard.putBoolean("Intake RPM Achieved", false);
    SmartDashboard.putBoolean("Intake Button", false);
    SmartDashboard.putNumber("Intake Motor P", 0.0001);
    SmartDashboard.putNumber("Intake Motor I", 0.0);
    SmartDashboard.putNumber("Intake Motor D", 0.0);

    intakeConfig.inverted(true);
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0001, 0.0, 0.0).velocityFF(0.00215);
    intakeConfig.encoder.velocityConversionFactor(1.0);

    intakeRoller.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intakeRunIn(){
    intakePID.setSetpoint(SmartDashboard.getNumber("Intake Target RPM", INTAKE_RPMS),ControlType.kVelocity);
    SmartDashboard.putBoolean("Intake Button", true);
  }

  public Command intakeRunInCommand() {
    return this.run(() -> intakeRunIn());
  }

  public void intakeRunOut(){
    intakePID.setSetpoint(SmartDashboard.getNumber("Intake Target RPM", INTAKE_RPMS) * -1,ControlType.kVelocity);
    SmartDashboard.putBoolean("Intake Button", true);
  }

  public Command intakeRunOutCommand() {
    return this.run(() -> intakeRunOut());
  }

  public void intakeStop() {
    intakeRoller.set(0);
    SmartDashboard.putBoolean("Intake Button", false);
  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
    double intakeRPM = intakeEncoder.getVelocity();
    if (intakeRPM > (INTAKE_RPMS - RPM_TOLERANCE) && intakeRPM < (INTAKE_RPMS + RPM_TOLERANCE)) {
      SmartDashboard.putBoolean("Intake RPM Achieved", true);
    } else {
      SmartDashboard.putBoolean("Intake RPM Achieved", false);
    }
    intakeRPM = Math.round(intakeRPM *1.0)/1.0;
    SmartDashboard.putNumber("Intake Current RPM", intakeRPM);
    SmartDashboard.putNumber("Intake Motor P", intakeRoller.configAccessor.closedLoop.getP());
    SmartDashboard.putNumber("Intake Motor I", intakeRoller.configAccessor.closedLoop.getI());
    SmartDashboard.putNumber("Intake Motor D", intakeRoller.configAccessor.closedLoop.getD());
  }
}
