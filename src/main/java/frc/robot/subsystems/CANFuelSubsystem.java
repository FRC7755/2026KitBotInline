// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeLauncherRoller.getEncoder();
  SparkClosedLoopController spinPID = intakeLauncherRoller.getClosedLoopController();
  SparkMaxConfig configShooter = new SparkMaxConfig();
  SparkMaxConfig launcherConfig = new SparkMaxConfig();

  @SuppressWarnings("removal")
  public CANFuelSubsystem() {
    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Target RPM", SPIN_RPMS);
    SmartDashboard.putNumber("Current RPM", 0);
    SmartDashboard.putBoolean("RPM Achieved", false);
    SmartDashboard.putBoolean("Shooter Button", false);
    SmartDashboard.putNumber("Motor P", 0.0001);
    SmartDashboard.putNumber("Motor I", 0.0);
    SmartDashboard.putNumber("Motor D", 0.0);

    launcherConfig.inverted(false);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0001, 0.0, 0.0).velocityFF(0.00215);
    launcherConfig.encoder.velocityConversionFactor(1.0);

    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVelocity(double rpm){
    spinPID.setSetpoint(rpm,ControlType.kVelocity);
    SmartDashboard.putBoolean("Shooter Button", true);
  }

  public void launchVelocity(double rpm){
    spinPID.setSetpoint(rpm,ControlType.kVelocity);
    SmartDashboard.putBoolean("Shooter Button", true);
  }

  public void intake() {
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }
  public void eject() {
    intakeLauncherRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_INTAKE_VOLTAGE));
  }

  public void launch() {
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  public void stop() {
    intakeLauncherRoller.set(0);
    SmartDashboard.putBoolean("Shooter Button", false);
  }

  public void spinUp() {
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  public Command setVelocityCommand(double targetrpm) {
    return this.run(() -> setVelocity(targetrpm));
  }

  public Command launchVelocityCommand(double targetrpm) {
    return this.run(() -> launchVelocity(targetrpm));
  }

  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
    double currentRPM = intakeEncoder.getVelocity();
    if (currentRPM > (SPIN_RPMS - RPM_TOLERANCE) && currentRPM < (SPIN_RPMS + RPM_TOLERANCE)) {
      SmartDashboard.putBoolean("RPM Achieved", true);
    } else {
      SmartDashboard.putBoolean("RPM Achieved", false);
    }
    SmartDashboard.putNumber("Current RPM", currentRPM);
    SmartDashboard.putNumber("Motor P", intakeLauncherRoller.configAccessor.closedLoop.getP());
    SmartDashboard.putNumber("Motor I", intakeLauncherRoller.configAccessor.closedLoop.getI());
    SmartDashboard.putNumber("Motor D", intakeLauncherRoller.configAccessor.closedLoop.getD());
  }
}
