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
  private final SparkMax feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeLauncherRoller.getEncoder();
  SparkClosedLoopController spinPID = intakeLauncherRoller.getClosedLoopController();
  SparkMaxConfig configShooter = new SparkMaxConfig();
  SparkMaxConfig feederConfig = new SparkMaxConfig();
  SparkMaxConfig launcherConfig = new SparkMaxConfig();
  /** Creates a new CANBallSubsystem. */

  @SuppressWarnings("removal")
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    //configShooter.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.3, 0.2, 0.1).velocityFF(0.00018);
    //intakeLauncherRoller.configure(configShooter,SparkMax.ResetMode.kResetSafeParameters,SparkMax.PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Target RPM", SPIN_RPMS);
    SmartDashboard.putNumber("Current RPM", 0);
    SmartDashboard.putBoolean("RPM Achieved", false);
    SmartDashboard.putBoolean("Shooter Button", false);
    SmartDashboard.putNumber("Motor P", 0.0001);
    SmartDashboard.putNumber("Motor I", 0.0);
    SmartDashboard.putNumber("Motor D", 0.0);


    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    feederConfig.inverted(true);
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    launcherConfig.inverted(false);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0001, 0.0, 0.0).velocityFF(0.0021);
    launcherConfig.encoder.velocityConversionFactor(1.0);

    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVelocity(double rpm){
    System.out.println(rpm);
    spinPID.setSetpoint(rpm,ControlType.kVelocity);
    SmartDashboard.putBoolean("RPM Achieved", spinPID.isAtSetpoint());
    SmartDashboard.putBoolean("Shooter Button", true);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
    SmartDashboard.putBoolean("Shooter Button", false);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
  //  feederRoller
  //      .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command setVelocityCommand(double targetrpm) {
    return this.run(() -> setVelocity(targetrpm));
  }

  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
    double currentRPM = intakeEncoder.getVelocity();
    SmartDashboard.putNumber("Current RPM", currentRPM);
    SmartDashboard.putNumber("Motor P", intakeLauncherRoller.configAccessor.closedLoop.getP());
    SmartDashboard.putNumber("Motor I", intakeLauncherRoller.configAccessor.closedLoop.getI());
    SmartDashboard.putNumber("Motor D", intakeLauncherRoller.configAccessor.closedLoop.getD());
  }
}
