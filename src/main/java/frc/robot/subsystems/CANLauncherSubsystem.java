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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANLauncherSubsystem extends SubsystemBase {
  private final SparkMax launcherRoller = new SparkMax(LAUNCHER_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder launcherEncoder = launcherRoller.getEncoder();
  SparkClosedLoopController launcherPID = launcherRoller.getClosedLoopController();
  SparkMaxConfig launcherConfig = new SparkMaxConfig();

  @SuppressWarnings("removal")
  public CANLauncherSubsystem() {
    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
//    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
//    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Launcher Target RPM", LAUNCHER_RPMS);
    SmartDashboard.putNumber("Launcher Slow Target RPM", LAUNCHER_SLOW_RPMS);
    SmartDashboard.putNumber("Launcher Current RPM", 0);
    SmartDashboard.putBoolean("Launcher RPM Achieved", false);
    SmartDashboard.putBoolean("Launcher Button", false);
    SmartDashboard.putNumber("Launcher Motor P", 0.0001);
    SmartDashboard.putNumber("Launcher Motor I", 0.0);
    SmartDashboard.putNumber("Launcher Motor D", 0.0);

    launcherConfig.inverted(false);
    launcherConfig.idleMode(IdleMode.kCoast);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0001, 0.0, 0.0).velocityFF(0.00215);
    launcherConfig.encoder.velocityConversionFactor(1.0);

    launcherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void launcherRun(double intakeRpm){
    launcherPID.setSetpoint(intakeRpm,ControlType.kVelocity);
    SmartDashboard.putBoolean("Launcher Button", true);
  }

  public void launcherStop() {
    launcherRoller.set(0);
    SmartDashboard.putBoolean("Launcher Button", false);
  }

  public Command launcherRunCommand(double targetrpm) {
    return this.run(() -> launcherRun(targetrpm));
  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
    double launcherRPM = launcherEncoder.getVelocity();
    if (launcherRPM > (LAUNCHER_RPMS - RPM_TOLERANCE) && launcherRPM < (LAUNCHER_RPMS + RPM_TOLERANCE)) {
      SmartDashboard.putBoolean("Launcher RPM Achieved", true);
    } else {
      SmartDashboard.putBoolean("Launcher RPM Achieved", false);
    }
    launcherRPM = Math.round(launcherRPM *1.0)/1.0;
    SmartDashboard.putNumber("Launcher Current RPM", launcherRPM);
    SmartDashboard.putNumber("Launcher Motor P", launcherRoller.configAccessor.closedLoop.getP());
    SmartDashboard.putNumber("Launcher Motor I", launcherRoller.configAccessor.closedLoop.getI());
    SmartDashboard.putNumber("Launcher Motor D", launcherRoller.configAccessor.closedLoop.getD());
  }
}
