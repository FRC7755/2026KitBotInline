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
import static frc.robot.Constants.ClimberConstants.*;

public class CANClimberSubsystem extends SubsystemBase {
  private final SparkMax climber = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);
  SparkMaxConfig climberConfig = new SparkMaxConfig();

  @SuppressWarnings("removal")
  public CANClimberSubsystem() {
    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Climber Voltage", CLIMBER_VOLTAGE);

    climberConfig.inverted(true);
    climberConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
    climber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void climberup() {
    climber.setVoltage(SmartDashboard.getNumber("Climber Voltage", CLIMBER_VOLTAGE));
  }
  
  public void climberdown() {
    climber.setVoltage(-1 * SmartDashboard.getNumber("Climber Voltage", CLIMBER_VOLTAGE));
  }
  
  // A method to stop the rollers
  public void stop() {
    climber.set(0);
  }

  @Override
  public void periodic() {
  }
}
