// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class CANDriveSubsystem extends SubsystemBase {
  private final WPI_VictorSPX leftLeader;
  private final WPI_VictorSPX leftFollower;
  private final WPI_VictorSPX rightLeader;
  private final WPI_VictorSPX rightFollower;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new WPI_VictorSPX(LEFT_LEADER_ID);
    leftFollower = new WPI_VictorSPX(LEFT_FOLLOWER_ID);
    rightLeader = new WPI_VictorSPX(RIGHT_LEADER_ID);
    rightFollower = new WPI_VictorSPX(RIGHT_FOLLOWER_ID);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Send motor controller info to LiveWindow
    SendableRegistry.addChild(drive, leftLeader);
    SendableRegistry.addChild(drive, rightLeader);

    // Default controllers to prevent unexpected behavior
    leftLeader.configFactoryDefault();
    rightLeader.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightFollower.configFactoryDefault();

    // Invert right side motors
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);
    
    // Set rear motors to follow front motors
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    
    // Set Brake mode on all controllers
    rightLeader.setNeutralMode(NeutralMode.Brake);
    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }
}
